#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "bsp_board_extra.h"
#include "lvgl.h"
#include "ui.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "esp_timer.h"
#include "odometer/odometer.h"

//-----Pin Assignment---------//

// CAN bus (MCP2551 transceiver)
// NOTE: MCP2551 RXD output is 5V — use 10kΩ+20kΩ voltage divider before ESP32 RX pin.
//       MCP2551 TXD input accepts 3.3V logic directly from ESP32.
#define CAN_TX_GPIO  GPIO_NUM_0  // ESP32 TX → MCP2551 TXD
#define CAN_RX_GPIO  GPIO_NUM_0  // MCP2551 RXD → (voltage divider) → ESP32 RX
#define CAN_RX_QUEUE_LEN 32

// ZombieVerter mapped CAN messages
// Direction: Zombie "dir" spot value, TX map: pos 0, 8 bits, gain 1
//            -1=R  0=N  1=D  (2=Park shown as N)
#define ZOMBIE_DIR_CAN_ID    0x7F0
#define ZOMBIE_DIR_BYTE      0
// Road speed: Zombie "Veh_Speed" (kph), TX map: pos 0, 16 bits, gain 1
#define ZOMBIE_SPEED_CAN_ID  0x7F1
#define ZOMBIE_SPEED_BYTE    0
#define ZOMBIE_SPEED_GAIN    1.0f   // raw * gain = speed in km/h (or mph, see below)
#define ZOMBIE_SPEED_IN_KMH  1      // 1 = value is km/h, 0 = value is already mph

//--------------------------//

// Gear pill index from Zombie dir: 0=R 1=N 2=D  (default N until first CAN frame)
static volatile int      g_can_gear_idx = 1;
// RPM from CAN ID 0x1DA bytes 4-5 (Leaf inverter: big-endian int16, raw / 2 = RPM)
static volatile uint32_t g_can_rpm      = 0;
static volatile uint32_t g_can_rpm_ms   = 0;     // time of last RPM frame
#define RPM_TIMEOUT_MS  500    // no RPM frame for this long → RPM = 0

// Backlight levels
#define BL_NORMAL  100
#define BL_DIM      50

// Sidelights (from CAN 0x801)
static volatile bool g_sidelights_on = false;

//-------------ROAD SPEED (ZombieVerter road speed over CAN)-------------//
#define SPEED_MAX_MPH        150.0f   // display clamp; above this is not road speed
#define SPEED_TIMEOUT_MS     1000     // no speed frame for 1 s → 0 mph
#define SPEED_FILTER_ALPHA   0.20f    // EMA, ~250 ms settle at the 20 Hz task rate
#define METRES_PER_MILE      1609.344f

static volatile float    g_can_speed_mph = 0.0f;  // latest value from CAN
static volatile uint32_t g_can_speed_ms  = 0;     // time of last speed frame

volatile float g_speed_mph = 0.0f;                // filtered, for display

//------------------------------//

static lv_color_t green_color;
static lv_color_t orange_color;

static void init_label_styles(void){

    green_color = lv_color_hex(0x28FF00);
    orange_color = lv_palette_main(LV_PALETTE_ORANGE);
}

static void update_label_if_needed(lv_obj_t *label, char *new_value, lv_color_t new_color) {
    // Only update text if changed
    const char *old_text = lv_label_get_text(label);
    if (strcmp(old_text, new_value) != 0) {
        lv_label_set_text(label, new_value);
    }
    // Only update color if changed
    lv_color_t old_color = lv_obj_get_style_text_color(label, LV_PART_MAIN);
    if (old_color.full != new_color.full) {
        lv_obj_set_style_text_color(label, new_color, LV_PART_MAIN);
    }
}

//-----------Road speed / odometer task-------------------------------------//
// Speed comes from ZombieVerter over CAN (see can_task). This task applies the
// timeout, filters the value for display and integrates it for the odometer.

void speed_task(void *arg) {
    static float meter_frac = 0.0f;   // sub-metre accumulator
    uint64_t lastUs = esp_timer_get_time();

    while (true) {
        uint64_t nowUs = esp_timer_get_time();
        float dt_s = (float)(nowUs - lastUs) / 1000000.0f;
        lastUs = nowUs;

        float mph_raw = g_can_speed_mph;
        uint32_t now_ms = (uint32_t)(nowUs / 1000);
        if (now_ms - g_can_speed_ms > SPEED_TIMEOUT_MS) {
            mph_raw = 0.0f;
        }

        // ── Speed (display) ───────────────────────────────────────────────
        if (mph_raw == 0.0f) {
            g_speed_mph = 0.0f;
        } else {
            g_speed_mph += SPEED_FILTER_ALPHA * (mph_raw - g_speed_mph);
        }

        // ── Odometer (integrate unfiltered speed → metres) ────────────────
        meter_frac += mph_raw * (METRES_PER_MILE / 3600.0f) * dt_s;
        uint32_t whole = (uint32_t)meter_frac;
        if (whole > 0) {
            odometer_add_meters(whole);
            meter_frac -= whole;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//--------------------------------------------------------------------------//

//-----------CAN bus (RPM: ID 0x1DA | gear + speed: ZombieVerter)---------//
// RPM:   ID 0x1DA (Nissan Leaf inverter), bytes [4..5] big-endian int16.
//        Raw value divided by 2 = RPM.  Negative = reverse rotation (clamped to 0).
// Gear:  ZOMBIE_DIR_CAN_ID, signed byte — -1=R  0=N  1=D
//        Pill index: 0=R 1=N 2=D
// Speed: ZOMBIE_SPEED_CAN_ID, uint16 little-endian × ZOMBIE_SPEED_GAIN

static void process_sidelights(uint8_t *data) {
    g_sidelights_on = (data[0] & 0x02) != 0;  // byte 0, bit 1
}

// Only touches the LEDC (and the BSP's log line) when the level actually changes.
static void update_backlight(void) {
    static int last_level = -1;
    int level = g_sidelights_on ? BL_DIM : BL_NORMAL;
    if (level != last_level) {
        bsp_display_brightness_set(level);
        last_level = level;
    }
}

static void can_init(void) {
    twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_LISTEN_ONLY);
    g_cfg.rx_queue_len = CAN_RX_QUEUE_LEN;
    // Adjust baud rate macro to match your CAN network (500K is most common):
    twai_timing_config_t  t_cfg = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_cfg, &t_cfg, &f_cfg));
    ESP_ERROR_CHECK(twai_start());
}

void can_task(void *arg) {
    twai_message_t msg;

    while (true) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) != ESP_OK)
            continue;

        // ── RPM (ID 0x1DA Leaf inverter, bytes 4-5, big-endian int16 / 2) ─
        if (msg.identifier == 0x1DA && msg.data_length_code >= 6) {
            int16_t raw_rpm = (int16_t)((msg.data[4] << 8) | msg.data[5]);
            int32_t rpm_val = raw_rpm / 2;
            g_can_rpm    = (rpm_val > 0) ? (uint32_t)rpm_val : 0;
            g_can_rpm_ms = (uint32_t)(esp_timer_get_time() / 1000);
        }

        // ── Gear position (Zombie dir: -1=R 0=N 1=D 2=P) ─────────────────
        // No P on the display, so Park (and anything unexpected) shows N.
        if (msg.identifier == ZOMBIE_DIR_CAN_ID &&
            msg.data_length_code > ZOMBIE_DIR_BYTE) {
            int8_t dir = (int8_t)msg.data[ZOMBIE_DIR_BYTE];
            g_can_gear_idx = (dir == -1) ? 0 : (dir == 1) ? 2 : 1;
        }

        // ── Road speed (Zombie, uint16 little-endian) ────────────────────
        if (msg.identifier == ZOMBIE_SPEED_CAN_ID &&
            msg.data_length_code >= ZOMBIE_SPEED_BYTE + 2) {
            uint16_t raw = (uint16_t)(msg.data[ZOMBIE_SPEED_BYTE] |
                                      (msg.data[ZOMBIE_SPEED_BYTE + 1] << 8));
            float spd = raw * ZOMBIE_SPEED_GAIN;
#if ZOMBIE_SPEED_IN_KMH
            spd *= 0.621371f;
#endif
            if (spd > SPEED_MAX_MPH) spd = SPEED_MAX_MPH;
            g_can_speed_mph = spd;
            g_can_speed_ms  = (uint32_t)(esp_timer_get_time() / 1000);
        }

        // ── Sidelights (ID 0x801, byte 0 bit 1) ──────────────────────────
        if (msg.identifier == 0x801 && msg.data_length_code >= 1) {
            process_sidelights(msg.data);
            update_backlight();
        }
    }
}

//--------------------------------------------------------------------------//

// Drive the RPM arc (0–12000). Colour: green < 6000, amber < 9000, red ≥ 9000.
static void rpm_set_arc(float rpm) {
    if (rpm < 0.0f)      rpm = 0.0f;
    if (rpm > 12000.0f)  rpm = 12000.0f;

    // Arc range is in whole degrees, so sub-degree RPM jitter doesn't trigger a redraw.
    lv_arc_set_value(ui_speed_arc, (int)(rpm * ARC_SWEEP_DEG / 12000.0f));

    lv_color_t c;
    if      (rpm < 6000.0f)  c = lv_color_hex(0x00cc44);  // green
    else if (rpm < 9000.0f)  c = lv_color_hex(0xffaa00);  // amber
    else                     c = lv_color_hex(0xcc2222);  // red

    static lv_color_t last_c = {.full = 0};
    if (c.full != last_c.full) {
        lv_obj_set_style_arc_color(ui_speed_arc, c,
                                   LV_PART_INDICATOR | LV_STATE_DEFAULT);
        last_c = c;
    }
}


// ── Startup sweep ─────────────────────────────────────────────────────────
// Called by ui.c once the boot screen transition fires.
// gauge_timer runs the sweep for STARTUP_TICKS × 10 ms, then hands off to live data.

#define STARTUP_TICKS 300   // 300 × 10 ms = 3 s  (1.5 s up, 1.5 s down)

static bool g_startup_active = false;
static int  g_startup_tick   = 0;

// Last values drawn by the live path; reset to -1 to force a redraw.
static int live_last_int   = -1;
static int last_active_idx = -1;

void startup_sweep_begin(void)
{
    g_startup_active = true;
    g_startup_tick   = 0;
}

// Gear sequence: R→N→D→N  (4 stages, one per quarter of sweep time)
static const int sweep_gear_seq[] = {0, 1, 2, 1};
#define SWEEP_GEAR_STAGES (int)(sizeof(sweep_gear_seq) / sizeof(sweep_gear_seq[0]))

void gauge_timer(lv_timer_t * t) {

    // ── Startup sweep ─────────────────────────────────────────────────────
    if (g_startup_active) {
        g_startup_tick++;
        float frac     = (float)g_startup_tick / (float)STARTUP_TICKS; // 0→1
        float progress = (frac <= 0.5f) ? (frac * 2.0f) : (2.0f - frac * 2.0f); // 0→1→0

        // Arc
        rpm_set_arc(progress * 12000.0f);

        // Speed
        static int last_sweep_mph = -1;
        int sweep_mph = (int)(progress * 60.0f);
        if (sweep_mph != last_sweep_mph) {
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", sweep_mph);
            lv_label_set_text(ui_label_mph_value, buf);
            last_sweep_mph = sweep_mph;
        }

        // Gear
        static int last_sweep_gear = -1;
        int stage = (int)(frac * SWEEP_GEAR_STAGES);
        if (stage > SWEEP_GEAR_STAGES - 1) stage = SWEEP_GEAR_STAGES - 1;
        int sweep_gear = sweep_gear_seq[stage];
        if (sweep_gear != last_sweep_gear) {
            for (int i = 0; i < UI_GEAR_COUNT; i++) {
                lv_color_t gc = (i == sweep_gear)
                    ? ((i == 1) ? orange_color : green_color)
                    : lv_color_hex(0x1e2e1e);
                lv_obj_set_style_text_color(ui_gear_labels[i], gc, LV_PART_MAIN);
            }
            last_sweep_gear = sweep_gear;
        }

        if (g_startup_tick >= STARTUP_TICKS) {
            g_startup_active = false;
            // Reset display; live data takes over next tick
            rpm_set_arc(0);
            lv_label_set_text(ui_label_mph_value, "0");
            // The sweep overwrote the speed and gear labels, so force the
            // live path to redraw them even if the values haven't changed.
            live_last_int   = -1;
            last_active_idx = -1;
        }
        return;
    }

    // ── Live data ─────────────────────────────────────────────────────────
    float effective_mph = g_speed_mph;
    // Round, don't truncate: the speed EMA settles a hair below its target
    // (e.g. 59.999996 for 60), which (int) would show as 59.
    int live_int = (int)lroundf(effective_mph);
    if (live_int != live_last_int) {
        static char live_buf[8];
        snprintf(live_buf, sizeof(live_buf), "%d", live_int);
        lv_label_set_text(ui_label_mph_value, live_buf);
        live_last_int = live_int;
    }

    // ── RPM arc ───────────────────────────────────────────────────────────
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t rpm = g_can_rpm;
    if (now_ms - g_can_rpm_ms > RPM_TIMEOUT_MS) {
        rpm = 0;
    }
    rpm_set_arc((float)rpm);

    // ── Odometer (only reformat when the displayed tenth changes) ─────────
    static long last_odo_tenths = -1;
    double miles = odometer_get_miles();
    long odo_tenths = lround(miles * 10.0);
    if (odo_tenths != last_odo_tenths) {
        char odo_buf[16];
        snprintf(odo_buf, sizeof(odo_buf), "%06.1f", miles);
        update_label_if_needed(ui_label_odometer_value, odo_buf, lv_color_hex(0xaabbaa));
        last_odo_tenths = odo_tenths;
    }

    // ── Gear indicator ────────────────────────────────────────────────────
    int active_idx = g_can_gear_idx;
    if (active_idx != last_active_idx) {
        for (int i = 0; i < UI_GEAR_COUNT; i++) {
            lv_color_t gc = (i == active_idx)
                ? ((i == 1) ? orange_color : green_color)
                : lv_color_hex(0x1e2e1e);
            lv_obj_set_style_text_color(ui_gear_labels[i], gc, LV_PART_MAIN);
        }
        last_active_idx = active_idx;
    }
}

//------------------------------------------------------------------------//


void save_miles_task(void *arg){
    while (1){
        odometer_periodic_save();
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

//------------------------------------------------------------------------//

// Same as bsp_display_start_with_config() minus the touch controller. The
// speedo has no touch input, and the BSP aborts the whole boot whenever the
// GT911 misses its I2C init (seen on roughly half of resets).
static lv_display_t *display_start_no_touch(void) {
    lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_cfg.task_affinity = 1;   // core 1 to itself; speed/CAN/save tasks are pinned to core 0
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    ESP_ERROR_CHECK(bsp_display_brightness_init());

    bsp_lcd_handles_t lcd;
    ESP_ERROR_CHECK(bsp_display_new_with_handles(NULL, &lcd));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle      = lcd.io,
        .panel_handle   = lcd.panel,
        .control_handle = lcd.control,
        .buffer_size    = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer  = BSP_LCD_DRAW_BUFF_DOUBLE,
        .hres           = BSP_LCD_H_RES,
        .vres           = BSP_LCD_V_RES,
        .monochrome     = false,
        .rotation = {
            .swap_xy  = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma    = true,
            .buff_spiram = false,
            .sw_rotate   = true,   // DSI panel has no hardware rotation
        },
    };
    const lvgl_port_display_dsi_cfg_t dsi_cfg = {
        .flags = {
            .avoid_tearing = false,   // not supported together with sw_rotate
        },
    };
    lv_display_t *disp = lvgl_port_add_disp_dsi(&disp_cfg, &dsi_cfg);
    assert(disp);
    return disp;
}

void app_main(void) {
    lv_display_t *disp = display_start_no_touch();

    init_label_styles();
    can_init();
    odometer_init();

    // The LVGL task is already running, so every LVGL call from here must hold the lock.
    bsp_display_lock(0);
    bsp_display_rotate(disp, LV_DISPLAY_ROTATION_270);
    ui_init();
    lv_timer_create(gauge_timer, 10, NULL);
    bsp_display_unlock();

    // Set the backlight before can_task starts, so a sidelight frame that
    // arrives early isn't overridden here.
    bsp_display_backlight_off();
    vTaskDelay(pdMS_TO_TICKS(100));
    update_backlight();

    xTaskCreatePinnedToCore(speed_task, "speed_task", 4096, NULL, 9, NULL, 0);
    xTaskCreatePinnedToCore(can_task,  "can_task",  4096, NULL,  8, NULL, 0);
    xTaskCreatePinnedToCore(save_miles_task, "save_miles_task", 4096, NULL, 4, NULL, 0);

}
