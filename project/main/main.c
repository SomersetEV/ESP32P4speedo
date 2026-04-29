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
#include "lv_demos.h"
#include "ui.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "driver/adc.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "odometer/odometer.h"
#include "lap_timer.h"

//-----Pin Assignment---------//

//UART0 Transaction - GPIO 37
#define UART_TX_PIN 37

//UART1 Transaction - GPIO30
#define UART1_TX_PIN 30

//Water Temp - GPIO 20
#define WATER_TEMP_ADC_CHANNEL ADC1_CHANNEL_4

//Oil Temp - GPIO 50
#define OIL_TEMP_ADC_CHANNEL ADC2_CHANNEL_1

// Boost - GPIO 52
#define BOOST_ADC_CHANNEL ADC2_CHANNEL_3

//Oil Pressure - GPIO 21
#define OIL_PRESSURE_ADC_CHANNEL ADC1_CHANNEL_5

//Fuel Pressure - GPIO 22
#define FUEL_PRESSURE_ADC_CHANNEL ADC1_CHANNEL_6

#define TACH_GPIO GPIO_NUM_4

// AFR - GPIO 49
#define AFR_ADC_CHANNEL ADC2_CHANNEL_0 

// Fuel level - GPIO 51
#define FUEL_ADC_CHANNEL ADC2_CHANNEL_2

// CAN bus (MCP2551 transceiver)
// NOTE: MCP2551 RXD output is 5V — use 10kΩ+20kΩ voltage divider before ESP32 RX pin.
//       MCP2551 TXD input accepts 3.3V logic directly from ESP32.
#define CAN_TX_GPIO  GPIO_NUM_0  // <-- ESP32 TX → MCP2551 TXD  (assign real pin before enabling CAN)
#define CAN_RX_GPIO  GPIO_NUM_0  // <-- MCP2551 RXD → (voltage divider) → ESP32 RX

//--------------------------//


#define UART_PORT UART_NUM_1
#define UART1_PORT UART_NUM_2
#define GAUGE_PKT_SOF   0xA5
#define GAUGE_PKT_LEN   26
#define UART_TX_BUF_SIZE 256
#define UART_BAUD_RATE 2000000

#define ENABLE_LOGS false
#define ADC_UPDATE_PERIOD_MS 10
#define FILTER_SAMPLES_DEFAULT 8

// Gear pill index from CAN: 0=P 1=N 2=R 3=D 4=S  (default N until first CAN frame)
static volatile int      g_can_gear_idx = 1;
// RPM from CAN ID 0x1DA bytes 4-5 (Leaf inverter: big-endian int16, raw / 2 = RPM)
static volatile uint32_t g_can_rpm      = 0;

// Backlight levels
#define BL_NORMAL  100
#define BL_DIM      50

// Sidelights (from CAN 0x801)
static volatile bool g_sidelights_on = false;

//--------UPDATE/REFRESH_DELAYS------//
#define FUEL_UPDATE_PERIOD 1000 // 1 updates a second
#define TEMP_UPDATE_DELAY 250 // 4 updates a second
#define PRESSURE_UPDATE_DELAY 50 // 20 updates a second
#define AFR_UPDATE_DELAY   20 // 50 updates a second
#define TACH_UPDATE_DELAY   7 // ~143 updates a second
//-----------------------------------//

//-------------TEMP-------------//
#define R_PULLUP       1000.0f
#define R1 10000.0f
#define R2 20000.0f
#define ADC_SCALE (R2 / (R1 + R2))
#define ADC_MAX        4095.0f
#define ADC_VREF       3.7f
#define SENSOR_SUPPLY  5.0f
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_12

#define TEMP_SENSOR_TABLE_SIZE 9
const float tempF[] = {
    32, 68, 104, 140, 176, 212, 248, 284, 302
};

const float sensorR[] = {
    12000, 8000, 5830, 3020, 1670, 975, 599, 386, 316
};
//------------------------------//

//-------------BOOST-------------//
#define BOOST_FILTER_ALPHA 0.18F
#define BOOST_DIVIDER_SCALE (20.0f / (10.0f + 20.0f))
//equals voltage that boost sensor reads 0psi
#define BOOST_ZERO_OFFSET 1.0F
//adds psi for gauge calibration as all gauges read a little off...
#define BOOST_OFFSET 3.2F
//--------------------------------//


//-------------PRESSURE-------------//
#define ADC_ATTEN ADC_ATTEN_DB_12           
#define ADC_UNIT ADC_UNIT_1
#define OIL_FUEL_DIVIDER_SCALE ((10.0f + 20.0f) / 20.0f)

#define PRESSURE_SENSOR_TABLE_SIZE 5
const float Vpts[PRESSURE_SENSOR_TABLE_SIZE] = {0.5f, 1.3f, 2.5f, 3.7f, 4.5f};
const float PSIpts[PRESSURE_SENSOR_TABLE_SIZE] = {0.0f, 29.0f, 72.5f, 116.0f, 145.0f};
const float oil_fuel_pressure_alpha = 0.18f;
//------------------------------//


//-------------WIDEBAND AFR-------------//

#define AFR_DIVIDER_GAIN ((68.0f + 33.0f) / 33.0f)   // ≈ 3.06
#define AFR_OFFSET 0.7f 
#define AFR_FILTER_ALPHA 0.3f   // smoothing
//--------------------------------------//

//-------------RPM-------------//
#define PULSES_PER_REV 2
#define MIN_PULSE_COUNT 2       // minimum pulses before computing RPM
#define MAX_RPM 12000.0f
#define ARC_SCALE (100.0f / MAX_RPM) 
#define RPM_MIN_PERIOD  3000
#define RPM_TIMEOUT_MS  500    // If no pulse for this long, RPM = 0

static portMUX_TYPE tachMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint64_t lastTachUs = 0;
volatile uint64_t tachPeriodUs = 0;
volatile uint64_t lastPulseMs = 0;

float rpmNow = 0.0f;
float rpmFiltered = 0.0f;

#define PERIOD_AVG_SAMPLES 4

uint64_t periodBuffer[PERIOD_AVG_SAMPLES] = {0};
int periodIndex = 0;

//------------------------------//

//-------------VSS (YBE100530 Hall effect, LT230 transfer box)-------------//
// Signal via optocoupler — output is open-collector 3.3V, internal pull-up used.
// Calibration: 8,000 pulses per mile.
// mph = 450,000.0f / period_us  (derived: 3,600,000,000 / (8000 * period_us))

#define VSS_GPIO            GPIO_NUM_35  // <-- set to your wired GPIO pin
#define VSS_MIN_PERIOD_US   1000         // ~3600 mph floor, kills glitches
#define VSS_TIMEOUT_MS      1000         // no pulse for 1 s → 0 mph
#define VSS_AVG_SAMPLES     4

static portMUX_TYPE vssMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint64_t lastVssUs    = 0;
static volatile uint64_t lastVssMs    = 0;
static volatile uint32_t vssPulseCount = 0;   // raw pulse count for odometer
static uint64_t vssPeriodBuf[VSS_AVG_SAMPLES] = {0};
static int      vssPeriodIdx = 0;

volatile float g_vss_mph = 0.0f;

//------------------------------//

//-------------FUEL-------------//
#define FUEL_PULLUP_VOLTAGE   3.3f
#define ADC_MAX               4095.0f
#define PULLUP_RESISTOR_OHMS  150.0f
#define FILTER_SAMPLES_FUEL   16
#define BOOT_SETTLE_MS        1500

static uint8_t fuel_percent = 0;
static int64_t boot_time_ms = 0;
//------------------------------//

//------------------------------//

//-------------LOGGING------------//
static const char *TAG_FUEL = "FUEL_SENSOR";
//--------------------------------//

//------------DATA_SENT_OUT---------//
typedef struct {
    float oil_temp_f;
    float water_temp_f;
    float oil_pressure_psi;
    float fuel_pressure_psi;
    float fuel_level_pct;
    float afr;
    float boost_psi;
} gauge_data_t;

static gauge_data_t g_gauge_data;


typedef struct __attribute__((packed)) {
    uint16_t oil_temp;       // °F x10
    uint16_t water_temp;     // °F x10
    uint16_t oil_pressure;   // psi x10
    uint16_t fuel_pressure;  // psi x10
    uint16_t fuel_level;     // % x10
    uint16_t afr;            // AFR x10
    int16_t boost;           // psi x10
    uint32_t lap_time_ms;    // current lap in milliseconds
    int32_t  lap_delta_ms;   // predictive delta in ms
} gauge_payload_t;


//---------------------------------------//

static lv_color_t green_color;
static lv_color_t red_color;
static lv_color_t orange_color;
static lv_color_t purple_color;
static lv_color_t pink_color;
static lv_color_t blue_color;

static inline int constrain_int(int x, int low, int high) {
    if (x < low) return low;
    if (x > high) return high;
    return x;
}

static void init_label_styles(void){

    green_color = lv_color_hex(0x28FF00);
    red_color = lv_palette_main(LV_PALETTE_RED);
    orange_color = lv_palette_main(LV_PALETTE_ORANGE);
    purple_color = lv_palette_main(LV_PALETTE_PURPLE);
    pink_color = lv_palette_main(LV_PALETTE_PINK);
    blue_color = lv_palette_main(LV_PALETTE_CYAN);
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

//-----------------------FUEL---------------------------//



float fuel_pct_from_voltage(float v){
    const float V_FULL  = 0.06f;
    const float V_HALF  = 0.53f;
    const float V_25    = 0.845f;
    const float V_5     = 0.91f;
    const float V_EMPTY = 0.95f;

    if (v <= V_FULL)  return 100.0f;
    if (v >= V_EMPTY) return 0.0f;

    if (v <= V_HALF)
        return 50.0f +
               50.0f * (V_HALF - v) / (V_HALF - V_FULL);

    if (v <= V_25)
        return 25.0f +
               25.0f * (V_25 - v) / (V_25 - V_HALF);

    if (v <= V_5)
        return 5.0f +
               20.0f * (V_5 - v) / (V_5 - V_25);

    return 5.0f *
           (V_EMPTY - v) / (V_EMPTY - V_5);
}

//-----------------------------------------------------//



//-----------------------TEMP--------------------------//

float read_temp_resistance(int raw){

    float adc_voltage = (raw / ADC_MAX) * ADC_VREF;

    // Undo scaling divider
    float signal_voltage = adc_voltage / ADC_SCALE;

    // Calculate sensor resistance
    float sensor_resistance = R_PULLUP *
        (signal_voltage / (SENSOR_SUPPLY - signal_voltage));

    return sensor_resistance;
}

//-----------------------------------------------------//

//-----------------------_RPM---------------------------//

// Optional adaptive alpha for EMA
static inline float alphaForRPM(float rpmRaw) {
    // Base alpha depending on RPM range (smoother at low RPM, faster at high RPM)
    float base;
    if (rpmRaw < 1200.0f) base = 0.04f;      // idle, very smooth
    else if (rpmRaw < 3000.0f) base = 0.10f; // mid RPM
    else base = 0.18f; 
    return base;                       // high RPM, faster updates
}


void IRAM_ATTR tachISR(void* arg) {
    uint64_t now = esp_timer_get_time();
    uint64_t dt = now - lastTachUs;
    if (dt < RPM_MIN_PERIOD) return;
    portENTER_CRITICAL_ISR(&tachMux);
    periodBuffer[periodIndex] = dt;
    periodIndex = (periodIndex + 1) % PERIOD_AVG_SAMPLES;
    tachPeriodUs = dt;   // store last good period
    lastTachUs = now;
    lastPulseMs = now / 1000;
    portEXIT_CRITICAL_ISR(&tachMux);
}

void tach_init() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;  // optocoupler pulls low on pulse
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << TACH_GPIO);
    io_conf.pull_up_en = 1;   // internal pull-up; optocoupler is open-collector
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(TACH_GPIO, tachISR, NULL);
}

void tach_task(void *arg) {
    const TickType_t delay = pdMS_TO_TICKS(TACH_UPDATE_DELAY);

    static int log_counter = 0;

    while (true) {

        float rpmRaw = 0.0f;
        int count = 0;

        // ----- Average RPM instead of period -----
        portENTER_CRITICAL(&tachMux);
        for (int i = 0; i < PERIOD_AVG_SAMPLES; i++) {
            if (periodBuffer[i] > 0) {
                float rpm = 60000000.0f /
                            (periodBuffer[i] * PULSES_PER_REV);
                rpmRaw += rpm;
                count++;
            }
        }
        portEXIT_CRITICAL(&tachMux);

        if (count > 0) {
            rpmRaw /= count;
        }

        uint64_t nowMs = esp_timer_get_time() / 1000;

        // ----- Timeout → RPM = 0 -----
        if (nowMs - lastPulseMs > RPM_TIMEOUT_MS) {
            rpmFiltered = 0.0f;
            rpmNow = 0.0f;
        }
        else if (count > 0) {

            float alpha = alphaForRPM(rpmRaw);

            if (rpmFiltered == 0.0f)
                rpmFiltered = rpmRaw;
            else
                rpmFiltered += alpha * (rpmRaw - rpmFiltered);

            rpmNow = rpmFiltered;
        }

        #if ENABLE_LOGS
            if (++log_counter >= 50) {
                ESP_LOGI(TAG_TACH, "RPM Now: %.1f, Filtered: %.1f", rpmNow, rpmFiltered);
                log_counter = 0;
            }
        #endif

        vTaskDelay(delay);
    }
}

//-----------VSS ISR / init / task------------------------------------------//

void IRAM_ATTR vssISR(void *arg) {
    uint64_t now = esp_timer_get_time();
    uint64_t dt  = now - lastVssUs;
    if (dt < VSS_MIN_PERIOD_US) return;
    portENTER_CRITICAL_ISR(&vssMux);
    vssPeriodBuf[vssPeriodIdx] = dt;
    vssPeriodIdx = (vssPeriodIdx + 1) % VSS_AVG_SAMPLES;
    lastVssUs = now;
    lastVssMs = now / 1000;
    vssPulseCount++;
    portEXIT_CRITICAL_ISR(&vssMux);
}

static void vss_init(void) {
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_NEGEDGE, // optocoupler pulls low on pulse
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << VSS_GPIO),
        .pull_up_en   = 1,   // internal pull-up; optocoupler is open-collector
        .pull_down_en = 0,
    };
    gpio_config(&io_conf);
    // ISR service already installed by tach_init(); just add handler.
    gpio_isr_handler_add(VSS_GPIO, vssISR, NULL);
}

void vss_task(void *arg) {
    static float meter_frac = 0.0f;   // sub-metre accumulator

    while (true) {
        uint64_t periods[VSS_AVG_SAMPLES];
        uint32_t pulses;
        portENTER_CRITICAL(&vssMux);
        memcpy(periods, vssPeriodBuf, sizeof(periods));
        pulses = vssPulseCount;
        vssPulseCount = 0;
        portEXIT_CRITICAL(&vssMux);

        // ── Speed (period average) ─────────────────────────────────────────
        float sum = 0.0f;
        int   count = 0;
        for (int i = 0; i < VSS_AVG_SAMPLES; i++) {
            if (periods[i] > 0) {
                sum += 450000.0f / (float)periods[i];
                count++;
            }
        }

        uint64_t nowMs = esp_timer_get_time() / 1000;
        if (nowMs - lastVssMs > VSS_TIMEOUT_MS) {
            g_vss_mph = 0.0f;
        } else if (count > 0) {
            g_vss_mph = sum / count;
        }

        // ── Odometer (pulse count → metres) ───────────────────────────────
        // 8000 pulses/mile, 1 pulse = 1609.344 / 8000 = 0.201168 m
        if (pulses > 0) {
            meter_frac += pulses * 0.201168f;
            uint32_t whole = (uint32_t)meter_frac;
            if (whole > 0) {
                odometer_add_meters(whole);
                meter_frac -= whole;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//--------------------------------------------------------------------------//

//-----------CAN bus (RPM: ID 0x1DA bytes 4-5 | gear: ID 0x312 byte 2)----//
// RPM:  ID 0x1DA (Nissan Leaf inverter), bytes [4..5] big-endian int16.
//       Raw value divided by 2 = RPM.  Negative = reverse rotation (clamped to 0).
// Gear: ID 0x312, byte 2 — 0x8F=P  0x8E=R  0x8D=N  0x8C=D  0x88=S
//       Pill index: 0=P 1=N 2=R 3=D 4=S

static void process_sidelights(uint8_t *data) {
    g_sidelights_on = (data[0] & 0x02) != 0;  // byte 0, bit 1
}

static void update_backlight(void) {
    bsp_display_brightness_set(g_sidelights_on ? BL_DIM : BL_NORMAL);
}

static void can_init(void) {
    twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_LISTEN_ONLY);
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
            g_can_rpm = (rpm_val > 0) ? (uint32_t)rpm_val : 0;
        }

        // ── Gear position (ID 0x312, byte 2) ─────────────────────────────
        if (msg.identifier == 0x312 && msg.data_length_code >= 3) {
            switch (msg.data[2]) {
                case 0x8F: g_can_gear_idx = 0; break;  // P
                case 0x8D: g_can_gear_idx = 1; break;  // N
                case 0x8E: g_can_gear_idx = 2; break;  // R
                case 0x8C: g_can_gear_idx = 3; break;  // D
                case 0x88: g_can_gear_idx = 4; break;  // S
                default: break;                         // unknown — keep last
            }
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

    lv_arc_set_value(ui_speed_arc, (int)rpm);

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

void startup_sweep_begin(void)
{
    g_startup_active = true;
    g_startup_tick   = 0;
}

// Gear sequence: P→N→R→D→S→N  (6 stages, one per sixth of sweep time)
static const int sweep_gear_seq[] = {0, 1, 2, 3, 4, 1};

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
        int stage = (int)(frac * 6.0f);
        if (stage > 5) stage = 5;
        int sweep_gear = sweep_gear_seq[stage];
        if (sweep_gear != last_sweep_gear) {
            for (int i = 0; i < 5; i++) {
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
        }
        return;
    }

    // ── Live data ─────────────────────────────────────────────────────────
    float effective_mph = g_vss_mph;
    static int live_last_int = -1;
    int live_int = (int)effective_mph;
    if (live_int != live_last_int) {
        static char live_buf[8];
        snprintf(live_buf, sizeof(live_buf), "%d", live_int);
        lv_label_set_text(ui_label_mph_value, live_buf);
        live_last_int = live_int;
    }

    // ── RPM arc ───────────────────────────────────────────────────────────
    rpm_set_arc((float)g_can_rpm);

    // ── Odometer ──────────────────────────────────────────────────────────
    double miles = odometer_get_miles();
    char odo_buf[16];
    snprintf(odo_buf, sizeof(odo_buf), "%06.1f", miles);
    update_label_if_needed(ui_label_odometer_value, odo_buf, lv_color_hex(0xaabbaa));

    // ── Gear indicator ────────────────────────────────────────────────────
    int active_idx = g_can_gear_idx;
    static int last_active_idx = -1;
    if (active_idx != last_active_idx) {
        for (int i = 0; i < 5; i++) {
            lv_color_t gc = (i == active_idx)
                ? ((i == 1) ? orange_color : green_color)
                : lv_color_hex(0x1e2e1e);
            lv_obj_set_style_text_color(ui_gear_labels[i], gc, LV_PART_MAIN);
        }
        last_active_idx = active_idx;
    }
}

//------------------------------------------------------------------------//



float resistance_to_F(float R) {
    if (R >= sensorR[0]) return tempF[0];
    if (R <= sensorR[TEMP_SENSOR_TABLE_SIZE - 1]) return tempF[TEMP_SENSOR_TABLE_SIZE - 1];

    for (int i = 0; i < TEMP_SENSOR_TABLE_SIZE - 1; i++) {
        if (R <= sensorR[i] && R >= sensorR[i+1]) {
            float t = tempF[i] + (sensorR[i] - R) * (tempF[i+1] - tempF[i]) / (sensorR[i] - sensorR[i+1]);
            return t; 
        }
    }
    return tempF[0];
}

float voltage_to_psi(float v) {

    // Below first threshold → clamp to 0 PSI
    if (v <= Vpts[0]) return 0.0f;
    // Above last threshold → clamp to max
    if (v >= Vpts[4]) return PSIpts[4];

    // Find segment and linearly interpolate
    for (int i = 0; i < 4; i++) {
        if (v < Vpts[i+1]) {
            float t = (v - Vpts[i]) / (Vpts[i+1] - Vpts[i]);
            return PSIpts[i] + t * (PSIpts[i+1] - PSIpts[i]);
        }
    }
    return 0.0f;
}

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}


void save_miles_task(void *arg){
    while (1){
        odometer_periodic_save();
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

//------------------------------------------------------------------------//


//------------------------------ADC_UART---------------------------------------//
static void adc_global_init(void) {
    adc1_config_width(ADC_WIDTH);

    // ADC1 channels
    adc1_config_channel_atten(WATER_TEMP_ADC_CHANNEL, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(OIL_PRESSURE_ADC_CHANNEL, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(FUEL_PRESSURE_ADC_CHANNEL, ADC_ATTEN_DB_12);
    

    // ADC2 channels
    adc2_config_channel_atten(BOOST_ADC_CHANNEL, ADC_ATTEN_DB_12);
    adc2_config_channel_atten(OIL_TEMP_ADC_CHANNEL, ADC_ATTEN_DB_12);
    adc2_config_channel_atten(FUEL_ADC_CHANNEL, ADC_ATTEN_DB_12);
    adc2_config_channel_atten(AFR_ADC_CHANNEL, ADC_ATTEN_DB_12);

    ESP_LOGI("ADC", "ADC Global Init Complete");
}

uint32_t sample_sum_adc1(adc1_channel_t adc_channel, int samples){
    uint32_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += adc1_get_raw(adc_channel);
    }
    return sum / samples;
}

uint32_t sample_sum_adc2(adc2_channel_t adc_channel, int samples){
    uint32_t sum = 0;
    int raw = 0;

    for (int i = 0; i < samples; i++) {
        adc2_get_raw(adc_channel, ADC_WIDTH, &raw);
        sum += raw;
    }

    return sum / samples;
}

static void adc_task(void *arg) {
    int64_t last_temp_ms     = 0;
    int64_t last_pressure_ms = 0;
    int64_t last_afr_ms      = 0;
    int64_t last_fuel_ms     = 0;
    int64_t last_tx_ms       = 0;
    boot_time_ms = esp_timer_get_time() / 1000;
    static float water_filtered = -1;
    static float oil_filtered = -1;
    static float oil_press_filtered = -1;
    static float fuel_press_filtered = -1;
    static float fuel_filtered = 0.0f;
    static bool fuel_initialized = false;
    static bool fuel_ever_valid = false;


    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;

        // ---------- Temperature Update ---------- //
        if (now_ms - last_temp_ms >= TEMP_UPDATE_DELAY) {
            last_temp_ms = now_ms;

            // Water temp (ADC1)
            //int raw_water = adc1_get_raw(WATER_TEMP_ADC_CHANNEL);
            int raw_water = sample_sum_adc1(WATER_TEMP_ADC_CHANNEL, FILTER_SAMPLES_DEFAULT);
            float R_water = read_temp_resistance(raw_water);

            // Oil temp (ADC2)
            // adc2_get_raw(OIL_TEMP_ADC_CHANNEL, ADC_WIDTH, &raw_oil);
            int raw_oil = sample_sum_adc2(OIL_TEMP_ADC_CHANNEL, FILTER_SAMPLES_DEFAULT);
            float R_oil = read_temp_resistance(raw_oil);

            float water_new = resistance_to_F(R_water);
            float oil_new   = resistance_to_F(R_oil);

            if (water_filtered < 0) water_filtered = water_new;
            if (oil_filtered   < 0) oil_filtered   = oil_new;

            water_filtered = water_filtered * 0.95f + water_new * 0.05f;
            oil_filtered   = oil_filtered   * 0.95f + oil_new   * 0.05f;

            g_gauge_data.water_temp_f = water_filtered;
            g_gauge_data.oil_temp_f   = oil_filtered;
        }

        // ---------- Pressure Update ---------- //
        if (now_ms - last_pressure_ms >= PRESSURE_UPDATE_DELAY) {
            last_pressure_ms = now_ms;

            // Oil pressure (ADC1)
            //int raw_oil_press = adc1_get_raw(OIL_PRESSURE_ADC_CHANNEL);
            float raw_oil_press = sample_sum_adc1(OIL_PRESSURE_ADC_CHANNEL, FILTER_SAMPLES_DEFAULT);
            float voltage_oil = ((float)raw_oil_press / 4095.0f) * ADC_VREF;

            // Fuel pressure (ADC1)
            //int raw_fuel_press = adc1_get_raw(FUEL_PRESSURE_ADC_CHANNEL);
            int raw_fuel_press = sample_sum_adc1(FUEL_PRESSURE_ADC_CHANNEL, FILTER_SAMPLES_DEFAULT);
            float voltage_fuel = ((float)raw_fuel_press / 4095.0f) * ADC_VREF;
    

            float oil_press_new = voltage_to_psi(voltage_oil * OIL_FUEL_DIVIDER_SCALE);
            float fuel_press_new = voltage_to_psi(voltage_fuel * OIL_FUEL_DIVIDER_SCALE);

            if (oil_press_filtered < 0) oil_press_filtered = oil_press_new;
            if (fuel_press_filtered < 0) fuel_press_filtered = fuel_press_new;

            oil_press_filtered =
                oil_press_filtered + oil_fuel_pressure_alpha * (oil_press_new - oil_press_filtered);

            fuel_press_filtered =
                fuel_press_filtered + oil_fuel_pressure_alpha * (fuel_press_new - fuel_press_filtered);


            g_gauge_data.fuel_pressure_psi = fuel_press_filtered;
            g_gauge_data.oil_pressure_psi = oil_press_filtered;

            // ---------- Boost pressure (ADC2) ----------
            int raw_boost;
            adc2_get_raw(BOOST_ADC_CHANNEL, ADC_WIDTH, &raw_boost);
            float voltage_boost = ((float)raw_boost / 4095.0f) * ADC_VREF;
            // Undo any voltage divider if present
            float sensor_voltage = voltage_boost / BOOST_DIVIDER_SCALE;
            // Prosport sender scales ~1V @ 0 PSI to ~4V @ ~43.5 PSI
            float pressure_psi = (sensor_voltage - BOOST_ZERO_OFFSET) * 14.5f;
            pressure_psi += BOOST_OFFSET;
            // Clamp to realistic limits
            if (pressure_psi < -15.0f) pressure_psi = -15.0f;
            if (pressure_psi > 45.0f)  pressure_psi = 45.0f;
            //Simple EMA filter
            static float boost_filtered = 0.0f;
            boost_filtered = boost_filtered * (1 - BOOST_FILTER_ALPHA)
                            + pressure_psi * BOOST_FILTER_ALPHA;
            g_gauge_data.boost_psi = boost_filtered;

        }

        // ---------- Wideband AFR (ADC2) ---------- //
        if (now_ms - last_afr_ms >= AFR_UPDATE_DELAY) { 
            last_afr_ms = now_ms; 
            int raw_afr; 
            adc2_get_raw(AFR_ADC_CHANNEL, ADC_WIDTH, &raw_afr); 
            float adc_voltage = ((float)raw_afr / 4095.0f) * ADC_VREF; 
            // Undo voltage divider to get actual AEM output voltage 
            float wb_voltage = adc_voltage * AFR_DIVIDER_GAIN; 
            // Apply AEM linear scaling (Page 11) 
            float afr = (2.3750f * wb_voltage) + 7.3125f;
            afr += AFR_OFFSET;
            // Optional clamp for sanity 
            if (afr < 7.0f) afr = 7.0f; 
            if (afr > 22.0f) afr = 22.0f; 
            // Simple EMA filter 
            static float afr_filtered = 14.7f; 
            afr_filtered = afr_filtered + AFR_FILTER_ALPHA * (afr - afr_filtered); 
            g_gauge_data.afr = afr_filtered;
        }

        // ---------- Fuel Gauge Update ----------
        if (now_ms - last_fuel_ms >= FUEL_UPDATE_PERIOD) {
            last_fuel_ms = now_ms;

            float avg_raw = sample_sum_adc2(FUEL_ADC_CHANNEL, FILTER_SAMPLES_DEFAULT);
            float vFuel = (avg_raw * ADC_VREF) / ADC_MAX;

            bool fuelSettled =
                (now_ms - boot_time_ms) > BOOT_SETTLE_MS;

            bool fuelValid =
                fuelSettled &&
                (vFuel >= 0.01f) &&
                (vFuel <= 3.29f);

            if (fuelValid) {

                float new_pct = fuel_pct_from_voltage(vFuel);

                // Initialize once
                if (!fuel_initialized) {
                    fuel_filtered = new_pct;
                    fuel_initialized = true;
                }

                float alpha;

                if (new_pct < fuel_filtered) {
                    // Tank dropping → respond faster
                    alpha = 0.20f;   // 20% per second
                } else {
                    // Tank rising (slosh / incline) → respond slowly
                    alpha = 0.10f;   // 10% per second
                }

                fuel_filtered += alpha * (new_pct - fuel_filtered);

                fuel_percent = fuel_filtered; 
                g_gauge_data.fuel_level_pct = fuel_percent;

                fuel_ever_valid = true;

            } else {

                if (!fuel_ever_valid) {
                    fuel_percent = 0;
                    g_gauge_data.fuel_level_pct = 0;
                }

                ESP_LOGW(TAG_FUEL, "Fuel invalid V=%.3f", vFuel);
            }
        }
        if (now_ms - last_tx_ms >= 20) {
            last_tx_ms = now_ms;
            static uint8_t seq = 0;
            uint8_t buf[GAUGE_PKT_LEN];

            buf[0] = GAUGE_PKT_SOF;
            buf[1] = seq++;

            gauge_payload_t *p = (gauge_payload_t *)&buf[2];

            p->oil_temp      = (uint16_t)(g_gauge_data.oil_temp_f * 10.0f);
            p->water_temp    = (uint16_t)(g_gauge_data.water_temp_f * 10.0f);
            p->oil_pressure  = (uint16_t)(g_gauge_data.oil_pressure_psi * 10.0f);
            p->fuel_pressure = (uint16_t)(g_gauge_data.fuel_pressure_psi * 10.0f);
            p->fuel_level    = (uint16_t)(g_gauge_data.fuel_level_pct * 10.0f);
            p->afr           = (uint16_t)(g_gauge_data.afr * 10.0f);
            p->boost         = (int16_t)(g_gauge_data.boost_psi * 10.0f);
            
            uint64_t lap_us = lap_timer_get_current_us();
            int32_t  delta_us = lap_timer_get_delta_us();

            p->lap_time_ms  = (uint32_t)(lap_us / 1000);
            p->lap_delta_ms = (int32_t)(delta_us / 1000);

            uint16_t crc = crc16_ccitt(&buf[1], GAUGE_PKT_LEN - 3);
            memcpy(&buf[GAUGE_PKT_LEN - 2], &crc, 2);

            uart_write_bytes(UART_PORT, buf, GAUGE_PKT_LEN);
            uart_write_bytes(UART1_PORT, buf, GAUGE_PKT_LEN);
        }

        // Optional logging
        #if ENABLE_LOGS
            ESP_LOGI(TAG_FUEL,
                    "Fuel:  %%=%u",
                    fuel_percent);
            ESP_LOGI(TAG_TEMP,
                    "Water: %.1fF  Oil: %.1fF",
                    g_gauge_data.water_temp_f,
                    g_gauge_data.oil_temp_f);
            ESP_LOGI(TAG_PRESSURE,
                    "Oil PSI: %.2f  Fuel PSI: %.2f Boost PSI: %.2f ",
                    g_gauge_data.oil_pressure_psi,
                    g_gauge_data.fuel_pressure_psi,
                    g_gauge_data.boost_psi);
            ESP_LOGI(TAG_AFR,
                    "AFR: %.2f",
                    g_gauge_data.afr);
        #endif

        vTaskDelay(pdMS_TO_TICKS(ADC_UPDATE_PERIOD_MS));
    }
}


static void uart_init(uart_port_t uart_num, int txPin, int rxPin, int bufSize, int baud) {
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT 
    };

    ESP_ERROR_CHECK(uart_driver_install(
        uart_num,
        bufSize,   // TX buffer
        0,         // RX buffer
        0,
        NULL,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(uart_num, &cfg));

    ESP_ERROR_CHECK(uart_set_pin(
        uart_num,
        txPin,
        rxPin,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));
}

//------------------------------------------------------------------------//

void app_main(void) {
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
            .sw_rotate = true,
        }
    };
    lv_display_t *disp = bsp_display_start_with_config(&cfg);
    bsp_display_rotate(disp, LV_DISPLAY_ROTATION_270);

    adc_global_init();
    init_label_styles();
    tach_init();
    vss_init();
    can_init();
    odometer_init();

    ui_init();
    lv_timer_create(gauge_timer, 10, NULL);

    uart_init(UART_PORT, UART_TX_PIN, UART_PIN_NO_CHANGE, UART_TX_BUF_SIZE, UART_BAUD_RATE); 
    uart_init(UART1_PORT, UART1_TX_PIN, UART_PIN_NO_CHANGE, UART_TX_BUF_SIZE, UART_BAUD_RATE); 
    xTaskCreatePinnedToCore(tach_task, "tach_task", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(vss_task,  "vss_task",  2048, NULL,  9, NULL, 0);
    xTaskCreatePinnedToCore(can_task,  "can_task",  3072, NULL,  8, NULL, 0);
    xTaskCreatePinnedToCore(adc_task, "adc_uart_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(save_miles_task, "save_miles_task", 4096, NULL, 4, NULL, 0);

    bsp_display_backlight_off();
    vTaskDelay(pdMS_TO_TICKS(100));
    bsp_display_brightness_set(BL_NORMAL);

}
