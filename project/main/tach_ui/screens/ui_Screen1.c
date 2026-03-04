// LT230 Speedometer — ui_Screen1.c
// Target: Waveshare ESP32-P4-WIFI6-Touch-LCD-3.4C   (800 x 800 circular display)
// LVGL 8.3.11
//
// Layout (all coordinates are relative to screen centre 400,400):
//
//   • RPM arc    760 x 760  — bg rail #1a2a1a, active green→amber→red
//   • Tick labels  0/2/4/6/8/10/12 (×1000 RPM) inside the arc (montserrat_28)
//   • "RPM x 1000" descriptor label     dy = -260
//   • Speed number (Montserrat 120 pt)  dy =  -75
//   • "M P H" unit label                dy =  +30
//   • Gear pill row  P N R D S          dy = +130
//   • Odometer "ODO  000000.0  mi"      dy = +235

#include "../ui.h"

// ── Globals declared in header ─────────────────────────────────────────────
lv_obj_t *ui_Screen1          = NULL;
lv_obj_t *ui_speed_arc        = NULL;
lv_obj_t *ui_label_mph_value  = NULL;
lv_obj_t *ui_label_gear_value = NULL;
lv_obj_t *ui_label_odometer_value = NULL;
lv_obj_t *ui_gear_labels[5]   = {NULL};

// ── Helpers ───────────────────────────────────────────────────────────────

// Common style helper: transparent bg + border, no scroll
static void make_transparent(lv_obj_t *obj) {
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(obj, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

// ── Screen init ───────────────────────────────────────────────────────────
void ui_Screen1_screen_init(void)
{
    // ── Screen background ──────────────────────────────────────────────────
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(0x0a0d0a),
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    // ────────────────────────────────────────────────────────────────────────
    // RPM ARC  (outer ring)
    //   Widget  760 x 760  →  outer radius ≈ 380 px, inner ≈ 358 px
    //   bg_angles(140, 40) = 260° sweep, gap at the bottom of the circle.
    //   Range 0-12000 RPM.  green → amber at 6000 → red at 9000 (set in fw).
    // ────────────────────────────────────────────────────────────────────────
    ui_speed_arc = lv_arc_create(ui_Screen1);
    lv_obj_set_size(ui_speed_arc, 760, 760);
    lv_obj_align(ui_speed_arc, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_bg_angles(ui_speed_arc, 140, 40);   // 260° sweep, CW from east
    lv_arc_set_range(ui_speed_arc, 0, 12000);
    lv_arc_set_value(ui_speed_arc, 0);
    lv_obj_clear_flag(ui_speed_arc, LV_OBJ_FLAG_CLICKABLE);

    // Rail (LV_PART_MAIN = background arc)
    lv_obj_set_style_arc_color(ui_speed_arc, lv_color_hex(0x1a2a1a),
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_speed_arc, 22,
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speed_arc, LV_OPA_TRANSP,
                            LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_speed_arc, LV_OPA_TRANSP,
                                LV_PART_MAIN | LV_STATE_DEFAULT);

    // Active indicator
    lv_obj_set_style_arc_color(ui_speed_arc, lv_color_hex(0x00cc44),
                               LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_speed_arc, 22,
                               LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_rounded(ui_speed_arc, true,
                                 LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_speed_arc, LV_OPA_TRANSP,
                            LV_PART_INDICATOR | LV_STATE_DEFAULT);

    // Hide knob
    lv_obj_set_style_bg_opa(ui_speed_arc, LV_OPA_TRANSP,
                            LV_PART_KNOB | LV_STATE_DEFAULT);

    // ────────────────────────────────────────────────────────────────────────
    // TICK RPM LABELS  (0 2 4 6 8 10 12  ×1000 RPM)
    //   Placed at R = 315 px from screen centre, INSIDE the arc inner edge.
    //   Angles from screen-east (3-o'clock), clockwise:
    //     i   krpm  deg    dx     dy
    //     0     0   140  -241   +202
    //     1     2   183  -314    -18
    //     2     4   227  -216   -229
    //     3     6   270     0   -315
    //     4     8   313  +216   -229
    //     5    10   357  +314    -18
    //     6    12    40  +241   +202
    // ────────────────────────────────────────────────────────────────────────
    static const char   *spd_str[] = {"0","2","4","6","8","10","12"};
    static const int16_t tick_dx[] = {-241, -314, -216,    0, +216, +314, +241};
    static const int16_t tick_dy[] = { +202,  -18, -229, -315, -229,  -18, +202};

    for (int i = 0; i < 7; i++) {
        lv_obj_t *tl = lv_label_create(ui_Screen1);
        lv_label_set_text(tl, spd_str[i]);
        lv_obj_set_style_text_color(tl, lv_color_hex(0x556655),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(tl, &lv_font_montserrat_28,
                                   LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align(tl, LV_ALIGN_CENTER, tick_dx[i], tick_dy[i]);
    }

    // ────────────────────────────────────────────────────────────────────────
    // "RPM x 1000" ARC DESCRIPTOR  — above the speed number
    // ────────────────────────────────────────────────────────────────────────
    lv_obj_t *rpm_label = lv_label_create(ui_Screen1);
    lv_label_set_text(rpm_label, "RPM x 1000");
    lv_obj_set_style_text_color(rpm_label, lv_color_hex(0x556655),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(rpm_label, &lv_font_montserrat_32,
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(rpm_label, LV_ALIGN_CENTER, 0, -260);

    // ────────────────────────────────────────────────────────────────────────
    // SPEED NUMBER  (Montserrat 120 pt)
    // ────────────────────────────────────────────────────────────────────────
    ui_label_mph_value = lv_label_create(ui_Screen1);
    lv_label_set_text(ui_label_mph_value, "--");
    lv_obj_set_style_text_color(ui_label_mph_value, lv_color_hex(0xffffff),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_label_mph_value, &ui_font_Montserrat_120,
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(ui_label_mph_value, LV_ALIGN_CENTER, 0, -75);

    // ────────────────────────────────────────────────────────────────────────
    // "M P H" UNIT LABEL
    // ────────────────────────────────────────────────────────────────────────
    lv_obj_t *mph_unit = lv_label_create(ui_Screen1);
    lv_label_set_text(mph_unit, "M P H");
    lv_obj_set_style_text_color(mph_unit, lv_color_hex(0x88aa88),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(mph_unit, &lv_font_montserrat_32,
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_align(mph_unit, LV_ALIGN_CENTER, 0, +30);

    // ────────────────────────────────────────────────────────────────────────
    // GEAR PILL ROW   P  N  R  D  S
    //   360 x 70  pill container, flex row, evenly spaced.
    //   Active gear: N=amber #ffaa00,  D/S=green #00cc44,  P/R=amber.
    //   Inactive: very dark #1e2e1e.
    // ────────────────────────────────────────────────────────────────────────
    lv_obj_t *gear_cont = lv_obj_create(ui_Screen1);
    lv_obj_set_size(gear_cont, 360, 70);
    lv_obj_align(gear_cont, LV_ALIGN_CENTER, 0, +130);
    lv_obj_set_style_bg_color(gear_cont, lv_color_hex(0x0d140d),
                              LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(gear_cont, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(gear_cont, lv_color_hex(0x1e2e1e),
                                  LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(gear_cont, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(gear_cont, 35, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(gear_cont, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(gear_cont, 14, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(gear_cont, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(gear_cont, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(gear_cont, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(gear_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(gear_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(gear_cont,
                          LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    static const char *gear_str[] = {"P","N","R","D","S"};
    for (int i = 0; i < 5; i++) {
        ui_gear_labels[i] = lv_label_create(gear_cont);
        lv_label_set_text(ui_gear_labels[i], gear_str[i]);
        lv_obj_set_style_text_font(ui_gear_labels[i], &ui_font_Montserrat_48,
                                   LV_PART_MAIN | LV_STATE_DEFAULT);
        // All dimmed by default; gauge_timer lights up the active one
        lv_obj_set_style_text_color(ui_gear_labels[i], lv_color_hex(0x1e2e1e),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);
    }
    // Pre-light N on startup (index 1 = N)
    lv_obj_set_style_text_color(ui_gear_labels[1], lv_color_hex(0xffaa00),
                                LV_PART_MAIN | LV_STATE_DEFAULT);

    // Hidden compat label so old references don't cause link errors
    ui_label_gear_value = lv_label_create(ui_Screen1);
    lv_label_set_text(ui_label_gear_value, "N");
    lv_obj_add_flag(ui_label_gear_value, LV_OBJ_FLAG_HIDDEN);

    // ────────────────────────────────────────────────────────────────────────
    // ODOMETER   "ODO   000000.0   mi"
    //   Row container with "ODO" header, value in Doto_48, "mi" suffix.
    // ────────────────────────────────────────────────────────────────────────
    lv_obj_t *odo_row = lv_obj_create(ui_Screen1);
    lv_obj_set_size(odo_row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(odo_row, LV_ALIGN_CENTER, 0, +235);
    make_transparent(odo_row);
    lv_obj_set_flex_flow(odo_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(odo_row,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(odo_row, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *odo_hdr = lv_label_create(odo_row);
    lv_label_set_text(odo_hdr, "ODO");
    lv_obj_set_style_text_color(odo_hdr, lv_color_hex(0x446644),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(odo_hdr, &lv_font_montserrat_14,
                               LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_label_odometer_value = lv_label_create(odo_row);
    lv_label_set_text(ui_label_odometer_value, "000000.0");
    lv_obj_set_style_text_color(ui_label_odometer_value, lv_color_hex(0xaabbaa),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_label_odometer_value, &ui_font_Montserrat_48,
                               LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *odo_mi = lv_label_create(odo_row);
    lv_label_set_text(odo_mi, "mi");
    lv_obj_set_style_text_color(odo_mi, lv_color_hex(0x446644),
                                LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(odo_mi, &lv_font_montserrat_14,
                               LV_PART_MAIN | LV_STATE_DEFAULT);

}

// ── Screen destroy ────────────────────────────────────────────────────────
void ui_Screen1_screen_destroy(void)
{
    if (ui_Screen1) lv_obj_del(ui_Screen1);

    ui_Screen1              = NULL;
    ui_speed_arc            = NULL;
    ui_label_mph_value      = NULL;
    ui_label_gear_value     = NULL;
    ui_label_odometer_value = NULL;
    for (int i = 0; i < 5; i++) ui_gear_labels[i] = NULL;
}
