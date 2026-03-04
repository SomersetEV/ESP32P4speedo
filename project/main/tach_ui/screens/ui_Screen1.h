// LT230 Speedometer UI — ESP32-P4 (800x800 round display)
// Redesigned for GPS-based speed display with gear indicator

#ifndef UI_SCREEN1_H
#define UI_SCREEN1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

// ── Screen ────────────────────────────────────────────────────────────────
extern lv_obj_t *ui_Screen1;

// ── RPM arc (0-12000 rpm, outer ring)  ───────────────────────────────────
// ui_rpm_arc is the preferred alias; ui_speed_arc kept for compatibility.
extern lv_obj_t *ui_speed_arc;
#define ui_rpm_arc ui_speed_arc

// ── Labels ────────────────────────────────────────────────────────────────
extern lv_obj_t *ui_label_mph_value;       // large speed number (DotoLarge)
extern lv_obj_t *ui_label_kmh_value;       // secondary km/h readout
extern lv_obj_t *ui_label_gear_value;      // hidden compat label
extern lv_obj_t *ui_label_odometer_value;  // odometer miles (Doto_48)

// ── Gear pill row ─────────────────────────────────────────────────────────
// Index:  0=P  1=N  2=R  3=D  4=S
// Colour the active entry from gauge_timer after each gear-detection update.
extern lv_obj_t *ui_gear_labels[5];

// ── Screen lifecycle ─────────────────────────────────────────────────────
void ui_Screen1_screen_init(void);
void ui_Screen1_screen_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // UI_SCREEN1_H
