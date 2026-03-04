#include "boot_screen.h"
#include "ui.h"

LV_IMG_DECLARE(logotransparent);

lv_obj_t *boot_screen = NULL;
static lv_obj_t *boot_logo;

/* ---------- Callbacks ---------- */

static void logo_opa_cb(void *obj, int32_t v)
{
    lv_obj_set_style_opa(obj, v, 0);
}

/* ---------- Create Screen ---------- */
void boot_screen_create(void)
{
    boot_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(boot_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(boot_screen, LV_OPA_COVER, 0);

    boot_logo = lv_img_create(boot_screen);
    lv_img_set_src(boot_logo, &logotransparent);
    lv_obj_center(boot_logo);
    lv_obj_set_style_opa(boot_logo, LV_OPA_0, 0);
}

/* ---------- Start Animation ---------- */
// Fades the logo in over 800 ms, starting after a 400 ms delay.
void boot_start(void)
{
    lv_scr_load(boot_screen);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, boot_logo);
    lv_anim_set_exec_cb(&a, logo_opa_cb);
    lv_anim_set_values(&a, 0, 255);
    lv_anim_set_time(&a, 800);
    lv_anim_set_delay(&a, 400);
    lv_anim_start(&a);
}

/* ---------- Exit ---------- */
// Fades the boot screen out and loads next_screen.
// auto_del=true — LVGL deletes boot_screen once the transition is complete.
void boot_finish(lv_obj_t *next_screen)
{
    lv_scr_load_anim(next_screen, LV_SCR_LOAD_ANIM_FADE_ON, 800, 0, true);
    boot_screen = NULL;   // pointer cleared; object freed by LVGL auto_del
}
