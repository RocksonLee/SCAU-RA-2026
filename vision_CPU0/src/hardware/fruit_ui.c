#include "fruit_ui.h"

#include <stddef.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "lvgl.h"
#pragma GCC diagnostic pop

#define UI_W 480
#define UI_H 320

typedef struct st_target_view
{
    fruit_ui_target_t type;
    char const * name;
    char const * vision_name;
    lv_color_t color;
    int32_t x;
    int32_t y;
    int32_t z;
} target_view_t;

typedef enum e_ui_page
{
    UI_PAGE_HOME = 0,
    UI_PAGE_DETAIL,
} ui_page_t;

static target_view_t g_targets[] =
{
    {FRUIT_UI_TARGET_NONE,         "None",         "no target",    LV_COLOR_MAKE(119, 129, 140), 0,   0,   0},
    {FRUIT_UI_TARGET_TOMATO,       "Tomato",       "red fruit",    LV_COLOR_MAKE(222, 55, 48),  186, 214, 72},
    {FRUIT_UI_TARGET_PURPLE_GRAPE, "Purple Grape", "purple grape", LV_COLOR_MAKE(112, 55, 160), 241, 168, 64},
    {FRUIT_UI_TARGET_GREEN_GRAPE,  "Green Grape",  "green grape",  LV_COLOR_MAKE(78, 165, 75),  142, 238, 69},
};

static fruit_ui_target_t g_selected = FRUIT_UI_TARGET_NONE;
static volatile fruit_ui_target_t g_pending_target = FRUIT_UI_TARGET_NONE;
static volatile int32_t g_pending_x;
static volatile int32_t g_pending_y;
static volatile int32_t g_pending_z;
static volatile bool g_target_update_pending;
static ui_page_t g_current_page = UI_PAGE_HOME;
static lv_obj_t * g_home_detected_label;
static lv_obj_t * g_home_start_button;
static lv_obj_t * g_detail_type_label;
static lv_obj_t * g_detail_coordinate_label;
static bool g_style_ready;
static lv_style_t g_style_screen;
static lv_style_t g_style_card;
static lv_style_t g_style_button;
static lv_style_t g_style_button_alt;
static lv_style_t g_style_chip;

static void show_home(void);
static void show_detail(fruit_ui_target_t target);

static target_view_t * get_target(fruit_ui_target_t target)
{
    for (uint32_t i = 0U; i < (uint32_t) (sizeof(g_targets) / sizeof(g_targets[0])); i++) {
        if (g_targets[i].type == target) {
            return &g_targets[i];
        }
    }

    return &g_targets[0];
}

static void init_styles(void)
{
    if (g_style_ready) {
        return;
    }

    lv_style_init(&g_style_screen);
    lv_style_set_bg_color(&g_style_screen, lv_color_hex(0xF2F5F7));
    lv_style_set_bg_opa(&g_style_screen, LV_OPA_COVER);
    lv_style_set_pad_all(&g_style_screen, 0);

    lv_style_init(&g_style_card);
    lv_style_set_bg_color(&g_style_card, lv_color_white());
    lv_style_set_bg_opa(&g_style_card, LV_OPA_COVER);
    lv_style_set_radius(&g_style_card, 8);
    lv_style_set_border_width(&g_style_card, 1);
    lv_style_set_border_color(&g_style_card, lv_color_hex(0xD8DEE8));
    lv_style_set_pad_all(&g_style_card, 10);
    lv_style_set_shadow_width(&g_style_card, 8);
    lv_style_set_shadow_opa(&g_style_card, LV_OPA_10);
    lv_style_set_shadow_ofs_y(&g_style_card, 3);

    lv_style_init(&g_style_button);
    lv_style_set_bg_color(&g_style_button, lv_color_hex(0x1F7A5A));
    lv_style_set_bg_opa(&g_style_button, LV_OPA_COVER);
    lv_style_set_radius(&g_style_button, 7);
    lv_style_set_border_width(&g_style_button, 0);
    lv_style_set_text_color(&g_style_button, lv_color_white());
    lv_style_set_text_font(&g_style_button, &lv_font_montserrat_16);

    lv_style_init(&g_style_button_alt);
    lv_style_set_bg_color(&g_style_button_alt, lv_color_hex(0x31445A));
    lv_style_set_bg_opa(&g_style_button_alt, LV_OPA_COVER);
    lv_style_set_radius(&g_style_button_alt, 7);
    lv_style_set_border_width(&g_style_button_alt, 0);
    lv_style_set_text_color(&g_style_button_alt, lv_color_white());
    lv_style_set_text_font(&g_style_button_alt, &lv_font_montserrat_14);

    lv_style_init(&g_style_chip);
    lv_style_set_bg_color(&g_style_chip, lv_color_hex(0xE8EEF2));
    lv_style_set_bg_opa(&g_style_chip, LV_OPA_COVER);
    lv_style_set_radius(&g_style_chip, 8);
    lv_style_set_border_width(&g_style_chip, 0);
    lv_style_set_pad_left(&g_style_chip, 8);
    lv_style_set_pad_right(&g_style_chip, 8);
    lv_style_set_pad_top(&g_style_chip, 4);
    lv_style_set_pad_bottom(&g_style_chip, 4);

    g_style_ready = true;
}

static void prepare_screen(void)
{
    lv_obj_t * scr = lv_screen_active();

    init_styles();
    g_home_detected_label = NULL;
    g_home_start_button = NULL;
    g_detail_type_label = NULL;
    g_detail_coordinate_label = NULL;
    lv_obj_clean(scr);
    lv_obj_add_style(scr, &g_style_screen, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
}

static lv_obj_t * add_label(lv_obj_t * parent,
                            char const * text,
                            lv_color_t color,
                            lv_font_t const * font,
                            lv_align_t align,
                            int32_t x_ofs,
                            int32_t y_ofs)
{
    lv_obj_t * label = lv_label_create(parent);

    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, color, 0);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_align(label, align, x_ofs, y_ofs);
    return label;
}

static lv_obj_t * add_button(lv_obj_t * parent,
                             char const * text,
                             int32_t x,
                             int32_t y,
                             int32_t w,
                             int32_t h,
                             lv_event_cb_t cb,
                             void * user_data)
{
    lv_obj_t * btn = lv_button_create(parent);
    lv_obj_t * label;

    lv_obj_remove_style_all(btn);
    lv_obj_add_style(btn, &g_style_button, 0);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, w, h);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);

    label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_center(label);

    return btn;
}

static lv_obj_t * add_small_button(lv_obj_t * parent,
                                   char const * text,
                                   int32_t x,
                                   int32_t y,
                                   int32_t w,
                                   int32_t h,
                                   lv_event_cb_t cb,
                                   void * user_data)
{
    lv_obj_t * btn = lv_button_create(parent);
    lv_obj_t * label;

    lv_obj_remove_style_all(btn);
    lv_obj_add_style(btn, &g_style_button_alt, 0);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, w, h);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);

    label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_center(label);

    return btn;
}

static lv_obj_t * add_card(lv_obj_t * parent, int32_t x, int32_t y, int32_t w, int32_t h)
{
    lv_obj_t * card = lv_obj_create(parent);

    lv_obj_remove_style_all(card);
    lv_obj_add_style(card, &g_style_card, 0);
    lv_obj_set_pos(card, x, y);
    lv_obj_set_size(card, w, h);
    lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    return card;
}

static void on_home_start(lv_event_t * e)
{
    if ((lv_event_get_code(e) == LV_EVENT_CLICKED) &&
        (g_selected != FRUIT_UI_TARGET_NONE)) {
        show_detail(g_selected);
    }
}

static void on_back_home(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        show_home();
    }
}

static void on_back_select(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        show_home();
    }
}

static void show_home(void)
{
    lv_obj_t * card;
    lv_obj_t * chip;
    target_view_t * target = get_target(g_selected);

    prepare_screen();
    g_current_page = UI_PAGE_HOME;

    add_label(lv_screen_active(), "RENESAS CUP", lv_color_hex(0x1F7A5A),
              &lv_font_montserrat_16, LV_ALIGN_TOP_LEFT, 20, 14);
    add_label(lv_screen_active(), "Fruit Harvest Robot", lv_color_hex(0x20303F),
              &lv_font_montserrat_22, LV_ALIGN_TOP_LEFT, 20, 42);
    add_label(lv_screen_active(), "Vision target recognition and robot arm picking control",
              lv_color_hex(0x687685), &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 20, 74);

    card = add_card(lv_screen_active(), 20, 104, 270, 150);
    add_label(card, "System Overview", lv_color_hex(0x20303F),
              &lv_font_montserrat_18, LV_ALIGN_TOP_LEFT, 0, 0);
    add_label(card, "Vision: online\nTouch: ready\nArm link: standby\nMode: target select,weight detect",
              lv_color_hex(0x435466), &lv_font_montserrat_14, LV_ALIGN_TOP_LEFT, 0, 36);

    chip = lv_obj_create(card);
    lv_obj_remove_style_all(chip);
    lv_obj_add_style(chip, &g_style_chip, 0);
    lv_obj_set_size(chip, 124, 28);
    lv_obj_align(chip, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    add_label(chip, "RA8P1 + LVGL", lv_color_hex(0x31445A),
              &lv_font_montserrat_12, LV_ALIGN_CENTER, 0, 0);

    card = add_card(lv_screen_active(), 310, 104, 150, 150);
    add_label(card, "Detected", lv_color_hex(0x77818C),
              &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 0, 0);
    g_home_detected_label = add_label(card, target->name, target->color,
                                      &lv_font_montserrat_16, LV_ALIGN_CENTER, 0, 8);

    g_home_start_button = add_button(lv_screen_active(), "START", 138, 274, 204, 36, on_home_start, NULL);
    if (g_selected == FRUIT_UI_TARGET_NONE) {
        lv_obj_add_state(g_home_start_button, LV_STATE_DISABLED);
    }
}

static void show_detail(fruit_ui_target_t target)
{
    lv_obj_t * card;
    char buf[96];
    target_view_t * info = get_target(target);

    prepare_screen();
    g_current_page = UI_PAGE_DETAIL;

    add_small_button(lv_screen_active(), "BACK", 12, 12, 68, 30, on_back_select, NULL);
    add_label(lv_screen_active(), "Target Detail", lv_color_hex(0x20303F),
              &lv_font_montserrat_18, LV_ALIGN_TOP_MID, 0, 16);

    card = add_card(lv_screen_active(), 20, 62, 150, 82);
    add_label(card, "Fruit Type", lv_color_hex(0x77818C),
              &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 0, 0);
    g_detail_type_label = add_label(card, info->name, info->color,
                                    &lv_font_montserrat_18, LV_ALIGN_CENTER, 0, 12);

    card = add_card(lv_screen_active(), 188, 62, 272, 82);
    add_label(card, "Actual Coordinate", lv_color_hex(0x77818C),
              &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 0, 0);
    (void) snprintf(buf, sizeof(buf), "X: %ld mm   Y: %ld mm\nZ: %ld mm",
                    (long) info->x, (long) info->y, (long) info->z);
    g_detail_coordinate_label = add_label(card, buf, lv_color_hex(0x20303F),
                                          &lv_font_montserrat_16, LV_ALIGN_CENTER, 0, 14);

    card = add_card(lv_screen_active(), 20, 164, 440, 86);
    add_label(card, "Robot Plan", lv_color_hex(0x77818C),
              &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 0, 0);
    add_label(card, "1.lock target 2.solve arm pose 3.close gripper 4.detect weight",
              lv_color_hex(0x435466), &lv_font_montserrat_14, LV_ALIGN_TOP_LEFT, 0, 30);

    add_button(lv_screen_active(), "CONFIRM PICK", 138, 270, 204, 36, on_back_home, NULL);
}

void fruit_ui_create(void)
{
    init_styles();
    show_home();
}

void fruit_ui_process(void)
{
    fruit_ui_target_t pending_target = FRUIT_UI_TARGET_NONE;
    int32_t pending_x = 0;
    int32_t pending_y = 0;
    int32_t pending_z = 0;
    bool has_update = false;

    taskENTER_CRITICAL();
    if (g_target_update_pending) {
        pending_target = g_pending_target;
        pending_x = g_pending_x;
        pending_y = g_pending_y;
        pending_z = g_pending_z;
        g_target_update_pending = false;
        has_update = true;
    }
    taskEXIT_CRITICAL();

    if (!has_update) {
        return;
    }

    target_view_t * info = get_target(pending_target);
    char coordinate_text[96];

    info->x = pending_x;
    info->y = pending_y;
    info->z = pending_z;
    g_selected = info->type;

    if ((g_current_page == UI_PAGE_HOME) && (g_home_detected_label != NULL)) {
        lv_label_set_text(g_home_detected_label, info->name);
        lv_obj_set_style_text_color(g_home_detected_label, info->color, 0);

        if (g_home_start_button != NULL) {
            if (g_selected == FRUIT_UI_TARGET_NONE) {
                lv_obj_add_state(g_home_start_button, LV_STATE_DISABLED);
            } else {
                lv_obj_remove_state(g_home_start_button, LV_STATE_DISABLED);
            }
        }
    } else if (g_current_page == UI_PAGE_DETAIL) {
        if (g_detail_type_label != NULL) {
            lv_label_set_text(g_detail_type_label, info->name);
            lv_obj_set_style_text_color(g_detail_type_label, info->color, 0);
        }

        if (g_detail_coordinate_label != NULL) {
            (void) snprintf(coordinate_text,
                            sizeof(coordinate_text),
                            "X: %ld mm   Y: %ld mm\nZ: %ld mm",
                            (long) info->x,
                            (long) info->y,
                            (long) info->z);
            lv_label_set_text(g_detail_coordinate_label, coordinate_text);
        }
    }
}

void fruit_ui_set_target(fruit_ui_target_t target, int32_t x, int32_t y, int32_t z)
{
    fruit_ui_target_t const valid_target = get_target(target)->type;

    taskENTER_CRITICAL();
    g_pending_target = valid_target;
    g_pending_x = x;
    g_pending_y = y;
    g_pending_z = z;
    g_target_update_pending = true;
    taskEXIT_CRITICAL();
}
