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
    UI_PAGE_SELECT,
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
static fruit_ui_detection_t g_detections[FRUIT_UI_MAX_DETECTIONS];
static uint32_t g_detection_count;
static volatile fruit_ui_detection_t g_pending_detections[FRUIT_UI_MAX_DETECTIONS];
static volatile uint32_t g_pending_detection_count;
static volatile bool g_target_update_pending;
static volatile int32_t g_pending_weight_0p1g;
static volatile bool g_pending_weight_valid;
static volatile bool g_weight_update_pending;
static int32_t g_weight_0p1g;
static bool g_weight_valid;
static ui_page_t g_current_page = UI_PAGE_HOME;
static lv_obj_t * g_home_weight_label;
static lv_obj_t * g_home_detected_label;
static lv_obj_t * g_home_start_button;
static lv_obj_t * g_detail_type_label;
static lv_obj_t * g_detail_coordinate_label;
static bool g_style_ready;
static lv_style_t g_style_screen;
static lv_style_t g_style_card;
static lv_style_t g_style_button;
static lv_style_t g_style_button_alt;

static void show_home(void);
static void show_select(void);
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

static bool is_target_detected(fruit_ui_target_t target)
{
    for (uint32_t i = 0U; i < g_detection_count; i++) {
        if (g_detections[i].target == target) {
            return true;
        }
    }

    return false;
}

static uint32_t detected_target_mask(void)
{
    uint32_t mask = 0U;

    for (uint32_t i = 0U; i < g_detection_count; i++) {
        mask |= 1UL << (uint32_t) g_detections[i].target;
    }

    return mask;
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

    g_style_ready = true;
}

static void prepare_screen(void)
{
    lv_obj_t * scr = lv_screen_active();

    init_styles();
    g_home_weight_label = NULL;
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

static void add_circle(lv_obj_t * parent, int32_t x, int32_t y, int32_t size, lv_color_t color)
{
    lv_obj_t * obj = lv_obj_create(parent);

    lv_obj_remove_style_all(obj);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, size, size);
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(obj, color, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
}

static void add_circle_center(lv_obj_t * parent, int32_t cx, int32_t cy, int32_t size, lv_color_t color)
{
    add_circle(parent, cx - (size / 2), cy - (size / 2), size, color);
}

static void add_tomato_icon(lv_obj_t * parent, int32_t cx, int32_t cy)
{
    add_circle_center(parent, cx - 16, cy, 52, lv_color_hex(0xD83B35));
    add_circle_center(parent, cx + 16, cy + 2, 50, lv_color_hex(0xE85048));
    add_circle_center(parent, cx - 12, cy - 34, 13, lv_color_hex(0x3F8A3E));
    add_circle_center(parent, cx + 5, cy - 36, 11, lv_color_hex(0x5AA24D));
}

static void add_grape_icon(lv_obj_t * parent, int32_t cx, int32_t cy, lv_color_t color)
{
    add_circle_center(parent, cx - 12, cy - 26, 22, color);
    add_circle_center(parent, cx + 12, cy - 26, 22, color);
    add_circle_center(parent, cx - 24, cy - 6, 22, color);
    add_circle_center(parent, cx,      cy - 6, 22, color);
    add_circle_center(parent, cx + 24, cy - 6, 22, color);
    add_circle_center(parent, cx - 12, cy + 15, 22, color);
    add_circle_center(parent, cx + 12, cy + 15, 22, color);
    add_circle_center(parent, cx,      cy - 48, 10, lv_color_hex(0x4D8C3D));
}

static void update_home_detection_widgets(void)
{
    char detected_text[48] = "None";
    size_t used = 0U;

    if (g_detection_count > 0U) {
        detected_text[0] = '\0';

        for (uint32_t i = 0U; i < g_detection_count; i++) {
            target_view_t const * info = get_target(g_detections[i].target);
            int const count = snprintf(&detected_text[used],
                                       sizeof(detected_text) - used,
                                       "%s%s",
                                       (used > 0U) ? "\n" : "",
                                       info->name);

            if ((count < 0) || ((size_t) count >= (sizeof(detected_text) - used))) {
                break;
            }

            used += (size_t) count;
        }
    }

    if (g_home_detected_label != NULL) {
        lv_label_set_text(g_home_detected_label, detected_text);
        lv_obj_set_style_text_color(g_home_detected_label,
                                    (g_detection_count > 0U) ? lv_color_hex(0x20303F) :
                                                               get_target(FRUIT_UI_TARGET_NONE)->color,
                                    0);
    }

    if (g_home_start_button != NULL) {
        if (g_detection_count == 0U) {
            lv_obj_add_state(g_home_start_button, LV_STATE_DISABLED);
        } else {
            lv_obj_remove_state(g_home_start_button, LV_STATE_DISABLED);
        }
    }
}

static void update_home_weight_widget(void)
{
    char weight_text[32];

    if (g_weight_valid) {
        int32_t const weight_abs = (g_weight_0p1g < 0) ? -g_weight_0p1g : g_weight_0p1g;
        char const * const sign = (g_weight_0p1g < 0) ? "-" : "";

        (void) snprintf(weight_text,
                        sizeof(weight_text),
                        "%s%ld.%ld g",
                        sign,
                        (long) (weight_abs / 10),
                        (long) (weight_abs % 10));
    } else {
        (void) snprintf(weight_text, sizeof(weight_text), "--.- g");
    }

    if (g_home_weight_label != NULL) {
        lv_label_set_text(g_home_weight_label, weight_text);
        lv_obj_set_style_text_color(g_home_weight_label,
                                    g_weight_valid ? lv_color_hex(0x1F7A5A) : lv_color_hex(0x77818C),
                                    0);
    }
}

static void on_home_start(lv_event_t * e)
{
    if ((lv_event_get_code(e) == LV_EVENT_CLICKED) &&
        (g_detection_count > 0U)) {
        show_select();
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
        show_select();
    }
}

static void on_confirm_pick(lv_event_t * e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }

    for (uint32_t i = 0U; i < g_detection_count; i++) {
        if (g_detections[i].target == g_selected) {
            for (uint32_t remaining = i + 1U; remaining < g_detection_count; remaining++) {
                g_detections[remaining - 1U] = g_detections[remaining];
            }

            g_detection_count--;
            break;
        }
    }

    g_selected = FRUIT_UI_TARGET_NONE;
    show_home();
}

static void on_pick(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        fruit_ui_target_t const target = (fruit_ui_target_t) (uintptr_t) lv_event_get_user_data(e);

        if (is_target_detected(target)) {
            g_selected = target;
            show_detail(target);
        }
    }
}

static void show_home(void)
{
    lv_obj_t * card;

    prepare_screen();
    g_current_page = UI_PAGE_HOME;

    add_label(lv_screen_active(), "RENESAS CUP", lv_color_hex(0x1F7A5A),
              &lv_font_montserrat_16, LV_ALIGN_TOP_LEFT, 20, 14);
    add_label(lv_screen_active(), "Fruit Harvest Robot", lv_color_hex(0x20303F),
              &lv_font_montserrat_22, LV_ALIGN_TOP_LEFT, 20, 42);
    add_label(lv_screen_active(), "Vision target recognition and robot arm picking control",
              lv_color_hex(0x687685), &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 20, 74);

    card = add_card(lv_screen_active(), 20, 104, 140, 150);
    add_label(card, "Overview", lv_color_hex(0x20303F),
              &lv_font_montserrat_16, LV_ALIGN_TOP_LEFT, 0, 0);
    add_label(card, "Vision  online\nTouch   ready\nArm     standby",
              lv_color_hex(0x435466), &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 0, 38);

    card = add_card(lv_screen_active(), 170, 104, 140, 150);
    add_label(card, "Weight", lv_color_hex(0x77818C),
              &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 0, 0);
    g_home_weight_label = add_label(card, "--.- g", lv_color_hex(0x77818C),
                                    &lv_font_montserrat_18, LV_ALIGN_CENTER, 0, 8);
    update_home_weight_widget();

    card = add_card(lv_screen_active(), 320, 104, 140, 150);
    add_label(card, "Detected", lv_color_hex(0x77818C),
              &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 0, 0);
    g_home_detected_label = add_label(card, "None", get_target(FRUIT_UI_TARGET_NONE)->color,
                                      &lv_font_montserrat_16, LV_ALIGN_CENTER, 0, 8);

    g_home_start_button = add_button(lv_screen_active(), "START", 138, 274, 204, 36, on_home_start, NULL);
    update_home_detection_widgets();
}

static void add_fruit_column(fruit_ui_target_t target, int32_t x)
{
    lv_obj_t * card = add_card(lv_screen_active(), x, 78, 138, 198);
    target_view_t * info = get_target(target);

    lv_obj_set_style_pad_all(card, 0, 0);

    add_label(card, info->name, lv_color_hex(0x20303F),
              &lv_font_montserrat_12, LV_ALIGN_TOP_MID, 0, 14);

    if (target == FRUIT_UI_TARGET_TOMATO) {
        add_tomato_icon(card, 69, 90);
    } else {
        add_grape_icon(card, 69, 88, info->color);
    }

    add_label(card, info->vision_name, lv_color_hex(0x687685),
              &lv_font_montserrat_10, LV_ALIGN_TOP_MID, 0, 132);
    add_button(card, "SELECT", 22, 154, 94, 32, on_pick, (void *) (uintptr_t) target);
}

static void show_select(void)
{
    prepare_screen();
    g_current_page = UI_PAGE_SELECT;

    add_small_button(lv_screen_active(), "HOME", 12, 12, 68, 30, on_back_home, NULL);
    add_label(lv_screen_active(), "Select Detected Fruit", lv_color_hex(0x20303F),
              &lv_font_montserrat_18, LV_ALIGN_TOP_MID, 0, 16);
    add_label(lv_screen_active(), "Choose one detected fruit for robot-arm planning",
              lv_color_hex(0x687685), &lv_font_montserrat_12, LV_ALIGN_TOP_MID, 0, 46);

    if (g_detection_count > 0U) {
        int32_t const card_width = 138;
        int32_t const card_gap = 19;
        int32_t const total_width = ((int32_t) g_detection_count * card_width) +
                                    (((int32_t) g_detection_count - 1) * card_gap);
        int32_t const start_x = (UI_W - total_width) / 2;

        for (uint32_t i = 0U; i < g_detection_count; i++) {
            add_fruit_column(g_detections[i].target,
                             start_x + ((int32_t) i * (card_width + card_gap)));
        }
    } else {
        add_label(lv_screen_active(), "No fruit detected", get_target(FRUIT_UI_TARGET_NONE)->color,
                  &lv_font_montserrat_18, LV_ALIGN_CENTER, 0, 12);
    }

    add_label(lv_screen_active(), "Camera -> select target -> IK solve -> grip",
              lv_color_hex(0x77818C), &lv_font_montserrat_10, LV_ALIGN_BOTTOM_MID, 0, -12);
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

    add_button(lv_screen_active(), "CONFIRM PICK", 138, 270, 204, 36, on_confirm_pick, NULL);
}

void fruit_ui_create(void)
{
    init_styles();
    show_home();
}

void fruit_ui_process(void)
{
    fruit_ui_detection_t pending_detections[FRUIT_UI_MAX_DETECTIONS];
    uint32_t pending_detection_count = 0U;
    bool has_update = false;
    int32_t pending_weight_0p1g = 0;
    bool pending_weight_valid = false;
    bool has_weight_update = false;

    taskENTER_CRITICAL();
    if (g_target_update_pending) {
        pending_detection_count = g_pending_detection_count;
        for (uint32_t i = 0U; i < pending_detection_count; i++) {
            pending_detections[i].target = g_pending_detections[i].target;
            pending_detections[i].x = g_pending_detections[i].x;
            pending_detections[i].y = g_pending_detections[i].y;
            pending_detections[i].z = g_pending_detections[i].z;
        }
        g_target_update_pending = false;
        has_update = true;
    }
    if (g_weight_update_pending) {
        pending_weight_0p1g = g_pending_weight_0p1g;
        pending_weight_valid = g_pending_weight_valid;
        g_weight_update_pending = false;
        has_weight_update = true;
    }
    taskEXIT_CRITICAL();

    if (has_weight_update) {
        g_weight_0p1g = pending_weight_0p1g;
        g_weight_valid = pending_weight_valid;

        if (g_current_page == UI_PAGE_HOME) {
            update_home_weight_widget();
        }
    }

    if (!has_update) {
        return;
    }

    uint32_t const old_target_mask = detected_target_mask();
    g_detection_count = 0U;

    for (uint32_t i = 0U; i < pending_detection_count; i++) {
        fruit_ui_target_t const target = get_target(pending_detections[i].target)->type;

        if ((target == FRUIT_UI_TARGET_NONE) || is_target_detected(target)) {
            continue;
        }

        g_detections[g_detection_count] = pending_detections[i];
        g_detections[g_detection_count].target = target;
        g_detection_count++;

        target_view_t * info = get_target(target);
        info->x = pending_detections[i].x;
        info->y = pending_detections[i].y;
        info->z = pending_detections[i].z;
    }

    if (g_current_page == UI_PAGE_HOME) {
        update_home_detection_widgets();
    } else if ((g_current_page == UI_PAGE_SELECT) &&
               (old_target_mask != detected_target_mask())) {
        show_select();
    } else if (g_current_page == UI_PAGE_DETAIL) {
        target_view_t * info = get_target(g_selected);
        char coordinate_text[96];

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

void fruit_ui_set_detections(fruit_ui_detection_t const * p_detections, uint32_t detection_count)
{
    if ((detection_count > FRUIT_UI_MAX_DETECTIONS) ||
        ((detection_count > 0U) && (p_detections == NULL))) {
        return;
    }

    taskENTER_CRITICAL();
    for (uint32_t i = 0U; i < detection_count; i++) {
        g_pending_detections[i].target = get_target(p_detections[i].target)->type;
        g_pending_detections[i].x = p_detections[i].x;
        g_pending_detections[i].y = p_detections[i].y;
        g_pending_detections[i].z = p_detections[i].z;
    }
    g_pending_detection_count = detection_count;
    g_target_update_pending = true;
    taskEXIT_CRITICAL();
}

void fruit_ui_set_weight(int32_t weight_0p1g, bool valid)
{
    taskENTER_CRITICAL();
    g_pending_weight_0p1g = weight_0p1g;
    g_pending_weight_valid = valid;
    g_weight_update_pending = true;
    taskEXIT_CRITICAL();
}
