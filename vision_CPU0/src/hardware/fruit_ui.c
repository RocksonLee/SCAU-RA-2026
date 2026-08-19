#include "fruit_ui.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "app_detection.h"
#include "camera_ov5640.h"
#include "ospi_flash.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "lvgl.h"
#pragma GCC diagnostic pop

#include "ui_assets.h"
#include "competition_title_font.inc"

#define UI_W 480
#define UI_H 320
#define UI_PREVIEW_W (320U)
#define UI_PREVIEW_H (240U)
#define UI_PREVIEW_PIXELS (UI_PREVIEW_W * UI_PREVIEW_H)
#define UI_TARGET_COUNT (4U)
#define UI_WEIGHT_LOCK_DELAY_MS (1500U)
#define UI_WEIGHT_STABLE_SAMPLES (5U)
#define UI_WEIGHT_STABLE_TOLERANCE_0P1G (5)
#define UI_WEIGHT_MIN_VALID_0P1G (20)
#define UI_AXIS_COUNT (5U)
#define UI_AXIS_ANGLE_MIN_DEG (-180)
#define UI_AXIS_ANGLE_MAX_DEG (180)
#define UI_AXIS_ANGLE_QUEUE_LENGTH (16U)

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
    UI_PAGE_STREAM,
    UI_PAGE_SELECT,
    UI_PAGE_DETAIL,
    UI_PAGE_DEBUG,
    UI_PAGE_AXIS_ANGLE,
} ui_page_t;

typedef struct st_axis_angle_request
{
    uint8_t axis;
    int32_t angle_deg;
} axis_angle_request_t;

typedef enum e_target_pick_state
{
    TARGET_PICK_AVAILABLE = 0,
    TARGET_PICK_SENDING,
    TARGET_PICK_WEIGHING,
    TARGET_PICK_COMPLETE,
} target_pick_state_t;

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
static fruit_ui_detection_t g_debug_detections[FRUIT_UI_MAX_DETECTIONS];
static uint32_t g_debug_detection_count;
static volatile fruit_ui_detection_t g_pending_debug_detections[FRUIT_UI_MAX_DETECTIONS];
static volatile uint32_t g_pending_debug_detection_count;
static volatile bool g_debug_detection_update_pending;
static volatile bool g_debug_mode_active;
static volatile bool g_debug_side_camera_requested;
static volatile bool g_debug_light_requested;
static TickType_t g_debug_camera_button_open_tick;
static volatile fruit_ui_task_mode_t g_task_mode = FRUIT_UI_TASK_NONE;
static volatile uint32_t g_task_generation;
static volatile bool g_task_stream_active;
static volatile bool g_task_side_camera;
static volatile bool g_pending_task_side_camera;
static volatile bool g_task_camera_update_pending;
static volatile bool g_frame_dump_request_pending;
static volatile bool g_arm_zero_request_pending;
static volatile bool g_claw_request_pending;
static volatile bool g_claw_open_requested;
static volatile bool g_task_joint5_request_pending;
static volatile int32_t g_task_joint5_angle_deg;
static volatile axis_angle_request_t g_axis_angle_queue[UI_AXIS_ANGLE_QUEUE_LENGTH];
static volatile uint32_t g_axis_angle_queue_write;
static volatile uint32_t g_axis_angle_queue_read;
static uint32_t g_arm_telemetry_valid_mask;
static int32_t g_arm_telemetry_angles_0p1deg[UI_AXIS_COUNT];
static bool g_arm_coordinate_valid;
static int32_t g_arm_x_0p1mm;
static int32_t g_arm_y_0p1mm;
static int32_t g_arm_z_0p1mm;
static uint16_t g_debug_preview_pixels[2][UI_PREVIEW_PIXELS]
    BSP_PLACE_IN_SECTION(".sdram_nocache") BSP_ALIGN_VARIABLE(32);
static lv_image_dsc_t g_debug_preview_dsc[2];
static volatile uint32_t g_debug_preview_display_index;
static volatile uint32_t g_debug_preview_ready_index;
static volatile bool g_debug_preview_ready;
static volatile fruit_ui_target_t g_pending_pick_target;
static volatile bool g_pick_request_pending;
static volatile fruit_ui_target_t g_pending_pick_sent_target;
static volatile bool g_pick_sent_update_pending;
static volatile int32_t g_pending_weight_0p1g;
static volatile bool g_pending_weight_valid;
static volatile bool g_weight_update_pending;
static int32_t g_weight_0p1g;
static bool g_weight_valid;
static target_pick_state_t g_pick_state[UI_TARGET_COUNT];
static int32_t g_locked_weight_0p1g[UI_TARGET_COUNT];
static fruit_ui_target_t g_weighing_target = FRUIT_UI_TARGET_NONE;
static TickType_t g_weighing_start_tick;
static int32_t g_weighing_baseline_0p1g;
static bool g_weight_change_seen;
static int32_t g_weight_stability_reference_0p1g;
static uint32_t g_weight_stability_count;
static ui_page_t g_current_page = UI_PAGE_HOME;
static lv_obj_t * g_page_screen;
static lv_obj_t * g_home_weight_label;
static lv_obj_t * g_home_detected_label;
static lv_obj_t * g_detail_type_label;
static lv_obj_t * g_detail_coordinate_label;
static lv_obj_t * g_debug_threshold_label;
static lv_obj_t * g_debug_save_label;
static lv_obj_t * g_debug_results_label;
static lv_obj_t * g_debug_slider;
static lv_obj_t * g_debug_preview_image;
static lv_obj_t * g_debug_camera_title;
static lv_obj_t * g_debug_camera_button;
static lv_obj_t * g_debug_light_button;
static lv_obj_t * g_top_light_default_button;
static lv_obj_t * g_side_light_default_button;
static lv_obj_t * g_axis_angle_inputs[UI_AXIS_COUNT];
static lv_obj_t * g_axis_current_labels[UI_AXIS_COUNT];
static lv_obj_t * g_axis_angle_dialog;
static lv_obj_t * g_axis_angle_editor;
static lv_obj_t * g_axis_angle_keyboard;
static lv_obj_t * g_axis_angle_status_label;
static lv_obj_t * g_axis_coordinate_label;
static uint32_t g_axis_angle_edit_index;
static lv_obj_t * g_task_preview_image;
static lv_obj_t * g_task_camera_title;
static lv_obj_t * g_task_camera_status;
static lv_obj_t * g_confirm_pick_button;
static lv_obj_t * g_debug_boxes[FRUIT_UI_MAX_DETECTIONS];
static lv_obj_t * g_debug_box_labels[FRUIT_UI_MAX_DETECTIONS];
static bool g_style_ready;
static lv_style_t g_style_screen;
static lv_style_t g_style_card;
static lv_style_t g_style_button;
static lv_style_t g_style_button_alt;

static char const * const g_axis_angle_keyboard_map[] =
{
    "1", "2", "3", LV_SYMBOL_BACKSPACE, "\n",
    "4", "5", "6", "-",                 "\n",
    "7", "8", "9", LV_SYMBOL_CLOSE,     "\n",
    "0", LV_SYMBOL_OK, ""
};

static lv_buttonmatrix_ctrl_t const g_axis_angle_keyboard_ctrl[] =
{
    1, 1, 1, 2,
    1, 1, 1, 2,
    1, 1, 1, 2,
    2, 2
};

static void show_home(void);
static void show_task_stream(void);
static void show_select(void);
static void show_detail(fruit_ui_target_t target);
static void show_debug(void);
static void show_axis_angle(void);

static void init_debug_preview(void)
{
    for (uint32_t i = 0U; i < 2U; i++) {
        memset(&g_debug_preview_dsc[i], 0, sizeof(g_debug_preview_dsc[i]));
        g_debug_preview_dsc[i].header.magic = LV_IMAGE_HEADER_MAGIC;
        g_debug_preview_dsc[i].header.cf = LV_COLOR_FORMAT_RGB565;
        g_debug_preview_dsc[i].header.w = UI_PREVIEW_W;
        g_debug_preview_dsc[i].header.h = UI_PREVIEW_H;
        g_debug_preview_dsc[i].header.stride = UI_PREVIEW_W * sizeof(uint16_t);
        g_debug_preview_dsc[i].data_size = UI_PREVIEW_PIXELS * sizeof(uint16_t);
        g_debug_preview_dsc[i].data = (uint8_t const *) g_debug_preview_pixels[i];
    }
}

static target_view_t * get_target(fruit_ui_target_t target)
{
    for (uint32_t i = 0U; i < (uint32_t) (sizeof(g_targets) / sizeof(g_targets[0])); i++) {
        if (g_targets[i].type == target) {
            return &g_targets[i];
        }
    }

    return &g_targets[0];
}

static uint32_t target_index(fruit_ui_target_t target)
{
    uint32_t const index = (uint32_t) target;

    return (index < UI_TARGET_COUNT) ? index : 0U;
}

static bool pick_flow_busy(void)
{
    for (uint32_t i = 1U; i < UI_TARGET_COUNT; i++) {
        if ((g_pick_state[i] == TARGET_PICK_SENDING) ||
            (g_pick_state[i] == TARGET_PICK_WEIGHING)) {
            return true;
        }
    }

    return false;
}

static bool all_detected_targets_complete(void)
{
    if (g_detection_count == 0U) {
        return false;
    }

    for (uint32_t i = 0U; i < g_detection_count; i++) {
        if (g_pick_state[target_index(g_detections[i].target)] != TARGET_PICK_COMPLETE) {
            return false;
        }
    }

    return true;
}

static void reset_preview_session(void)
{
    taskENTER_CRITICAL();
    g_debug_preview_ready = false;
    g_debug_detection_update_pending = false;
    g_pending_debug_detection_count = 0U;
    taskEXIT_CRITICAL();

    g_debug_detection_count = 0U;
}

static bool weight_is_stable_sample(int32_t weight_0p1g)
{
    int32_t const magnitude = (weight_0p1g < 0) ? -weight_0p1g : weight_0p1g;

    if (magnitude < UI_WEIGHT_MIN_VALID_0P1G) {
        g_weight_stability_count = 0U;
        return false;
    }

    if (g_weight_stability_count == 0U) {
        g_weight_stability_reference_0p1g = weight_0p1g;
        g_weight_stability_count = 1U;
        return false;
    }

    int32_t const delta = weight_0p1g - g_weight_stability_reference_0p1g;
    int32_t const delta_abs = (delta < 0) ? -delta : delta;

    if (delta_abs > UI_WEIGHT_STABLE_TOLERANCE_0P1G) {
        g_weight_stability_reference_0p1g = weight_0p1g;
        g_weight_stability_count = 1U;
        return false;
    }

    g_weight_stability_reference_0p1g =
        ((g_weight_stability_reference_0p1g * (int32_t) g_weight_stability_count) + weight_0p1g) /
        (int32_t) (g_weight_stability_count + 1U);
    g_weight_stability_count++;
    return g_weight_stability_count >= UI_WEIGHT_STABLE_SAMPLES;
}

static void format_weight(char * p_text, size_t text_size, int32_t weight_0p1g)
{
    int32_t const weight_abs = (weight_0p1g < 0) ? -weight_0p1g : weight_0p1g;

    (void) snprintf(p_text,
                    text_size,
                    "%s%ld.%ld g",
                    (weight_0p1g < 0) ? "-" : "",
                    (long) (weight_abs / 10),
                    (long) (weight_abs % 10));
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

static lv_obj_t * page_screen(void)
{
    return (NULL != g_page_screen) ? g_page_screen : lv_screen_active();
}

static void prepare_screen(void)
{
    lv_obj_t * scr;

    init_styles();
    g_page_screen = lv_obj_create(NULL);
    scr = g_page_screen;
    g_home_weight_label = NULL;
    g_home_detected_label = NULL;
    g_detail_type_label = NULL;
    g_detail_coordinate_label = NULL;
    g_debug_threshold_label = NULL;
    g_debug_save_label = NULL;
    g_debug_results_label = NULL;
    g_debug_slider = NULL;
    g_debug_preview_image = NULL;
    g_debug_camera_title = NULL;
    g_debug_camera_button = NULL;
    g_debug_light_button = NULL;
    g_top_light_default_button = NULL;
    g_side_light_default_button = NULL;
    for (uint32_t i = 0U; i < UI_AXIS_COUNT; i++) {
        g_axis_angle_inputs[i] = NULL;
        g_axis_current_labels[i] = NULL;
    }
    g_axis_angle_dialog = NULL;
    g_axis_angle_editor = NULL;
    g_axis_angle_keyboard = NULL;
    g_axis_angle_status_label = NULL;
    g_axis_coordinate_label = NULL;
    g_task_preview_image = NULL;
    g_task_camera_title = NULL;
    g_task_camera_status = NULL;
    g_confirm_pick_button = NULL;
    for (uint32_t i = 0U; i < FRUIT_UI_MAX_DETECTIONS; i++) {
        g_debug_boxes[i] = NULL;
        g_debug_box_labels[i] = NULL;
    }
    lv_obj_add_style(scr, &g_style_screen, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
}

static void finish_screen_switch(void)
{
    if (NULL == g_page_screen) {
        return;
    }

    lv_screen_load_anim(g_page_screen, LV_SCR_LOAD_ANIM_NONE, 0U, 0U, true);
    g_page_screen = NULL;
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

static void add_robot_arm_image(lv_obj_t * parent)
{
    lv_image_dsc_t const * source = ui_assets_robot_arm();
    lv_obj_t * image;

    if (NULL == source) {
        return;
    }

    image = lv_image_create(parent);
    lv_image_set_src(image, source);
    lv_obj_set_pos(image, 5, 5);
}

static void add_preview_backdrop(lv_obj_t * parent, int32_t x, int32_t y)
{
    lv_obj_t * backdrop = lv_obj_create(parent);

    lv_obj_remove_style_all(backdrop);
    lv_obj_set_pos(backdrop, x, y);
    lv_obj_set_size(backdrop, (int32_t) UI_PREVIEW_W, (int32_t) UI_PREVIEW_H);
    lv_obj_set_style_bg_color(backdrop, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(backdrop, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(backdrop, 1, 0);
    lv_obj_set_style_border_color(backdrop, lv_color_hex(0x31445A), 0);
    lv_obj_remove_flag(backdrop, LV_OBJ_FLAG_SCROLLABLE);
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

static int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
{
    return (value < low) ? low : ((value > high) ? high : value);
}

static void update_debug_boxes(void)
{
    for (uint32_t i = 0U; i < FRUIT_UI_MAX_DETECTIONS; i++) {
        if ((g_debug_boxes[i] == NULL) || (g_debug_box_labels[i] == NULL)) {
            continue;
        }

        if ((i >= g_debug_detection_count) ||
            (g_debug_detections[i].x2 <= g_debug_detections[i].x1) ||
            (g_debug_detections[i].y2 <= g_debug_detections[i].y1)) {
            lv_obj_add_flag(g_debug_boxes[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }

        int32_t const x1 = clamp_i32(g_debug_detections[i].x1, 0, (int32_t) CAMERA_OV5640_WIDTH - 1);
        int32_t const y1 = clamp_i32(g_debug_detections[i].y1, 0, (int32_t) CAMERA_OV5640_HEIGHT - 1);
        int32_t const x2 = clamp_i32(g_debug_detections[i].x2, x1 + 1, (int32_t) CAMERA_OV5640_WIDTH);
        int32_t const y2 = clamp_i32(g_debug_detections[i].y2, y1 + 1, (int32_t) CAMERA_OV5640_HEIGHT);
        int32_t const preview_x = 4 + ((x1 * (int32_t) UI_PREVIEW_W) / (int32_t) CAMERA_OV5640_WIDTH);
        int32_t const preview_y = 52 + ((y1 * (int32_t) UI_PREVIEW_H) / (int32_t) CAMERA_OV5640_HEIGHT);
        int32_t const preview_w = ((x2 - x1) * (int32_t) UI_PREVIEW_W) / (int32_t) CAMERA_OV5640_WIDTH;
        int32_t const preview_h = ((y2 - y1) * (int32_t) UI_PREVIEW_H) / (int32_t) CAMERA_OV5640_HEIGHT;
        target_view_t const * info = get_target(g_debug_detections[i].target);
        char const * short_name = (g_debug_detections[i].target == FRUIT_UI_TARGET_GREEN_GRAPE) ? "GREEN" :
                                  (g_debug_detections[i].target == FRUIT_UI_TARGET_PURPLE_GRAPE) ? "PURPLE" :
                                  (g_debug_detections[i].target == FRUIT_UI_TARGET_TOMATO) ? "TOMATO" : "?";

        lv_obj_set_pos(g_debug_boxes[i], preview_x, preview_y);
        lv_obj_set_size(g_debug_boxes[i], (preview_w > 2) ? preview_w : 2,
                                           (preview_h > 2) ? preview_h : 2);
        lv_obj_set_style_border_color(g_debug_boxes[i], info->color, 0);
        lv_obj_set_style_bg_color(g_debug_box_labels[i], info->color, 0);
        lv_label_set_text(g_debug_box_labels[i], short_name);
        lv_obj_remove_flag(g_debug_boxes[i], LV_OBJ_FLAG_HIDDEN);
    }
}

static void update_debug_results_widget(void)
{
    char text[320] = "No fruit detected";
    size_t used = 0U;

    if (g_debug_detection_count > 0U) {
        text[0] = '\0';

        for (uint32_t i = 0U; i < g_debug_detection_count; i++) {
            target_view_t const * info = get_target(g_debug_detections[i].target);
            char const * short_name = (g_debug_detections[i].target == FRUIT_UI_TARGET_GREEN_GRAPE) ? "Green" :
                                      (g_debug_detections[i].target == FRUIT_UI_TARGET_PURPLE_GRAPE) ? "Purple" :
                                      (g_debug_detections[i].target == FRUIT_UI_TARGET_TOMATO) ? "Tomato" :
                                                                                                 info->name;
            int const count = (g_debug_detections[i].mean_r >= 0) ?
                snprintf(&text[used],
                         sizeof(text) - used,
                         "%s%s (%ld,%ld)\nR%ld G%ld B%ld  %ld.%ld%%",
                         (used > 0U) ? "\n" : "",
                         short_name,
                         (long) g_debug_detections[i].x,
                         (long) g_debug_detections[i].y,
                         (long) g_debug_detections[i].mean_r,
                         (long) g_debug_detections[i].mean_g,
                         (long) g_debug_detections[i].mean_b,
                         (long) (g_debug_detections[i].green_ratio_0p1 / 10),
                         (long) (g_debug_detections[i].green_ratio_0p1 % 10)) :
                snprintf(&text[used],
                         sizeof(text) - used,
                         "%s%s (%ld,%ld)  RGB:--",
                         (used > 0U) ? "\n" : "",
                         short_name,
                         (long) g_debug_detections[i].x,
                         (long) g_debug_detections[i].y);

            if ((count < 0) || ((size_t) count >= (sizeof(text) - used))) {
                break;
            }

            used += (size_t) count;
        }
    }

    if (g_debug_results_label != NULL) {
        lv_label_set_text(g_debug_results_label, text);
        lv_obj_set_style_text_color(g_debug_results_label,
                                    (g_debug_detection_count > 0U) ? lv_color_hex(0x20303F) :
                                                                    lv_color_hex(0x77818C),
                                    0);
    }

    update_debug_boxes();
}

static void set_debug_threshold(uint32_t percent)
{
    char text[24];
    bool const saved = app_detection_set_green_ratio_threshold(percent);

    if (g_debug_slider != NULL) {
        lv_slider_set_value(g_debug_slider, (int32_t) percent, LV_ANIM_OFF);
    }

    if (g_debug_threshold_label != NULL) {
        (void) snprintf(text, sizeof(text), "%lu%%", (unsigned long) percent);
        lv_label_set_text(g_debug_threshold_label, text);
    }

    if (g_debug_save_label != NULL) {
        lv_label_set_text(g_debug_save_label, saved ? "Saved immediately" : "Save failed");
        lv_obj_set_style_text_color(g_debug_save_label,
                                    saved ? lv_color_hex(0x1F7A5A) : lv_color_hex(0xD83B35),
                                    0);
    }
}

static void cancel_activity_for_home(void)
{
    taskENTER_CRITICAL();
    g_debug_mode_active = false;
    g_debug_side_camera_requested = false;
    g_debug_light_requested = false;
    g_frame_dump_request_pending = false;
    g_task_joint5_request_pending = false;
    g_task_mode = FRUIT_UI_TASK_NONE;
    g_task_generation++;
    g_task_stream_active = false;
    g_task_side_camera = false;
    g_pending_task_side_camera = false;
    g_task_camera_update_pending = false;
    taskEXIT_CRITICAL();

    reset_preview_session();
}

static void on_task_select(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        fruit_ui_task_mode_t const mode =
            (fruit_ui_task_mode_t) (uintptr_t) lv_event_get_user_data(e);

        if (FRUIT_UI_TASK_NONE == g_task_mode) {
            taskENTER_CRITICAL();
            g_task_mode = mode;
            g_task_generation++;
            g_task_side_camera = false;
            g_task_joint5_angle_deg = (FRUIT_UI_TASK_TOP_ONLY == mode) ? -30 : 80;
            g_task_joint5_request_pending = true;
            taskEXIT_CRITICAL();

            memset(g_pick_state, 0, sizeof(g_pick_state));
            memset(g_locked_weight_0p1g, 0, sizeof(g_locked_weight_0p1g));
            g_detection_count = 0U;
            g_weighing_target = FRUIT_UI_TARGET_NONE;
            g_weight_stability_count = 0U;
            show_task_stream();
        } else if (mode == g_task_mode) {
            if (g_detection_count > 0U) {
                show_select();
            } else {
                show_task_stream();
            }
        }
    }
}

static void on_settings(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        taskENTER_CRITICAL();
        g_debug_side_camera_requested = false;
        g_debug_light_requested = false;
        taskEXIT_CRITICAL();
        g_debug_camera_button_open_tick = xTaskGetTickCount();
        show_debug();
    }
}

static void on_back_debug(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        cancel_activity_for_home();
        show_home();
    }
}

static void on_axis_angle_page(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        taskENTER_CRITICAL();
        g_debug_side_camera_requested = false;
        g_debug_light_requested = false;
        taskEXIT_CRITICAL();
        show_axis_angle();
    }
}

static void on_axis_angle_input(lv_event_t * e)
{
    if ((lv_event_get_code(e) == LV_EVENT_CLICKED) &&
        (g_axis_angle_dialog != NULL) &&
        (g_axis_angle_editor != NULL) &&
        (g_axis_angle_keyboard != NULL)) {
        uint32_t const index = (uint32_t) (uintptr_t) lv_event_get_user_data(e);

        if ((index >= UI_AXIS_COUNT) || (g_axis_angle_inputs[index] == NULL)) {
            return;
        }

        g_axis_angle_edit_index = index;
        lv_textarea_set_text(g_axis_angle_editor,
                             lv_textarea_get_text(g_axis_angle_inputs[index]));
        lv_keyboard_set_textarea(g_axis_angle_keyboard, g_axis_angle_editor);
        lv_obj_remove_flag(g_axis_angle_dialog, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(g_axis_angle_dialog);
    }
}

static void on_axis_angle_keyboard(lv_event_t * e)
{
    lv_event_code_t const code = lv_event_get_code(e);

    if (((LV_EVENT_READY == code) || (LV_EVENT_CANCEL == code)) &&
        (g_axis_angle_dialog != NULL) &&
        (g_axis_angle_editor != NULL) &&
        (g_axis_angle_keyboard != NULL)) {
        if ((LV_EVENT_READY == code) &&
            (g_axis_angle_edit_index < UI_AXIS_COUNT) &&
            (g_axis_angle_inputs[g_axis_angle_edit_index] != NULL)) {
            lv_textarea_set_text(g_axis_angle_inputs[g_axis_angle_edit_index],
                                 lv_textarea_get_text(g_axis_angle_editor));
        }
        lv_keyboard_set_textarea(g_axis_angle_keyboard, NULL);
        lv_obj_add_flag(g_axis_angle_dialog, LV_OBJ_FLAG_HIDDEN);
    }
}

static bool parse_axis_angle(char const * text, int32_t * p_angle_deg)
{
    char * end;
    long value;

    if ((NULL == text) || ('\0' == text[0]) || (NULL == p_angle_deg)) {
        return false;
    }

    value = strtol(text, &end, 10);
    if (('\0' != *end) ||
        (value < UI_AXIS_ANGLE_MIN_DEG) ||
        (value > UI_AXIS_ANGLE_MAX_DEG)) {
        return false;
    }

    *p_angle_deg = (int32_t) value;
    return true;
}

static void on_axis_angle_send(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        uint8_t const axis = (uint8_t) (uintptr_t) lv_event_get_user_data(e);
        int32_t angle_deg;
        uint32_t next;
        bool queued = false;
        bool valid = false;

        if ((axis >= 1U) && (axis <= UI_AXIS_COUNT) &&
            (g_axis_angle_inputs[axis - 1U] != NULL)) {
            valid = parse_axis_angle(
                lv_textarea_get_text(g_axis_angle_inputs[axis - 1U]),
                &angle_deg);
        }

        if (valid) {
            taskENTER_CRITICAL();
            next = (g_axis_angle_queue_write + 1U) % UI_AXIS_ANGLE_QUEUE_LENGTH;
            if (next != g_axis_angle_queue_read) {
                g_axis_angle_queue[g_axis_angle_queue_write].axis = axis;
                g_axis_angle_queue[g_axis_angle_queue_write].angle_deg = angle_deg;
                g_axis_angle_queue_write = next;
                queued = true;
            }
            taskEXIT_CRITICAL();
        }

        if (g_axis_angle_status_label != NULL) {
            char status[56];

            if (!valid) {
                (void) snprintf(status, sizeof(status),
                                "Enter an integer from -180 to 180 deg");
            } else if (queued) {
                (void) snprintf(status, sizeof(status),
                                "Axis %u: %ld deg queued",
                                (unsigned int) axis, (long) angle_deg);
            } else {
                (void) snprintf(status, sizeof(status), "Command queue full");
            }
            lv_label_set_text(g_axis_angle_status_label, status);
            lv_obj_set_style_text_color(g_axis_angle_status_label,
                                        queued ? lv_color_hex(0x1F7A5A) :
                                                 lv_color_hex(0xD83B35),
                                        0);
        }
    }
}

static void on_debug_camera_toggle(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if ((xTaskGetTickCount() - g_debug_camera_button_open_tick) <
            pdMS_TO_TICKS(350U)) {
            return;
        }

        bool const request_side = !g_debug_side_camera_requested;

        taskENTER_CRITICAL();
        g_debug_side_camera_requested = request_side;
        g_debug_preview_ready = false;
        g_debug_detection_update_pending = false;
        g_pending_debug_detection_count = 0U;
        taskEXIT_CRITICAL();

        g_debug_detection_count = 0U;
        update_debug_results_widget();

        if (g_debug_preview_image != NULL) {
            lv_obj_add_flag(g_debug_preview_image, LV_OBJ_FLAG_HIDDEN);
        }

        if (g_debug_camera_title != NULL) {
            lv_label_set_text(g_debug_camera_title,
                              request_side ? "SIDE CAMERA DEBUG" : "TOP CAMERA DEBUG");
        }

        if (g_debug_camera_button != NULL) {
            lv_obj_t * label = lv_obj_get_child(g_debug_camera_button, 0);
            if (label != NULL) {
                lv_label_set_text(label, request_side ? "TOP" : "SIDE");
            }
        }

        if (g_debug_save_label != NULL) {
            lv_label_set_text(g_debug_save_label, "Camera switching...");
            lv_obj_set_style_text_color(g_debug_save_label, lv_color_hex(0x31445A), 0);
        }
    }
}

static void on_debug_slider(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t * slider = lv_event_get_target_obj(e);
        set_debug_threshold((uint32_t) lv_slider_get_value(slider));
    }
}

static void on_debug_minus(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        uint32_t const value = app_detection_get_green_ratio_threshold();
        if (value > APP_DETECTION_GREEN_RATIO_MIN) {
            set_debug_threshold(value - 1U);
        }
    }
}

static void on_debug_plus(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        uint32_t const value = app_detection_get_green_ratio_threshold();
        if (value < APP_DETECTION_GREEN_RATIO_MAX) {
            set_debug_threshold(value + 1U);
        }
    }
}

static void on_debug_uart_dump(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        taskENTER_CRITICAL();
        g_frame_dump_request_pending = true;
        taskEXIT_CRITICAL();

        if (g_debug_save_label != NULL) {
            lv_label_set_text(g_debug_save_label, "UART queued");
            lv_obj_set_style_text_color(g_debug_save_label, lv_color_hex(0x31445A), 0);
        }
    }
}

static void on_home(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        cancel_activity_for_home();
        show_home();
    }
}

static void on_back_select(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        show_select();
    }
}

static void on_arm_zero(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        taskENTER_CRITICAL();
        g_axis_angle_queue_read = g_axis_angle_queue_write;
        g_arm_zero_request_pending = true;
        taskEXIT_CRITICAL();
    }
}

static void on_claw_control(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        bool const open = 0U != (uintptr_t) lv_event_get_user_data(e);

        taskENTER_CRITICAL();
        g_claw_open_requested = open;
        g_claw_request_pending = true;
        taskEXIT_CRITICAL();

        if (g_axis_angle_status_label != NULL) {
            lv_label_set_text(g_axis_angle_status_label,
                              open ? "Claw open queued" : "Claw close queued");
            lv_obj_set_style_text_color(g_axis_angle_status_label,
                                        lv_color_hex(0x1F7A5A), 0);
        }
    }
}

static void on_debug_light_toggle(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        bool const light_requested = !g_debug_light_requested;

        taskENTER_CRITICAL();
        g_debug_light_requested = light_requested;
        taskEXIT_CRITICAL();

        if (g_debug_light_button != NULL) {
            lv_obj_t * label = lv_obj_get_child(g_debug_light_button, 0);
            if (label != NULL) {
                lv_label_set_text(label, light_requested ? "LIGHT OFF" : "LIGHT ON");
            }
            lv_obj_set_style_bg_color(g_debug_light_button,
                                      light_requested ? lv_color_hex(0x1F7A5A) :
                                                        lv_color_hex(0x31445A),
                                      0);
        }
    }
}

static void update_camera_light_default_button(lv_obj_t * button,
                                               char const * camera_name,
                                               bool enabled)
{
    if (button != NULL) {
        char text[24];
        lv_obj_t * label = lv_obj_get_child(button, 0);

        (void) snprintf(text, sizeof(text), "%s: %s",
                        camera_name, enabled ? "ON" : "OFF");
        if (label != NULL) {
            lv_label_set_text(label, text);
        }
        lv_obj_set_style_bg_color(button,
                                  enabled ? lv_color_hex(0x1F7A5A) :
                                            lv_color_hex(0x31445A),
                                  0);
    }
}

static void on_camera_light_default_toggle(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        bool const side_camera =
            (0U != (uint32_t) (uintptr_t) lv_event_get_user_data(e));
        bool enabled;
        bool saved;

        if (side_camera) {
            enabled = !app_detection_get_side_camera_light_default();
            saved = app_detection_set_side_camera_light_default(enabled);
            update_camera_light_default_button(g_side_light_default_button,
                                               "SIDE", enabled);
        } else {
            enabled = !app_detection_get_top_camera_light_default();
            saved = app_detection_set_top_camera_light_default(enabled);
            update_camera_light_default_button(g_top_light_default_button,
                                               "TOP", enabled);
        }

        if (g_debug_save_label != NULL) {
            lv_label_set_text(g_debug_save_label,
                              saved ? "Camera light saved" : "Save failed");
            lv_obj_set_style_text_color(g_debug_save_label,
                                        saved ? lv_color_hex(0x1F7A5A) :
                                                lv_color_hex(0xD83B35),
                                        0);
        }
    }
}

static void on_confirm_pick(lv_event_t * e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }

    for (uint32_t i = 0U; i < g_detection_count; i++) {
        if (g_detections[i].target == g_selected) {
            uint32_t const index = target_index(g_selected);

            if (g_pick_state[index] != TARGET_PICK_AVAILABLE) {
                return;
            }

            g_pick_state[index] = TARGET_PICK_SENDING;
            taskENTER_CRITICAL();
            g_pending_pick_target = g_selected;
            g_pick_request_pending = true;
            taskEXIT_CRITICAL();

            if (g_confirm_pick_button != NULL) {
                lv_obj_add_state(g_confirm_pick_button, LV_STATE_DISABLED);
                lv_obj_t * label = lv_obj_get_child(g_confirm_pick_button, 0);
                if (label != NULL) {
                    lv_label_set_text(label, "SENDING...");
                }
            }
            return;
        }
    }
}

static void on_pick(lv_event_t * e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        fruit_ui_target_t const target = (fruit_ui_target_t) (uintptr_t) lv_event_get_user_data(e);

        if (!pick_flow_busy() && is_target_detected(target) &&
            (g_pick_state[target_index(target)] == TARGET_PICK_AVAILABLE)) {
            g_selected = target;
            show_detail(target);
        }
    }
}

static void show_home(void)
{
    lv_obj_t * card;
    lv_obj_t * title;
    lv_obj_t * logo;
    lv_obj_t * task_1_button;
    lv_obj_t * task_2_button;
    char status[80];

    prepare_screen();
    g_current_page = UI_PAGE_HOME;
    g_task_stream_active = false;

    if (NULL != ui_assets_competition_logo()) {
        logo = lv_image_create(page_screen());
        lv_image_set_src(logo, ui_assets_competition_logo());
        lv_obj_set_pos(logo, 24, 3);
    }

    title = lv_label_create(page_screen());
    lv_label_set_text(title, "2026年全国大学生电子设计竞赛\n信息科技前沿专题赛（瑞萨杯）");
    lv_obj_set_width(title, 348);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x20303F), 0);
    lv_obj_set_style_text_font(title, &name3, 0);
    lv_obj_set_pos(title, 116, 7);

    card = add_card(page_screen(), 18, 96, 180, 210);
    lv_obj_set_style_pad_all(card, 0, 0);
    add_robot_arm_image(card);
    add_label(card, "ROBOT ARM", lv_color_hex(0x77818C),
              &lv_font_montserrat_10, LV_ALIGN_BOTTOM_MID, 0, -3);

    add_label(page_screen(), "FLEXIBLE HARVEST ROBOT", lv_color_hex(0x1F7A5A),
              &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 218, 94);

    task_1_button = add_button(page_screen(), "TASK 1  |  TOP", 236, 119, 224, 40,
                               on_task_select,
                               (void *) (uintptr_t) FRUIT_UI_TASK_TOP_ONLY);
    task_2_button = add_button(page_screen(), "TASK 2  |  TOP + SIDE", 236, 169, 224, 40,
                               on_task_select,
                               (void *) (uintptr_t) FRUIT_UI_TASK_TOP_AND_SIDE);
    add_small_button(page_screen(), "CAMERA TEST", 236, 219, 108, 40, on_settings, NULL);
    add_small_button(page_screen(), "ARM SETTING", 352, 219, 108, 40,
                     on_axis_angle_page, NULL);

    card = add_card(page_screen(), 218, 269, 242, 39);
    lv_obj_set_style_pad_all(card, 5, 0);
    add_label(card, "FLASH", lv_color_hex(0x77818C),
              &lv_font_montserrat_10, LV_ALIGN_TOP_LEFT, 0, 0);
    if (ospi_flash_is_ready()) {
        if (ui_assets_is_ready()) {
            (void) snprintf(status,
                            sizeof(status),
                            "UI READY | %06lX",
                            (unsigned long) ospi_flash_jedec_id());
        } else {
            (void) snprintf(status,
                            sizeof(status),
                            "ASSET ERROR | %06lX",
                            (unsigned long) ospi_flash_jedec_id());
        }
    } else {
        switch (ospi_flash_status()) {
            case OSPI_FLASH_STATUS_OPEN_FAILED:
                (void) snprintf(status, sizeof(status), "OPEN ERROR");
                break;

            case OSPI_FLASH_STATUS_ID_READ_FAILED:
                (void) snprintf(status, sizeof(status), "READ ERROR");
                break;

            case OSPI_FLASH_STATUS_ID_MISMATCH:
                (void) snprintf(status,
                                sizeof(status),
                                "ID ERROR | %06lX",
                                (unsigned long) (ospi_flash_jedec_raw() & 0x00FFFFFFU));
                break;

            case OSPI_FLASH_STATUS_NOT_INITIALIZED:
            default:
                (void) snprintf(status, sizeof(status), "NOT INITIALIZED");
                break;
        }
    }
    add_label(card, status, lv_color_hex(0x435466),
              &lv_font_montserrat_10, LV_ALIGN_TOP_RIGHT, 0, 0);

    if (FRUIT_UI_TASK_TOP_ONLY == g_task_mode) {
        lv_obj_t * label = lv_obj_get_child(task_1_button, 0);
        if (label != NULL) {
            lv_label_set_text(label, "CONTINUE TASK 1");
        }
        lv_obj_add_state(task_2_button, LV_STATE_DISABLED);
    } else if (FRUIT_UI_TASK_TOP_AND_SIDE == g_task_mode) {
        lv_obj_t * label = lv_obj_get_child(task_2_button, 0);
        if (label != NULL) {
            lv_label_set_text(label, "CONTINUE TASK 2");
        }
        lv_obj_add_state(task_1_button, LV_STATE_DISABLED);
    }

    update_home_detection_widgets();
    finish_screen_switch();
}

static void show_task_stream(void)
{
    char const * title;

    prepare_screen();
    g_current_page = UI_PAGE_STREAM;
    g_task_stream_active = false;
    reset_preview_session();

    add_small_button(page_screen(), "HOME", 12, 12, 68, 30, on_home, NULL);

    if (FRUIT_UI_TASK_TOP_AND_SIDE == g_task_mode) {
        title = g_task_side_camera ? "TASK 2 - SIDE CAMERA" : "TASK 2 - TOP CAMERA";
    } else {
        title = "TASK 1 - TOP CAMERA";
    }

    g_task_camera_title = add_label(page_screen(), title, lv_color_hex(0x20303F),
                                    &lv_font_montserrat_18, LV_ALIGN_TOP_MID, 0, 15);
    g_task_camera_status = add_label(page_screen(), "Recognizing fruit...",
                                     lv_color_hex(0x1F7A5A), &lv_font_montserrat_12,
                                     LV_ALIGN_BOTTOM_MID, 0, -8);

    add_preview_backdrop(page_screen(), 80, 52);
    g_task_preview_image = lv_image_create(page_screen());
    lv_obj_set_pos(g_task_preview_image, 80, 52);
    lv_obj_set_style_border_width(g_task_preview_image, 1, 0);
    lv_obj_set_style_border_color(g_task_preview_image, lv_color_hex(0x31445A), 0);
    lv_obj_add_flag(g_task_preview_image, LV_OBJ_FLAG_HIDDEN);

    taskENTER_CRITICAL();
    g_task_stream_active = true;
    taskEXIT_CRITICAL();
    finish_screen_switch();
}

static void show_debug(void)
{
    lv_obj_t * card;
    uint32_t const percent = app_detection_get_green_ratio_threshold();
    bool const top_light_default =
        app_detection_get_top_camera_light_default();
    bool const side_light_default =
        app_detection_get_side_camera_light_default();
    char threshold_text[24];

    prepare_screen();
    g_current_page = UI_PAGE_DEBUG;
    g_task_stream_active = false;
    g_debug_mode_active = false;
    reset_preview_session();

    add_small_button(page_screen(), "HOME", 12, 12, 68, 30, on_back_debug, NULL);
    g_debug_camera_title = add_label(page_screen(), "TOP CAMERA DEBUG", lv_color_hex(0x20303F),
                                     &lv_font_montserrat_16, LV_ALIGN_TOP_MID, 0, 15);
    g_debug_camera_button = add_small_button(page_screen(), "SIDE", 84, 12, 76, 30,
                                             on_debug_camera_toggle, NULL);
    g_debug_light_button = add_small_button(page_screen(), "LIGHT ON", 392, 12, 76, 30,
                                            on_debug_light_toggle, NULL);

    add_preview_backdrop(page_screen(), 4, 52);
    g_debug_preview_image = lv_image_create(page_screen());
    lv_obj_set_pos(g_debug_preview_image, 4, 52);
    lv_obj_set_style_border_width(g_debug_preview_image, 1, 0);
    lv_obj_set_style_border_color(g_debug_preview_image, lv_color_hex(0x31445A), 0);
    lv_obj_add_flag(g_debug_preview_image, LV_OBJ_FLAG_HIDDEN);

    for (uint32_t i = 0U; i < FRUIT_UI_MAX_DETECTIONS; i++) {
        g_debug_boxes[i] = lv_obj_create(page_screen());
        lv_obj_remove_style_all(g_debug_boxes[i]);
        lv_obj_set_style_bg_opa(g_debug_boxes[i], LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(g_debug_boxes[i], 2, 0);
        lv_obj_set_style_radius(g_debug_boxes[i], 0, 0);
        lv_obj_remove_flag(g_debug_boxes[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_remove_flag(g_debug_boxes[i], LV_OBJ_FLAG_SCROLLABLE);

        g_debug_box_labels[i] = lv_label_create(g_debug_boxes[i]);
        lv_obj_remove_style_all(g_debug_box_labels[i]);
        lv_obj_set_style_text_color(g_debug_box_labels[i], lv_color_white(), 0);
        lv_obj_set_style_text_font(g_debug_box_labels[i], &lv_font_montserrat_10, 0);
        lv_obj_set_style_bg_opa(g_debug_box_labels[i], LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(g_debug_box_labels[i], 2, 0);
        lv_obj_set_pos(g_debug_box_labels[i], 1, 1);
        lv_obj_add_flag(g_debug_boxes[i], LV_OBJ_FLAG_HIDDEN);
    }

    card = add_card(page_screen(), 328, 52, 148, 240);
    lv_obj_set_style_pad_all(card, 6, 0);
    add_label(card, "G ratio threshold", lv_color_hex(0x687685),
              &lv_font_montserrat_10, LV_ALIGN_TOP_LEFT, 0, 0);
    (void) snprintf(threshold_text, sizeof(threshold_text), "%lu%%", (unsigned long) percent);
    g_debug_threshold_label = add_label(card, threshold_text, lv_color_hex(0x1F7A5A),
                                        &lv_font_montserrat_16, LV_ALIGN_TOP_RIGHT, 0, -3);

    add_small_button(card, "-", 0, 25, 50, 28, on_debug_minus, NULL);
    add_small_button(card, "+", 84, 25, 50, 28, on_debug_plus, NULL);
    g_debug_slider = lv_slider_create(card);
    lv_obj_set_pos(g_debug_slider, 4, 61);
    lv_obj_set_size(g_debug_slider, 126, 14);
    lv_slider_set_range(g_debug_slider,
                        (int32_t) APP_DETECTION_GREEN_RATIO_MIN,
                        (int32_t) APP_DETECTION_GREEN_RATIO_MAX);
    lv_slider_set_value(g_debug_slider, (int32_t) percent, LV_ANIM_OFF);
    lv_obj_add_event_cb(g_debug_slider, on_debug_slider, LV_EVENT_VALUE_CHANGED, NULL);

    g_debug_save_label = add_label(card, "Saved", lv_color_hex(0x1F7A5A),
                                   &lv_font_montserrat_10, LV_ALIGN_TOP_MID, 0, 80);

    add_label(card, "Detections / ROI", lv_color_hex(0x687685),
              &lv_font_montserrat_10, LV_ALIGN_TOP_LEFT, 0, 100);
    g_debug_results_label = add_label(card, "No fruit detected", lv_color_hex(0x77818C),
                                       &lv_font_montserrat_10, LV_ALIGN_TOP_LEFT, 0, 116);
    lv_obj_set_width(g_debug_results_label, 134);
    lv_obj_set_height(g_debug_results_label, 38);
    lv_label_set_long_mode(g_debug_results_label, LV_LABEL_LONG_CLIP);

    g_top_light_default_button =
        add_small_button(card, "TOP", 0, 160, 64, 28,
                         on_camera_light_default_toggle,
                         (void *) (uintptr_t) 0U);
    update_camera_light_default_button(g_top_light_default_button,
                                       "TOP", top_light_default);

    g_side_light_default_button =
        add_small_button(card, "SIDE", 70, 160, 64, 28,
                         on_camera_light_default_toggle,
                         (void *) (uintptr_t) 1U);
    update_camera_light_default_button(g_side_light_default_button,
                                       "SIDE", side_light_default);

    add_small_button(card, "UART DUMP", 20, 194, 94, 26, on_debug_uart_dump, NULL);
    update_debug_results_widget();

    taskENTER_CRITICAL();
    g_debug_mode_active = true;
    taskEXIT_CRITICAL();
    finish_screen_switch();
}

static void add_fruit_column(fruit_ui_target_t target, int32_t x)
{
    lv_obj_t * card = add_card(page_screen(), x, 72, 138, 210);
    target_view_t * info = get_target(target);
    uint32_t const index = target_index(target);
    target_pick_state_t const state = g_pick_state[index];
    lv_obj_t * button;
    char weight_text[32];

    lv_obj_set_style_pad_all(card, 0, 0);

    add_label(card, info->name, lv_color_hex(0x20303F),
              &lv_font_montserrat_12, LV_ALIGN_TOP_MID, 0, 10);

    if (target == FRUIT_UI_TARGET_TOMATO) {
        add_tomato_icon(card, 69, 78);
    } else {
        add_grape_icon(card, 69, 76, info->color);
    }

    add_label(card, info->vision_name, lv_color_hex(0x687685),
              &lv_font_montserrat_10, LV_ALIGN_TOP_MID, 0, 124);

    if (state == TARGET_PICK_COMPLETE) {
        format_weight(weight_text, sizeof(weight_text), g_locked_weight_0p1g[index]);
    } else if (state == TARGET_PICK_WEIGHING) {
        (void) snprintf(weight_text, sizeof(weight_text), "WEIGHING...");
    } else if (state == TARGET_PICK_SENDING) {
        (void) snprintf(weight_text, sizeof(weight_text), "SENDING...");
    } else {
        (void) snprintf(weight_text, sizeof(weight_text), "WEIGHT --.- g");
    }

    add_label(card,
              weight_text,
              (state == TARGET_PICK_AVAILABLE) ? lv_color_hex(0x77818C) : lv_color_hex(0x1F7A5A),
              &lv_font_montserrat_12,
              LV_ALIGN_TOP_MID,
              0,
              145);

    button = add_button(card,
                        (state == TARGET_PICK_COMPLETE) ? "DONE" : "SELECT",
                        22,
                        170,
                        94,
                        30,
                        on_pick,
                        (void *) (uintptr_t) target);
    if ((state != TARGET_PICK_AVAILABLE) || pick_flow_busy()) {
        lv_obj_add_state(button, LV_STATE_DISABLED);
    }
}

static void show_select(void)
{
    prepare_screen();
    g_current_page = UI_PAGE_SELECT;
    g_task_stream_active = false;

    add_small_button(page_screen(), "HOME", 12, 12, 68, 30, on_home, NULL);
    add_label(page_screen(), "Pick Detected Fruit", lv_color_hex(0x20303F),
              &lv_font_montserrat_18, LV_ALIGN_TOP_MID, 0, 16);
    add_label(page_screen(), "Choose a fruit; completed weights stay locked",
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
        add_label(page_screen(), "No fruit detected", get_target(FRUIT_UI_TARGET_NONE)->color,
                  &lv_font_montserrat_18, LV_ALIGN_CENTER, 0, 12);
    }

    add_label(page_screen(), "Pick -> Confirm Pick -> IPC -> weighing",
              lv_color_hex(0x77818C), &lv_font_montserrat_10, LV_ALIGN_BOTTOM_MID, 0, -5);
    finish_screen_switch();
}

static void show_detail(fruit_ui_target_t target)
{
    lv_obj_t * card;
    char buf[96];
    target_view_t * info = get_target(target);

    prepare_screen();
    g_current_page = UI_PAGE_DETAIL;
    g_task_stream_active = false;

    add_small_button(page_screen(), "BACK", 12, 12, 68, 30, on_back_select, NULL);
    add_small_button(page_screen(), "HOME", 400, 12, 68, 30, on_home, NULL);
    add_label(page_screen(), "Target Detail", lv_color_hex(0x20303F),
              &lv_font_montserrat_18, LV_ALIGN_TOP_MID, 0, 16);

    card = add_card(page_screen(), 20, 62, 150, 82);
    add_label(card, "Fruit Type", lv_color_hex(0x77818C),
              &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 0, 0);
    g_detail_type_label = add_label(card, info->name, info->color,
                                    &lv_font_montserrat_18, LV_ALIGN_CENTER, 0, 12);

    card = add_card(page_screen(), 188, 62, 272, 82);
    add_label(card, "Actual Coordinate", lv_color_hex(0x77818C),
              &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 0, 0);
    (void) snprintf(buf, sizeof(buf), "X: %ld mm   Y: %ld mm\nZ: %ld mm",
                    (long) info->x, (long) info->y, (long) info->z);
    g_detail_coordinate_label = add_label(card, buf, lv_color_hex(0x20303F),
                                          &lv_font_montserrat_16, LV_ALIGN_CENTER, 0, 14);

    card = add_card(page_screen(), 20, 164, 440, 86);
    add_label(card, "Robot Plan", lv_color_hex(0x77818C),
              &lv_font_montserrat_12, LV_ALIGN_TOP_LEFT, 0, 0);
    add_label(card, "1.lock target 2.solve arm pose 3.close gripper 4.detect weight",
              lv_color_hex(0x435466), &lv_font_montserrat_14, LV_ALIGN_TOP_LEFT, 0, 30);

    g_confirm_pick_button = add_button(page_screen(), "CONFIRM PICK", 138, 270, 204, 36,
                                       on_confirm_pick, NULL);
    finish_screen_switch();
}

void fruit_ui_create(void)
{
    app_detection_settings_init();
    init_debug_preview();
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
    fruit_ui_detection_t pending_debug_detections[FRUIT_UI_MAX_DETECTIONS];
    uint32_t pending_debug_detection_count = 0U;
    bool has_debug_update = false;
    uint32_t preview_index = 0U;
    bool has_preview_update = false;
    bool pending_task_side_camera = false;
    bool has_task_camera_update = false;
    fruit_ui_target_t pending_pick_sent_target = FRUIT_UI_TARGET_NONE;
    bool has_pick_sent_update = false;

    taskENTER_CRITICAL();
    if (g_target_update_pending) {
        pending_detection_count = g_pending_detection_count;
        for (uint32_t i = 0U; i < pending_detection_count; i++) {
            pending_detections[i].target = g_pending_detections[i].target;
            pending_detections[i].x = g_pending_detections[i].x;
            pending_detections[i].y = g_pending_detections[i].y;
            pending_detections[i].z = g_pending_detections[i].z;
            pending_detections[i].x1 = g_pending_detections[i].x1;
            pending_detections[i].y1 = g_pending_detections[i].y1;
            pending_detections[i].x2 = g_pending_detections[i].x2;
            pending_detections[i].y2 = g_pending_detections[i].y2;
            pending_detections[i].mean_r = g_pending_detections[i].mean_r;
            pending_detections[i].mean_g = g_pending_detections[i].mean_g;
            pending_detections[i].mean_b = g_pending_detections[i].mean_b;
            pending_detections[i].green_ratio_0p1 = g_pending_detections[i].green_ratio_0p1;
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
    if (g_debug_detection_update_pending) {
        pending_debug_detection_count = g_pending_debug_detection_count;
        for (uint32_t i = 0U; i < pending_debug_detection_count; i++) {
            pending_debug_detections[i].target = g_pending_debug_detections[i].target;
            pending_debug_detections[i].x = g_pending_debug_detections[i].x;
            pending_debug_detections[i].y = g_pending_debug_detections[i].y;
            pending_debug_detections[i].z = g_pending_debug_detections[i].z;
            pending_debug_detections[i].x1 = g_pending_debug_detections[i].x1;
            pending_debug_detections[i].y1 = g_pending_debug_detections[i].y1;
            pending_debug_detections[i].x2 = g_pending_debug_detections[i].x2;
            pending_debug_detections[i].y2 = g_pending_debug_detections[i].y2;
            pending_debug_detections[i].mean_r = g_pending_debug_detections[i].mean_r;
            pending_debug_detections[i].mean_g = g_pending_debug_detections[i].mean_g;
            pending_debug_detections[i].mean_b = g_pending_debug_detections[i].mean_b;
            pending_debug_detections[i].green_ratio_0p1 =
                g_pending_debug_detections[i].green_ratio_0p1;
        }
        g_debug_detection_update_pending = false;
        has_debug_update = true;
    }
    if (g_debug_preview_ready) {
        preview_index = g_debug_preview_ready_index;
        g_debug_preview_display_index = preview_index;
        g_debug_preview_ready = false;
        has_preview_update = true;
    }
    if (g_task_camera_update_pending) {
        pending_task_side_camera = g_pending_task_side_camera;
        g_task_camera_update_pending = false;
        has_task_camera_update = true;
    }
    if (g_pick_sent_update_pending) {
        pending_pick_sent_target = g_pending_pick_sent_target;
        g_pick_sent_update_pending = false;
        has_pick_sent_update = true;
    }
    taskEXIT_CRITICAL();

    if (has_task_camera_update) {
        g_task_side_camera = pending_task_side_camera;

        if (g_current_page == UI_PAGE_STREAM) {
            if (g_task_camera_title != NULL) {
                lv_label_set_text(g_task_camera_title,
                                  pending_task_side_camera ? "TASK 2 - SIDE CAMERA" :
                                                             ((g_task_mode == FRUIT_UI_TASK_TOP_AND_SIDE) ?
                                                              "TASK 2 - TOP CAMERA" :
                                                              "TASK 1 - TOP CAMERA"));
            }
            if (g_task_camera_status != NULL) {
                lv_label_set_text(g_task_camera_status,
                                  pending_task_side_camera ? "Switching to side camera..." :
                                                             "Recognizing fruit...");
            }
            if (g_task_preview_image != NULL) {
                lv_obj_add_flag(g_task_preview_image, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }

    if (has_pick_sent_update) {
        uint32_t const index = target_index(pending_pick_sent_target);

        if ((pending_pick_sent_target != FRUIT_UI_TARGET_NONE) &&
            (g_pick_state[index] == TARGET_PICK_SENDING)) {
            g_pick_state[index] = TARGET_PICK_WEIGHING;
            g_weighing_target = pending_pick_sent_target;
            g_weighing_start_tick = xTaskGetTickCount();
            g_weighing_baseline_0p1g = g_weight_valid ? g_weight_0p1g : 0;
            g_weight_change_seen = false;
            g_weight_stability_count = 0U;
            g_selected = FRUIT_UI_TARGET_NONE;
            show_select();
        }
    }

    if (has_weight_update) {
        g_weight_0p1g = pending_weight_0p1g;
        g_weight_valid = pending_weight_valid;

        if ((g_weighing_target != FRUIT_UI_TARGET_NONE) &&
            pending_weight_valid &&
            ((xTaskGetTickCount() - g_weighing_start_tick) >=
             pdMS_TO_TICKS(UI_WEIGHT_LOCK_DELAY_MS))) {
            int32_t const change = pending_weight_0p1g - g_weighing_baseline_0p1g;
            int32_t const change_abs = (change < 0) ? -change : change;

            if (change_abs >= UI_WEIGHT_MIN_VALID_0P1G) {
                g_weight_change_seen = true;
            }

            if (g_weight_change_seen && weight_is_stable_sample(pending_weight_0p1g)) {
                uint32_t const index = target_index(g_weighing_target);

                g_locked_weight_0p1g[index] = g_weight_stability_reference_0p1g;
                g_pick_state[index] = TARGET_PICK_COMPLETE;
                g_weighing_target = FRUIT_UI_TARGET_NONE;
                g_weight_stability_count = 0U;

                if (all_detected_targets_complete()) {
                    taskENTER_CRITICAL();
                    g_task_mode = FRUIT_UI_TASK_NONE;
                    g_task_stream_active = false;
                    taskEXIT_CRITICAL();
                }

                if (g_current_page == UI_PAGE_SELECT) {
                    show_select();
                }
            }
        }

        if (g_current_page == UI_PAGE_HOME) {
            update_home_weight_widget();
        }
    }

    if (has_debug_update) {
        g_debug_detection_count = pending_debug_detection_count;
        for (uint32_t i = 0U; i < pending_debug_detection_count; i++) {
            g_debug_detections[i] = pending_debug_detections[i];
        }

        if (g_current_page == UI_PAGE_DEBUG) {
            update_debug_results_widget();
        }
    }

    if (has_preview_update && (g_current_page == UI_PAGE_DEBUG) &&
        (g_debug_preview_image != NULL)) {
        lv_image_set_src(g_debug_preview_image, &g_debug_preview_dsc[preview_index]);
        lv_obj_remove_flag(g_debug_preview_image, LV_OBJ_FLAG_HIDDEN);
        lv_obj_invalidate(g_debug_preview_image);

        if (g_debug_save_label != NULL) {
            lv_label_set_text(g_debug_save_label,
                              g_debug_side_camera_requested ? "SIDE active" : "TOP active");
            lv_obj_set_style_text_color(g_debug_save_label, lv_color_hex(0x1F7A5A), 0);
        }
    } else if (has_preview_update && (g_current_page == UI_PAGE_STREAM) &&
               (g_task_preview_image != NULL)) {
        lv_image_set_src(g_task_preview_image, &g_debug_preview_dsc[preview_index]);
        lv_obj_remove_flag(g_task_preview_image, LV_OBJ_FLAG_HIDDEN);
        lv_obj_invalidate(g_task_preview_image);

        if (g_task_camera_status != NULL) {
            lv_label_set_text(g_task_camera_status,
                              g_task_side_camera ? "Side camera active - recognizing..." :
                                                   "Top camera active - recognizing...");
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

    if ((g_current_page == UI_PAGE_HOME) || (g_current_page == UI_PAGE_STREAM)) {
        if (g_detection_count > 0U) {
            show_select();
        } else {
            update_home_detection_widgets();
        }
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
        g_pending_detections[i].x1 = p_detections[i].x1;
        g_pending_detections[i].y1 = p_detections[i].y1;
        g_pending_detections[i].x2 = p_detections[i].x2;
        g_pending_detections[i].y2 = p_detections[i].y2;
        g_pending_detections[i].mean_r = p_detections[i].mean_r;
        g_pending_detections[i].mean_g = p_detections[i].mean_g;
        g_pending_detections[i].mean_b = p_detections[i].mean_b;
        g_pending_detections[i].green_ratio_0p1 = p_detections[i].green_ratio_0p1;
    }
    g_pending_detection_count = detection_count;
    g_target_update_pending = true;
    taskEXIT_CRITICAL();
}

static bool snapshot_owner_active(bool debug_snapshot, bool side_camera)
{
    if (debug_snapshot) {
        return g_debug_mode_active && (side_camera == g_debug_side_camera_requested);
    }

    return !g_debug_mode_active && g_task_stream_active &&
           (side_camera == g_task_side_camera);
}

static bool publish_camera_snapshot(uint8_t const              * p_rgb565_frame,
                                    fruit_ui_detection_t const * p_detections,
                                    uint32_t                     detection_count,
                                    bool                         side_camera,
                                    bool                         debug_snapshot)
{
    uint32_t write_index;

    if ((p_rgb565_frame == NULL) ||
        (detection_count > FRUIT_UI_MAX_DETECTIONS) ||
        ((detection_count > 0U) && (p_detections == NULL)) ||
        !snapshot_owner_active(debug_snapshot, side_camera)) {
        return false;
    }

    taskENTER_CRITICAL();
    if (g_debug_preview_ready) {
        taskEXIT_CRITICAL();
        return false;
    }
    write_index = 1U - g_debug_preview_display_index;
    taskEXIT_CRITICAL();

    for (uint32_t y = 0U; y < UI_PREVIEW_H; y++) {
        uint32_t const src_y = y * 2U;

        for (uint32_t x = 0U; x < UI_PREVIEW_W; x++) {
            uint32_t const src_x = x * 2U;
            uint32_t const src = ((src_y * CAMERA_OV5640_WIDTH) + src_x) *
                                 CAMERA_OV5640_BYTES_PER_PIXEL;
            g_debug_preview_pixels[write_index][(y * UI_PREVIEW_W) + x] =
                (uint16_t) (p_rgb565_frame[src] | ((uint16_t) p_rgb565_frame[src + 1U] << 8));
        }
    }

    __DMB();
    taskENTER_CRITICAL();
    if (g_debug_preview_ready || !snapshot_owner_active(debug_snapshot, side_camera)) {
        taskEXIT_CRITICAL();
        return false;
    }
    for (uint32_t i = 0U; i < detection_count; i++) {
        g_pending_debug_detections[i] = p_detections[i];
    }
    g_pending_debug_detection_count = detection_count;
    g_debug_preview_ready_index = write_index;
    g_debug_detection_update_pending = true;
    g_debug_preview_ready = true;
    taskEXIT_CRITICAL();
    return true;
}

bool fruit_ui_publish_debug_snapshot(uint8_t const              * p_rgb565_frame,
                                     fruit_ui_detection_t const * p_detections,
                                     uint32_t                     detection_count,
                                     bool                         side_camera)
{
    return publish_camera_snapshot(p_rgb565_frame,
                                   p_detections,
                                   detection_count,
                                   side_camera,
                                   true);
}

bool fruit_ui_publish_task_snapshot(uint8_t const              * p_rgb565_frame,
                                    fruit_ui_detection_t const * p_detections,
                                    uint32_t                     detection_count,
                                    bool                         side_camera)
{
    return publish_camera_snapshot(p_rgb565_frame,
                                   p_detections,
                                   detection_count,
                                   side_camera,
                                   false);
}

void fruit_ui_set_task_camera(bool side_camera)
{
    taskENTER_CRITICAL();
    g_pending_task_side_camera = side_camera;
    g_task_side_camera = side_camera;
    g_debug_preview_ready = false;
    g_debug_detection_update_pending = false;
    g_pending_debug_detection_count = 0U;
    g_task_camera_update_pending = true;
    taskEXIT_CRITICAL();
}

bool fruit_ui_is_debug_mode_active(void)
{
    return g_debug_mode_active;
}

bool fruit_ui_debug_side_camera_requested(void)
{
    return g_debug_side_camera_requested;
}

static void format_0p1(char * text, size_t text_size, int32_t value)
{
    long const magnitude = (value < 0) ? -(long) value : (long) value;

    (void) snprintf(text, text_size, "%s%ld.%ld",
                    (value < 0) ? "-" : "",
                    magnitude / 10L,
                    magnitude % 10L);
}

static void update_arm_telemetry_widgets(void)
{
    for (uint32_t i = 0U; i < UI_AXIS_COUNT; i++) {
        if (g_axis_current_labels[i] != NULL) {
            if (0U != (g_arm_telemetry_valid_mask & (1UL << i))) {
                char angle[16];
                char text[24];

                format_0p1(angle, sizeof(angle),
                            g_arm_telemetry_angles_0p1deg[i]);
                (void) snprintf(text, sizeof(text), "CALC %s", angle);
                lv_label_set_text(g_axis_current_labels[i], text);
            } else {
                lv_label_set_text(g_axis_current_labels[i], "CALC --.-");
            }
        }
    }

    if (g_axis_coordinate_label != NULL) {
        if (g_arm_coordinate_valid) {
            char x[16];
            char y[16];
            char z[16];
            char text[72];

            format_0p1(x, sizeof(x), g_arm_x_0p1mm);
            format_0p1(y, sizeof(y), g_arm_y_0p1mm);
            format_0p1(z, sizeof(z), g_arm_z_0p1mm);
            (void) snprintf(text, sizeof(text),
                            "XYZ: %s / %s / %s mm", x, y, z);
            lv_label_set_text(g_axis_coordinate_label, text);
        } else {
            lv_label_set_text(g_axis_coordinate_label,
                              "XYZ: --.- / --.- / --.- mm");
        }
    }
}

static void show_axis_angle(void)
{
    prepare_screen();
    g_current_page = UI_PAGE_AXIS_ANGLE;
    g_task_stream_active = false;
    reset_preview_session();

    /* Manual arm control is independent of camera debug mode. */
    taskENTER_CRITICAL();
    g_debug_mode_active = false;
    g_debug_side_camera_requested = false;
    g_debug_light_requested = false;
    taskEXIT_CRITICAL();

    add_small_button(page_screen(), "HOME", 12, 12, 68, 30, on_back_debug, NULL);
    add_small_button(page_screen(), "ZERO", 84, 12, 68, 30, on_arm_zero, NULL);
    add_small_button(page_screen(), "CLAW CLOSE", 156, 12, 98, 30,
                     on_claw_control, (void *) (uintptr_t) false);
    add_small_button(page_screen(), "CLAW OPEN", 258, 12, 98, 30,
                     on_claw_control, (void *) (uintptr_t) true);
    add_label(page_screen(), "ARM SETTING",
              lv_color_hex(0x20303F), &lv_font_montserrat_16,
              LV_ALIGN_TOP_RIGHT, -12, 18);

    for (uint32_t i = 0U; i < UI_AXIS_COUNT; i++) {
        int32_t const y = 56 + ((int32_t) i * 46);
        uint32_t const axis = i + 1U;
        lv_obj_t * card = add_card(page_screen(), 24, y, 432, 40);
        char axis_text[16];

        lv_obj_set_style_pad_all(card, 6, 0);
        (void) snprintf(axis_text, sizeof(axis_text), "AXIS %lu", (unsigned long) axis);
        add_label(card, axis_text, lv_color_hex(0x20303F),
                  &lv_font_montserrat_14, LV_ALIGN_LEFT_MID, 8, 0);

        g_axis_current_labels[i] =
            add_label(card, "CALC --.-", lv_color_hex(0x1F7A5A),
                      &lv_font_montserrat_10, LV_ALIGN_LEFT_MID, 72, 0);

        g_axis_angle_inputs[i] = lv_textarea_create(card);
        lv_obj_set_pos(g_axis_angle_inputs[i], 156, 2);
        lv_obj_set_size(g_axis_angle_inputs[i], 140, 28);
        lv_textarea_set_one_line(g_axis_angle_inputs[i], true);
        lv_textarea_set_accepted_chars(g_axis_angle_inputs[i], "-0123456789");
        lv_textarea_set_max_length(g_axis_angle_inputs[i], 4U);
        lv_textarea_set_placeholder_text(g_axis_angle_inputs[i], "0");
        lv_textarea_set_align(g_axis_angle_inputs[i], LV_TEXT_ALIGN_CENTER);
        lv_obj_set_style_text_font(g_axis_angle_inputs[i], &lv_font_montserrat_14, 0);
        lv_obj_set_style_pad_all(g_axis_angle_inputs[i], 4, 0);
        lv_obj_add_event_cb(g_axis_angle_inputs[i], on_axis_angle_input,
                            LV_EVENT_CLICKED, (void *) (uintptr_t) i);

        add_button(card, "MOVE", 308, 2, 98, 28, on_axis_angle_send,
                   (void *) (uintptr_t) axis);
    }

    g_axis_angle_status_label = add_label(page_screen(),
                                          "Tap a value, enter angle, then MOVE",
                                          lv_color_hex(0x77818C),
                                          &lv_font_montserrat_10,
                                          LV_ALIGN_BOTTOM_MID, 0, -19);
    g_axis_coordinate_label = add_label(page_screen(),
                                        "XYZ: --.- / --.- / --.- mm",
                                        lv_color_hex(0x20303F),
                                        &lv_font_montserrat_10,
                                        LV_ALIGN_BOTTOM_MID, 0, -4);
    update_arm_telemetry_widgets();

    g_axis_angle_dialog = lv_obj_create(page_screen());
    lv_obj_remove_style_all(g_axis_angle_dialog);
    lv_obj_set_pos(g_axis_angle_dialog, 0, 0);
    lv_obj_set_size(g_axis_angle_dialog, UI_W, UI_H);
    lv_obj_set_style_bg_color(g_axis_angle_dialog, lv_color_hex(0xF4F7F5), 0);
    lv_obj_set_style_bg_opa(g_axis_angle_dialog, LV_OPA_COVER, 0);
    lv_obj_remove_flag(g_axis_angle_dialog, LV_OBJ_FLAG_SCROLLABLE);

    add_label(g_axis_angle_dialog, "ENTER ANGLE  |  -180 TO 180 DEG",
              lv_color_hex(0x20303F), &lv_font_montserrat_16,
              LV_ALIGN_TOP_MID, 0, 10);
    g_axis_angle_editor = lv_textarea_create(g_axis_angle_dialog);
    lv_obj_set_pos(g_axis_angle_editor, 120, 38);
    lv_obj_set_size(g_axis_angle_editor, 240, 48);
    lv_textarea_set_one_line(g_axis_angle_editor, true);
    lv_textarea_set_accepted_chars(g_axis_angle_editor, "-0123456789");
    lv_textarea_set_max_length(g_axis_angle_editor, 4U);
    lv_textarea_set_placeholder_text(g_axis_angle_editor, "0");
    lv_textarea_set_align(g_axis_angle_editor, LV_TEXT_ALIGN_CENTER);
    lv_obj_set_style_text_font(g_axis_angle_editor, &lv_font_montserrat_16, 0);
    lv_obj_set_style_pad_all(g_axis_angle_editor, 10, 0);

    g_axis_angle_keyboard = lv_keyboard_create(g_axis_angle_dialog);
    lv_keyboard_set_map(g_axis_angle_keyboard, LV_KEYBOARD_MODE_NUMBER,
                        g_axis_angle_keyboard_map,
                        g_axis_angle_keyboard_ctrl);
    lv_keyboard_set_mode(g_axis_angle_keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_obj_set_size(g_axis_angle_keyboard, UI_W, 220);
    lv_obj_align(g_axis_angle_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_event_cb(g_axis_angle_keyboard, on_axis_angle_keyboard,
                        LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(g_axis_angle_keyboard, on_axis_angle_keyboard,
                        LV_EVENT_CANCEL, NULL);
    lv_obj_add_flag(g_axis_angle_dialog, LV_OBJ_FLAG_HIDDEN);
    finish_screen_switch();
}

bool fruit_ui_debug_light_requested(void)
{
    return g_debug_light_requested;
}

fruit_ui_task_mode_t fruit_ui_get_task_mode(void)
{
    return g_task_mode;
}

uint32_t fruit_ui_get_task_generation(void)
{
    return g_task_generation;
}

bool fruit_ui_take_frame_dump_request(void)
{
    bool requested;

    taskENTER_CRITICAL();
    requested = g_frame_dump_request_pending;
    g_frame_dump_request_pending = false;
    taskEXIT_CRITICAL();
    return requested;
}

bool fruit_ui_take_arm_zero_request(void)
{
    bool requested;

    taskENTER_CRITICAL();
    requested = g_arm_zero_request_pending;
    g_arm_zero_request_pending = false;
    taskEXIT_CRITICAL();
    return requested;
}

bool fruit_ui_take_claw_request(bool * p_open)
{
    bool requested;

    if (NULL == p_open) {
        return false;
    }

    taskENTER_CRITICAL();
    requested = g_claw_request_pending;
    if (requested) {
        *p_open = g_claw_open_requested;
        g_claw_request_pending = false;
    }
    taskEXIT_CRITICAL();
    return requested;
}

void fruit_ui_notify_claw_result(bool open, bool sent)
{
    if ((g_current_page == UI_PAGE_AXIS_ANGLE) &&
        (g_axis_angle_status_label != NULL)) {
        char status[40];

        (void) snprintf(status, sizeof(status),
                        "Claw %s command %s",
                        open ? "open" : "close",
                        sent ? "sent" : "failed");
        lv_label_set_text(g_axis_angle_status_label, status);
        lv_obj_set_style_text_color(g_axis_angle_status_label,
                                    sent ? lv_color_hex(0x1F7A5A) :
                                           lv_color_hex(0xD83B35),
                                    0);
    }
}

bool fruit_ui_take_task_joint5_request(int32_t * p_angle_deg)
{
    bool requested;

    if (NULL == p_angle_deg) {
        return false;
    }

    taskENTER_CRITICAL();
    requested = g_task_joint5_request_pending;
    if (requested) {
        *p_angle_deg = g_task_joint5_angle_deg;
        g_task_joint5_request_pending = false;
    }
    taskEXIT_CRITICAL();
    return requested;
}

bool fruit_ui_take_axis_angle_request(uint8_t * p_axis, int32_t * p_angle_deg)
{
    bool requested = false;

    if ((NULL == p_axis) || (NULL == p_angle_deg)) {
        return false;
    }

    taskENTER_CRITICAL();
    if (g_axis_angle_queue_read != g_axis_angle_queue_write) {
        *p_axis = g_axis_angle_queue[g_axis_angle_queue_read].axis;
        *p_angle_deg = g_axis_angle_queue[g_axis_angle_queue_read].angle_deg;
        g_axis_angle_queue_read = (g_axis_angle_queue_read + 1U) %
                                  UI_AXIS_ANGLE_QUEUE_LENGTH;
        requested = true;
    }
    taskEXIT_CRITICAL();
    return requested;
}

void fruit_ui_notify_axis_angle_result(uint8_t axis, int32_t angle_deg, bool sent)
{
    char status[48];

    if ((UI_PAGE_AXIS_ANGLE != g_current_page) ||
        (NULL == g_axis_angle_status_label)) {
        return;
    }

    (void) snprintf(status, sizeof(status),
                    sent ? "Axis %u: %ld deg sent" :
                           "Axis %u: %ld deg send failed",
                    (unsigned int) axis, (long) angle_deg);
    lv_label_set_text(g_axis_angle_status_label, status);
    lv_obj_set_style_text_color(g_axis_angle_status_label,
                                sent ? lv_color_hex(0x1F7A5A) :
                                       lv_color_hex(0xD83B35),
                                0);
}

bool fruit_ui_is_arm_setting_active(void)
{
    return UI_PAGE_AXIS_ANGLE == g_current_page;
}

void fruit_ui_set_arm_telemetry(uint32_t        valid_mask,
                                int32_t const * p_angles_0p1deg,
                                bool            coordinate_valid,
                                int32_t         x_0p1mm,
                                int32_t         y_0p1mm,
                                int32_t         z_0p1mm)
{
    if (NULL == p_angles_0p1deg) {
        return;
    }

    g_arm_telemetry_valid_mask = valid_mask & ((1UL << UI_AXIS_COUNT) - 1UL);
    for (uint32_t i = 0U; i < UI_AXIS_COUNT; i++) {
        g_arm_telemetry_angles_0p1deg[i] = p_angles_0p1deg[i];
    }
    g_arm_coordinate_valid = coordinate_valid;
    g_arm_x_0p1mm = x_0p1mm;
    g_arm_y_0p1mm = y_0p1mm;
    g_arm_z_0p1mm = z_0p1mm;

    if (UI_PAGE_AXIS_ANGLE == g_current_page) {
        update_arm_telemetry_widgets();
    }
}

bool fruit_ui_take_pick_request(fruit_ui_target_t * p_target)
{
    if (p_target == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    if (!g_pick_request_pending) {
        taskEXIT_CRITICAL();
        return false;
    }

    *p_target = g_pending_pick_target;
    g_pick_request_pending = false;
    taskEXIT_CRITICAL();
    return true;
}

void fruit_ui_notify_pick_sent(fruit_ui_target_t target)
{
    if (target == FRUIT_UI_TARGET_NONE) {
        return;
    }

    taskENTER_CRITICAL();
    g_pending_pick_sent_target = target;
    g_pick_sent_update_pending = true;
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
