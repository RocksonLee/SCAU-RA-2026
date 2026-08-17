#include "lv_port_indev.h"

#include "ft6336.h"

#include "FreeRTOS.h"
#include "task.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "lvgl.h"
#pragma GCC diagnostic pop

#define TOUCH_SCREEN_W           480
#define TOUCH_SCREEN_H           320
#define TOUCH_NATIVE_W           320
#define TOUCH_NATIVE_H           480
/* Keep touch/button hit coordinates aligned with the 180-degree LCD rotation. */
#define TOUCH_ROTATION           1
#define TOUCH_RELEASE_DEBOUNCE   2U
#define TOUCH_EVENT_QUEUE_LEN    16U

typedef struct st_touch_event
{
    int32_t x;
    int32_t y;
    bool pressed;
} touch_event_t;

lv_indev_t * g_touch_indev;
static int32_t g_last_x;
static int32_t g_last_y;
static bool g_last_pressed;
static int32_t g_sample_x;
static int32_t g_sample_y;
static bool g_sample_pressed;
static uint8_t g_release_streak;
static touch_event_t g_touch_events[TOUCH_EVENT_QUEUE_LEN];
static uint8_t g_touch_event_head;
static uint8_t g_touch_event_tail;
static uint8_t g_touch_event_count;

static void touch_event_push_locked(int32_t x, int32_t y, bool pressed)
{
    if (g_touch_event_count >= TOUCH_EVENT_QUEUE_LEN) {
        /* Keep the newest transitions if abnormal bouncing fills the queue. */
        g_touch_event_head = (uint8_t) ((g_touch_event_head + 1U) % TOUCH_EVENT_QUEUE_LEN);
        g_touch_event_count--;
    }

    g_touch_events[g_touch_event_tail].x = x;
    g_touch_events[g_touch_event_tail].y = y;
    g_touch_events[g_touch_event_tail].pressed = pressed;
    g_touch_event_tail = (uint8_t) ((g_touch_event_tail + 1U) % TOUCH_EVENT_QUEUE_LEN);
    g_touch_event_count++;
}

void lv_port_indev_get_state(int32_t * out_x, int32_t * out_y, bool * out_pressed)
{
    int32_t x;
    int32_t y;
    bool pressed;

    taskENTER_CRITICAL();
    x = g_last_x;
    y = g_last_y;
    pressed = g_last_pressed;
    taskEXIT_CRITICAL();

    if (out_x != NULL) {
        *out_x = x;
    }
    if (out_y != NULL) {
        *out_y = y;
    }
    if (out_pressed != NULL) {
        *out_pressed = pressed;
    }
}

static int32_t clamp_coord(int32_t value, int32_t upper_bound)
{
    if (value < 0) {
        return 0;
    }
    if (value > upper_bound) {
        return upper_bound;
    }
    return value;
}

static void map_touch_raw_to_screen(uint16_t raw_x, uint16_t raw_y, int32_t * screen_x, int32_t * screen_y)
{
    int32_t mapped_x = (int32_t) raw_x;
    int32_t mapped_y = (int32_t) raw_y;
    int32_t swap_temp;

    switch (TOUCH_ROTATION & 3) {
        case 1:
            swap_temp = mapped_x;
            mapped_x = mapped_y;
            mapped_y = (TOUCH_NATIVE_W - 1) - swap_temp;
            break;
        case 2:
            mapped_x = (TOUCH_NATIVE_W - 1) - mapped_x;
            mapped_y = (TOUCH_NATIVE_H - 1) - mapped_y;
            break;
        case 3:
            swap_temp = mapped_x;
            mapped_x = (TOUCH_NATIVE_H - 1) - mapped_y;
            mapped_y = swap_temp;
            break;
        default:
            break;
    }

    *screen_x = clamp_coord(mapped_x, TOUCH_SCREEN_W - 1);
    *screen_y = clamp_coord(mapped_y, TOUCH_SCREEN_H - 1);
}

static void touch_read_cb(lv_indev_t * indev, lv_indev_data_t * data)
{
    int32_t x;
    int32_t y;
    bool pressed;
    bool more_events;

    (void) indev;

    taskENTER_CRITICAL();
    if (g_touch_event_count > 0U) {
        touch_event_t const * event = &g_touch_events[g_touch_event_head];
        x = event->x;
        y = event->y;
        pressed = event->pressed;
        g_touch_event_head = (uint8_t) ((g_touch_event_head + 1U) % TOUCH_EVENT_QUEUE_LEN);
        g_touch_event_count--;
    } else {
        x = g_sample_x;
        y = g_sample_y;
        pressed = g_sample_pressed;
    }
    more_events = (g_touch_event_count > 0U);
    g_last_x = x;
    g_last_y = y;
    g_last_pressed = pressed;
    taskEXIT_CRITICAL();

    data->point.x = x;
    data->point.y = y;
    data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    data->continue_reading = more_events;
}

void lv_port_indev_sample(void)
{
    ft6336_point_t point = {0};
    int32_t sample_x = g_sample_x;
    int32_t sample_y = g_sample_y;
    bool sample_pressed;
    bool sample_valid = (ft6336_read_point(&point) != 0U);

    if (sample_valid) {
        g_release_streak = 0U;
        map_touch_raw_to_screen(point.x, point.y, &sample_x, &sample_y);
        sample_pressed = true;
    } else if (g_sample_pressed && (g_release_streak < TOUCH_RELEASE_DEBOUNCE)) {
        g_release_streak++;
        sample_pressed = true;
    } else {
        sample_pressed = false;
    }

    taskENTER_CRITICAL();
    if (sample_valid) {
        g_sample_x = sample_x;
        g_sample_y = sample_y;
    }
    if (sample_pressed != g_sample_pressed) {
        touch_event_push_locked(g_sample_x, g_sample_y, sample_pressed);
    }
    g_sample_pressed = sample_pressed;
    taskEXIT_CRITICAL();
}

void lv_port_indev_init(void)
{
    ft6336_init();

    g_last_x = 0;
    g_last_y = 0;
    g_last_pressed = false;
    g_sample_x = 0;
    g_sample_y = 0;
    g_sample_pressed = false;
    g_release_streak = 0U;
    g_touch_event_head = 0U;
    g_touch_event_tail = 0U;
    g_touch_event_count = 0U;

    g_touch_indev = lv_indev_create();
    lv_indev_set_type(g_touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(g_touch_indev, touch_read_cb);
}
