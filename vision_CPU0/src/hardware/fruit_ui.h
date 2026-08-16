#ifndef FRUIT_UI_H
#define FRUIT_UI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum e_fruit_ui_target
{
    FRUIT_UI_TARGET_NONE = 0,
    FRUIT_UI_TARGET_TOMATO,
    FRUIT_UI_TARGET_PURPLE_GRAPE,
    FRUIT_UI_TARGET_GREEN_GRAPE,
} fruit_ui_target_t;

typedef enum e_fruit_ui_task_mode
{
    FRUIT_UI_TASK_NONE = 0,
    FRUIT_UI_TASK_TOP_ONLY,
    FRUIT_UI_TASK_TOP_AND_SIDE,
} fruit_ui_task_mode_t;

#define FRUIT_UI_MAX_DETECTIONS (3U)

typedef struct st_fruit_ui_detection
{
    fruit_ui_target_t target;
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t x1;
    int32_t y1;
    int32_t x2;
    int32_t y2;
    int32_t mean_r;
    int32_t mean_g;
    int32_t mean_b;
    int32_t green_ratio_0p1;
} fruit_ui_detection_t;

void fruit_ui_create(void);
void fruit_ui_process(void);
void fruit_ui_set_detections(fruit_ui_detection_t const * p_detections, uint32_t detection_count);
bool fruit_ui_publish_debug_snapshot(uint8_t const              * p_rgb565_frame,
                                     fruit_ui_detection_t const * p_detections,
                                     uint32_t                     detection_count,
                                     bool                         side_camera);
bool fruit_ui_publish_task_snapshot(uint8_t const              * p_rgb565_frame,
                                    fruit_ui_detection_t const * p_detections,
                                    uint32_t                     detection_count,
                                    bool                         side_camera);
void fruit_ui_set_task_camera(bool side_camera);
bool fruit_ui_is_debug_mode_active(void);
bool fruit_ui_debug_side_camera_requested(void);
bool fruit_ui_debug_light_requested(void);
fruit_ui_task_mode_t fruit_ui_get_task_mode(void);
uint32_t fruit_ui_get_task_generation(void);
bool fruit_ui_take_frame_dump_request(void);
bool fruit_ui_take_arm_zero_request(void);
bool fruit_ui_take_pick_request(fruit_ui_target_t * p_target);
void fruit_ui_notify_pick_sent(fruit_ui_target_t target);
void fruit_ui_set_weight(int32_t weight_0p1g, bool valid);

#ifdef __cplusplus
}
#endif

#endif /* FRUIT_UI_H */
