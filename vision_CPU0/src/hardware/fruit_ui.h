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

#define FRUIT_UI_MAX_DETECTIONS (3U)

typedef struct st_fruit_ui_detection
{
    fruit_ui_target_t target;
    int32_t x;
    int32_t y;
    int32_t z;
} fruit_ui_detection_t;

void fruit_ui_create(void);
void fruit_ui_process(void);
void fruit_ui_set_detections(fruit_ui_detection_t const * p_detections, uint32_t detection_count);
void fruit_ui_set_weight(int32_t weight_0p1g, bool valid);

#ifdef __cplusplus
}
#endif

#endif /* FRUIT_UI_H */
