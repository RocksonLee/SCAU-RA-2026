#ifndef FRUIT_UI_H
#define FRUIT_UI_H

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

void fruit_ui_create(void);
void fruit_ui_process(void);
void fruit_ui_set_target(fruit_ui_target_t target, int32_t x, int32_t y, int32_t z);

#ifdef __cplusplus
}
#endif

#endif /* FRUIT_UI_H */
