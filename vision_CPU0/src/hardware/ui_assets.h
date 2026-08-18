#ifndef UI_ASSETS_H
#define UI_ASSETS_H

#include <stdbool.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "lvgl.h"
#pragma GCC diagnostic pop

#ifdef __cplusplus
extern "C" {
#endif

bool ui_assets_init(void);
bool ui_assets_is_ready(void);
lv_image_dsc_t const * ui_assets_competition_logo(void);
lv_image_dsc_t const * ui_assets_robot_arm(void);

#ifdef __cplusplus
}
#endif

#endif /* UI_ASSETS_H */
