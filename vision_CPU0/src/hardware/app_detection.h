#ifndef APP_DETECTION_H
#define APP_DETECTION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*app_detection_text_writer_t)(char const * p_text);

#define APP_DETECTION_MAX_RESULTS (3U)
#define APP_DETECTION_CLASS_TOMATO       (0U)
#define APP_DETECTION_CLASS_GREEN_GRAPE  (1U)
#define APP_DETECTION_CLASS_PURPLE_GRAPE (2U)

typedef struct st_app_detection_result
{
    uint32_t class_id;
    int32_t  x;
    int32_t  y;
} app_detection_result_t;

bool app_detection_init(void);
bool app_detection_run_frame(uint8_t const                 * p_rgb565_frame,
                             app_detection_result_t        * p_results,
                             uint32_t                        result_capacity,
                             uint32_t                      * p_result_count,
                             app_detection_text_writer_t     write_text);

#ifdef __cplusplus
}
#endif

#endif /* APP_DETECTION_H */
