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
#define APP_DETECTION_GREEN_RATIO_DEFAULT (43U)
#define APP_DETECTION_GREEN_RATIO_MIN     (1U)
#define APP_DETECTION_GREEN_RATIO_MAX     (100U)

typedef struct st_app_detection_result
{
    uint32_t class_id;
    int32_t  x;
    int32_t  y;
    int32_t  x1;
    int32_t  y1;
    int32_t  x2;
    int32_t  y2;
    int32_t  mean_r;
    int32_t  mean_g;
    int32_t  mean_b;
    int32_t  green_ratio_0p1;
} app_detection_result_t;

typedef struct st_app_performance_metrics
{
    uint32_t inference_0p1ms;
    uint32_t fps_0p1;
    uint32_t vision_to_execution_0p1ms;
    uint32_t ipc_latency_0p1ms;
    uint32_t cpu1_ram_bytes;
    uint32_t cpu1_flash_bytes;
    uint32_t cpu1_sdram_bytes;
    bool     inference_valid;
    bool     fps_valid;
    bool     vision_to_execution_valid;
    bool     ipc_latency_valid;
    bool     cpu1_memory_valid;
} app_performance_metrics_t;

bool app_detection_init(void);
void app_detection_settings_init(void);
uint32_t app_detection_get_green_ratio_threshold(void);
bool app_detection_set_green_ratio_threshold(uint32_t percent);
bool app_detection_get_top_camera_light_default(void);
bool app_detection_set_top_camera_light_default(bool enabled);
bool app_detection_get_side_camera_light_default(void);
bool app_detection_set_side_camera_light_default(bool enabled);
bool app_detection_run_frame(uint8_t const                 * p_rgb565_frame,
                             app_detection_result_t        * p_results,
                             uint32_t                        result_capacity,
                             uint32_t                      * p_result_count,
                             app_detection_text_writer_t     write_text);
void app_detection_get_performance(app_performance_metrics_t * p_metrics);
void app_detection_performance_set_ipc_latency(uint32_t latency_0p1ms);
void app_detection_performance_track_execution(uint32_t batch_id);
void app_detection_performance_finish_execution(uint32_t batch_id);
void app_detection_performance_set_cpu1_memory(uint32_t message_prefix,
                                               uint32_t bytes);

#ifdef __cplusplus
}
#endif

#endif /* APP_DETECTION_H */
