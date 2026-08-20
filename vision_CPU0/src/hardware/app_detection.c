#include "app_detection.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "camera_ov5640.h"
#include "hal_data.h"
#include "ipc_detection_protocol.h"
#include "model.h"
// 【34cm，112】，【36.5，146】[33,92] [37.5,172] [44,283] [47,338] [44.5,300] [40,219]
#define DET_INPUT_SIZE              (256U)
#define DET_NUM_CLASSES             (3U)
#define DET_NUM_VALUES              (35U)
#define DET_REG_BINS                (8U)
#define DET_NUM_LEVELS              (3U)
#define DET_CONF_THRESHOLD          (0.35f)
#define DET_PURPLE_CONF_THRESHOLD   (0.30f)
#define DET_NMS_IOU_THRESHOLD       (0.15f)
#define DET_MAX_CANDIDATES          (64U)
#define DET_MAX_OUTPUTS             APP_DETECTION_MAX_RESULTS
#define     DET_UART_LINE_BYTES         (96U)
#define DET_PIGMENT_CELL_COUNT      (9U)
#define DET_PIGMENT_TYPE_COUNT      (3U)
#define DET_PIGMENT_RED             (0U)
#define DET_PIGMENT_GREEN           (1U)
#define DET_PIGMENT_DARK            (2U)
#define DET_PIGMENT_MIN_PIXELS      (6U)
#define DET_CHROMA_MIN_PERMILLE     (12U)
#define DET_DARK_MIN_PERMILLE       (30U)
#define DET_REFINED_BOX_SCALE_NUM   (3U)
#define DET_REFINED_BOX_SCALE_DEN   (2U)
#define DET_MIN_BOX_SIDE_PX         (36)
#define DET_PIGMENT_CENTER_LIMIT_DIVISOR (8)
/* VBTBKR[0..5]w are reserved for the detection settings record. */
#define DET_SETTINGS_MAGIC_0        (0x47U)
#define DET_SETTINGS_MAGIC_1        (0x50U)
#define DET_SETTINGS_VERSION        (2U)
#define DET_SETTINGS_CHECK_XOR      (0xA5U)
#define DET_SETTINGS_BYTE_MAGIC_0   (0U)
#define DET_SETTINGS_BYTE_MAGIC_1   (1U)
#define DET_SETTINGS_BYTE_VERSION   (2U)
#define DET_SETTINGS_BYTE_PERCENT   (3U)
#define DET_SETTINGS_BYTE_INVERSE   (4U)
#define DET_SETTINGS_BYTE_CHECK     (5U)
/* VBTBKR[6..10] are reserved for the task-camera light setting. */
#define CAMERA_LIGHT_SETTINGS_MAGIC_0      (0x4CU)
#define CAMERA_LIGHT_SETTINGS_MAGIC_1      (0x54U)
#define CAMERA_LIGHT_SETTINGS_VERSION      (2U)
#define CAMERA_LIGHT_SETTINGS_VERSION_OLD  (1U)
#define CAMERA_LIGHT_SETTINGS_CHECK_XOR    (0xA7U)
#define CAMERA_LIGHT_SETTINGS_BYTE_MAGIC_0 (6U)
#define CAMERA_LIGHT_SETTINGS_BYTE_MAGIC_1 (7U)
#define CAMERA_LIGHT_SETTINGS_BYTE_VERSION (8U)
#define CAMERA_LIGHT_SETTINGS_BYTE_FLAGS   (9U)
#define CAMERA_LIGHT_SETTINGS_BYTE_CHECK   (10U)
#define CAMERA_LIGHT_TOP_ENABLED            (0x01U)
#define CAMERA_LIGHT_SIDE_ENABLED           (0x02U)
#define CAMERA_LIGHT_FLAGS_MASK             (0x03U)

typedef struct st_detection_box
{
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    uint32_t class_id;
} detection_box_t;

typedef struct st_pigment_stats
{
    uint32_t count;
    uint64_t sum_x;
    uint64_t sum_y;
    uint64_t sum_x2;
    uint64_t sum_y2;
    uint64_t sum_r;
    uint64_t sum_g;
    uint64_t sum_b;
} pigment_stats_t;

static detection_box_t g_candidates[DET_MAX_CANDIDATES];
static detection_box_t g_outputs[DET_MAX_OUTPUTS];
static pigment_stats_t g_pigment_stats[DET_PIGMENT_TYPE_COUNT][DET_PIGMENT_CELL_COUNT];
static uint32_t g_pigment_cell_samples[DET_PIGMENT_CELL_COUNT];
static char g_det_line[DET_UART_LINE_BYTES];
static bool g_detection_initialized;
static bool g_detection_settings_initialized;
static bool g_camera_light_settings_initialized;
static volatile uint32_t g_green_ratio_threshold = APP_DETECTION_GREEN_RATIO_DEFAULT;
static volatile bool g_top_camera_light_default = true;
static volatile bool g_side_camera_light_default = true;
static volatile uint32_t g_performance_inference_0p1ms;
static volatile uint32_t g_performance_fps_0p1;
static volatile uint32_t g_performance_vision_to_execution_0p1ms;
static volatile uint32_t g_performance_ipc_latency_0p1ms;
static volatile bool g_performance_inference_valid;
static volatile bool g_performance_fps_valid;
static volatile bool g_performance_vision_to_execution_valid;
static volatile bool g_performance_ipc_latency_valid;
static volatile uint32_t g_performance_cpu1_ram_bytes;
static volatile uint32_t g_performance_cpu1_flash_bytes;
static volatile uint32_t g_performance_cpu1_sdram_bytes;
static volatile bool g_performance_cpu1_memory_valid;
static TickType_t g_performance_frame_start_tick;
static TickType_t g_performance_fps_window_tick;
static uint32_t g_performance_fps_window_frames;
static volatile TickType_t g_performance_execution_origin_tick;
static volatile uint32_t g_performance_execution_batch_id;
static volatile bool g_performance_execution_pending;
static bool g_performance_cycle_counter_ready;

static void performance_cycle_counter_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_performance_cycle_counter_ready =
        (0U != (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk));
}

static void performance_record_frame(uint32_t inference_start_cycles,
                                     TickType_t inference_start_tick)
{
    TickType_t const now = xTaskGetTickCount();
    uint32_t inference_0p1ms;

    if (g_performance_cycle_counter_ready && (SystemCoreClock > 0U))
    {
        uint32_t const cycles = DWT->CYCCNT - inference_start_cycles;
        inference_0p1ms = (uint32_t) ((((uint64_t) cycles * 10000ULL) +
                                      ((uint64_t) SystemCoreClock / 2ULL)) /
                                     (uint64_t) SystemCoreClock);
    }
    else
    {
        inference_0p1ms =
            (uint32_t) pdTICKS_TO_MS(now - inference_start_tick) * 10U;
    }

    g_performance_inference_0p1ms = inference_0p1ms;
    g_performance_inference_valid = true;
    if (0U == g_performance_fps_window_tick)
    {
        g_performance_fps_window_tick = now;
        g_performance_fps_window_frames = 1U;
    }
    else
    {
        uint32_t const elapsed_ms =
            (uint32_t) pdTICKS_TO_MS(now - g_performance_fps_window_tick);

        if (elapsed_ms > 2000U)
        {
            /* Exclude time while recognition was deliberately idle. */
            g_performance_fps_window_frames = 1U;
            g_performance_fps_window_tick = now;
        }
        else
        {
            g_performance_fps_window_frames++;
            if (elapsed_ms >= 1000U)
            {
                g_performance_fps_0p1 =
                    (uint32_t) (((uint64_t) g_performance_fps_window_frames *
                                 10000ULL) / (uint64_t) elapsed_ms);
                g_performance_fps_valid = true;
                g_performance_fps_window_frames = 0U;
                g_performance_fps_window_tick = now;
            }
        }
    }
}

static void det_settings_enable_backup_access(void)
{
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_OM_LPC_BATT);
    R_SYSTEM->VBTBER_b.VBAE = 1U;
    R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_OM_LPC_BATT);
}

static uint8_t det_settings_check(uint8_t percent)
{
    return (uint8_t) (DET_SETTINGS_MAGIC_0 ^ DET_SETTINGS_MAGIC_1 ^
                      DET_SETTINGS_VERSION ^ percent ^ (uint8_t) ~percent ^
                      DET_SETTINGS_CHECK_XOR);
}

static uint8_t camera_light_settings_check(uint8_t version, uint8_t flags)
{
    return (uint8_t) (CAMERA_LIGHT_SETTINGS_MAGIC_0 ^ CAMERA_LIGHT_SETTINGS_MAGIC_1 ^
                      version ^ flags ^ CAMERA_LIGHT_SETTINGS_CHECK_XOR);
}

static void camera_light_settings_init(void)
{
    taskENTER_CRITICAL();
    if (g_camera_light_settings_initialized)
    {
        taskEXIT_CRITICAL();
        return;
    }

    det_settings_enable_backup_access();

    uint8_t const version = R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_VERSION];
    uint8_t const flags = R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_FLAGS];
    bool const header_valid =
        (R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_MAGIC_0] ==
         CAMERA_LIGHT_SETTINGS_MAGIC_0) &&
        (R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_MAGIC_1] ==
         CAMERA_LIGHT_SETTINGS_MAGIC_1) &&
        (R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_CHECK] ==
         camera_light_settings_check(version, flags));
    bool const valid = header_valid &&
                       (CAMERA_LIGHT_SETTINGS_VERSION == version) &&
                       (0U == (flags & (uint8_t) ~CAMERA_LIGHT_FLAGS_MASK));
    bool const old_valid = header_valid &&
                           (CAMERA_LIGHT_SETTINGS_VERSION_OLD == version) &&
                           (flags <= 1U);

    if (valid)
    {
        g_top_camera_light_default =
            (0U != (flags & CAMERA_LIGHT_TOP_ENABLED));
        g_side_camera_light_default =
            (0U != (flags & CAMERA_LIGHT_SIDE_ENABLED));
    }
    else if (old_valid)
    {
        /* Preserve the old shared task-light choice for top; side stays on. */
        g_top_camera_light_default = (0U != flags);
        g_side_camera_light_default = true;
    }

    g_camera_light_settings_initialized = true;
    taskEXIT_CRITICAL();
}

void app_detection_settings_init(void)
{
    taskENTER_CRITICAL();
    if (g_detection_settings_initialized)
    {
        taskEXIT_CRITICAL();
        return;
    }

    det_settings_enable_backup_access();

    uint8_t const percent = R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_PERCENT];
    bool const valid =
        (R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_MAGIC_0] == DET_SETTINGS_MAGIC_0) &&
        (R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_MAGIC_1] == DET_SETTINGS_MAGIC_1) &&
        (R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_VERSION] == DET_SETTINGS_VERSION) &&
        (R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_INVERSE] == (uint8_t) ~percent) &&
        (R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_CHECK] == det_settings_check(percent)) &&
        ((uint32_t) percent >= APP_DETECTION_GREEN_RATIO_MIN) &&
        ((uint32_t) percent <= APP_DETECTION_GREEN_RATIO_MAX);

    if (valid)
    {
        g_green_ratio_threshold = percent;
    }

    g_detection_settings_initialized = true;
    taskEXIT_CRITICAL();
}

uint32_t app_detection_get_green_ratio_threshold(void)
{
    app_detection_settings_init();
    return g_green_ratio_threshold;
}

bool app_detection_set_green_ratio_threshold(uint32_t percent)
{
    if ((percent < APP_DETECTION_GREEN_RATIO_MIN) ||
        (percent > APP_DETECTION_GREEN_RATIO_MAX))
    {
        return false;
    }

    app_detection_settings_init();
    g_green_ratio_threshold = percent;

    uint8_t const value = (uint8_t) percent;
    uint8_t const inverse = (uint8_t) ~value;
    uint8_t const check = det_settings_check(value);

    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_OM_LPC_BATT);
    R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_MAGIC_0] = 0U;
    R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_VERSION] = DET_SETTINGS_VERSION;
    R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_PERCENT] = value;
    R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_INVERSE] = inverse;
    R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_CHECK] = check;
    R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_MAGIC_1] = DET_SETTINGS_MAGIC_1;
    R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_MAGIC_0] = DET_SETTINGS_MAGIC_0;
    R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_OM_LPC_BATT);

    return (R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_MAGIC_0] == DET_SETTINGS_MAGIC_0) &&
           (R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_MAGIC_1] == DET_SETTINGS_MAGIC_1) &&
           (R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_VERSION] == DET_SETTINGS_VERSION) &&
           (R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_PERCENT] == value) &&
           (R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_INVERSE] == inverse) &&
           (R_SYSTEM->VBTBKR[DET_SETTINGS_BYTE_CHECK] == check);
}

static bool camera_light_settings_save(void)
{
    uint8_t const flags =
        (g_top_camera_light_default ? CAMERA_LIGHT_TOP_ENABLED : 0U) |
        (g_side_camera_light_default ? CAMERA_LIGHT_SIDE_ENABLED : 0U);
    uint8_t const check =
        camera_light_settings_check(CAMERA_LIGHT_SETTINGS_VERSION, flags);

    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_OM_LPC_BATT);
    R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_MAGIC_0] = 0U;
    R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_VERSION] =
        CAMERA_LIGHT_SETTINGS_VERSION;
    R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_FLAGS] = flags;
    R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_CHECK] = check;
    R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_MAGIC_1] =
        CAMERA_LIGHT_SETTINGS_MAGIC_1;
    R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_MAGIC_0] =
        CAMERA_LIGHT_SETTINGS_MAGIC_0;
    R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_OM_LPC_BATT);

    return (R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_MAGIC_0] ==
            CAMERA_LIGHT_SETTINGS_MAGIC_0) &&
           (R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_MAGIC_1] ==
            CAMERA_LIGHT_SETTINGS_MAGIC_1) &&
           (R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_VERSION] ==
            CAMERA_LIGHT_SETTINGS_VERSION) &&
           (R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_FLAGS] == flags) &&
           (R_SYSTEM->VBTBKR[CAMERA_LIGHT_SETTINGS_BYTE_CHECK] == check);
}

bool app_detection_get_top_camera_light_default(void)
{
    camera_light_settings_init();
    return g_top_camera_light_default;
}

bool app_detection_set_top_camera_light_default(bool enabled)
{
    camera_light_settings_init();
    g_top_camera_light_default = enabled;
    return camera_light_settings_save();
}

bool app_detection_get_side_camera_light_default(void)
{
    camera_light_settings_init();
    return g_side_camera_light_default;
}

bool app_detection_set_side_camera_light_default(bool enabled)
{
    camera_light_settings_init();
    g_side_camera_light_default = enabled;
    return camera_light_settings_save();
}

static const uint32_t g_nanodet_grids[DET_NUM_LEVELS] =
{
    32U,
    16U,
    8U,
};

static const float g_nanodet_strides[DET_NUM_LEVELS] =
{
    8.0f,
    16.0f,
    32.0f,
};

static float det_maxf(float a, float b)
{
    return (a > b) ? a : b;
}

static float det_minf(float a, float b)
{
    return (a < b) ? a : b;
}

static float det_clampf(float value, float low, float high)
{
    return det_minf(det_maxf(value, low), high);
}

static uint8_t det_rgb565_r(uint16_t pixel)
{
    uint8_t const r5 = (uint8_t) ((pixel >> 11) & 0x1FU);
    return (uint8_t) ((r5 << 3) | (r5 >> 2));
}

static uint8_t det_rgb565_g(uint16_t pixel)
{
    uint8_t const g6 = (uint8_t) ((pixel >> 5) & 0x3FU);
    return (uint8_t) ((g6 << 2) | (g6 >> 4));
}

static uint8_t det_rgb565_b(uint16_t pixel)
{
    uint8_t const b5 = (uint8_t) (pixel & 0x1FU);
    return (uint8_t) ((b5 << 3) | (b5 >> 2));
}

static void det_preprocess_rgb565(uint8_t const * p_frame)
{
    float * const p_input = GetModelInputPtr_data();
    uint32_t const crop_size = CAMERA_OV5640_HEIGHT;
    uint32_t const crop_x = (CAMERA_OV5640_WIDTH - crop_size) / 2U;
    uint32_t const crop_y = 0U;
    uint32_t const plane_size = DET_INPUT_SIZE * DET_INPUT_SIZE;

    for (uint32_t y = 0U; y < DET_INPUT_SIZE; y++)
    {
        uint32_t const src_y = crop_y + ((y * crop_size) / DET_INPUT_SIZE);

        for (uint32_t x = 0U; x < DET_INPUT_SIZE; x++)
        {
            uint32_t const src_x = crop_x + ((x * crop_size) / DET_INPUT_SIZE);
            uint32_t const src_index = ((src_y * CAMERA_OV5640_WIDTH) + src_x) * CAMERA_OV5640_BYTES_PER_PIXEL;
            uint16_t const pixel = (uint16_t) (p_frame[src_index] | ((uint16_t) p_frame[src_index + 1U] << 8));
            uint32_t const dst_index = (y * DET_INPUT_SIZE) + x;

            p_input[dst_index] = (float) det_rgb565_b(pixel) / 255.0f;
            p_input[plane_size + dst_index] = (float) det_rgb565_g(pixel) / 255.0f;
            p_input[(2U * plane_size) + dst_index] = (float) det_rgb565_r(pixel) / 255.0f;
        }
    }
}

static float det_class_conf_threshold(uint32_t class_id)
{
    return (APP_DETECTION_CLASS_PURPLE_GRAPE == class_id) ?
           DET_PURPLE_CONF_THRESHOLD : DET_CONF_THRESHOLD;
}

static void det_insert_candidate(detection_box_t const * p_box, uint32_t * p_count)
{
    if (p_box->score < det_class_conf_threshold(p_box->class_id))
    {
        return;
    }

    uint32_t insert = *p_count;

    while ((insert > 0U) && (g_candidates[insert - 1U].score < p_box->score))
    {
        if (insert < DET_MAX_CANDIDATES)
        {
            g_candidates[insert] = g_candidates[insert - 1U];
        }

        insert--;
    }

    if (insert < DET_MAX_CANDIDATES)
    {
        g_candidates[insert] = *p_box;
    }

    if (*p_count < DET_MAX_CANDIDATES)
    {
        (*p_count)++;
    }
}

static float det_nanodet_distance(float const logits[DET_REG_BINS])
{
    float maximum = logits[0];
    for (uint32_t bin = 1U; bin < DET_REG_BINS; bin++)
    {
        maximum = det_maxf(maximum, logits[bin]);
    }

    float sum = 0.0f;
    float weighted_sum = 0.0f;
    for (uint32_t bin = 0U; bin < DET_REG_BINS; bin++)
    {
        float const probability = expf(logits[bin] - maximum);
        sum += probability;
        weighted_sum += probability * (float) bin;
    }

    return (sum > 0.0f) ? (weighted_sum / sum) : 0.0f;
}

static void det_decode_nanodet(float const * p_output, uint32_t * p_candidate_count)
{
    uint32_t point_offset = 0U;

    for (uint32_t level = 0U; level < DET_NUM_LEVELS; level++)
    {
        uint32_t const grid = g_nanodet_grids[level];
        float const stride = g_nanodet_strides[level];

        for (uint32_t gy = 0U; gy < grid; gy++)
        {
            for (uint32_t gx = 0U; gx < grid; gx++)
            {
                uint32_t const point = point_offset + (gy * grid) + gx;
                float const * const values = &p_output[point * DET_NUM_VALUES];
                float score = values[0];
                uint32_t class_id = 0U;

                for (uint32_t c = 1U; c < DET_NUM_CLASSES; c++)
                {
                    if (values[c] > score)
                    {
                        score = values[c];
                        class_id = c;
                    }
                }

                if (score < det_class_conf_threshold(class_id))
                {
                    continue;
                }

                float const center_x = (float) gx * stride;
                float const center_y = (float) gy * stride;
                float const left = det_nanodet_distance(&values[DET_NUM_CLASSES]) * stride;
                float const top = det_nanodet_distance(&values[DET_NUM_CLASSES + DET_REG_BINS]) * stride;
                float const right = det_nanodet_distance(&values[DET_NUM_CLASSES + (2U * DET_REG_BINS)]) * stride;
                float const bottom = det_nanodet_distance(&values[DET_NUM_CLASSES + (3U * DET_REG_BINS)]) * stride;

                detection_box_t box;
                box.x1 = det_clampf(center_x - left, 0.0f, (float) DET_INPUT_SIZE);
                box.y1 = det_clampf(center_y - top, 0.0f, (float) DET_INPUT_SIZE);
                box.x2 = det_clampf(center_x + right, 0.0f, (float) DET_INPUT_SIZE);
                box.y2 = det_clampf(center_y + bottom, 0.0f, (float) DET_INPUT_SIZE);
                box.score = score;
                box.class_id = class_id;

                if ((box.x2 > box.x1) && (box.y2 > box.y1))
                {
                    det_insert_candidate(&box, p_candidate_count);
                }
            }
        }

        point_offset += grid * grid;
    }
}

static float det_iou(detection_box_t const * a, detection_box_t const * b)
{
    float const ix1 = det_maxf(a->x1, b->x1);
    float const iy1 = det_maxf(a->y1, b->y1);
    float const ix2 = det_minf(a->x2, b->x2);
    float const iy2 = det_minf(a->y2, b->y2);
    float const iw = det_maxf(0.0f, ix2 - ix1);
    float const ih = det_maxf(0.0f, iy2 - iy1);
    float const intersection = iw * ih;
    float const area_a = det_maxf(0.0f, a->x2 - a->x1) * det_maxf(0.0f, a->y2 - a->y1);
    float const area_b = det_maxf(0.0f, b->x2 - b->x1) * det_maxf(0.0f, b->y2 - b->y1);
    float const denom = area_a + area_b - intersection;

    return (denom > 0.0f) ? (intersection / denom) : 0.0f;
}

static uint32_t det_nms(uint32_t candidate_count)
{
    uint32_t output_count = 0U;

    for (uint32_t i = 0U; (i < candidate_count) && (output_count < DET_MAX_OUTPUTS); i++)
    {
        bool keep = true;

        for (uint32_t j = 0U; j < output_count; j++)
        {
            if ((g_candidates[i].class_id == g_outputs[j].class_id) &&
                (det_iou(&g_candidates[i], &g_outputs[j]) > DET_NMS_IOU_THRESHOLD))
            {
                keep = false;
                break;
            }
        }

        if (keep)
        {
            g_outputs[output_count] = g_candidates[i];
            output_count++;
        }
    }

    return output_count;
}

static void det_to_camera_coords(detection_box_t const * p_model_box,
                                 int32_t * p_x1,
                                 int32_t * p_y1,
                                 int32_t * p_x2,
                                 int32_t * p_y2)
{
    float const crop_size = (float) CAMERA_OV5640_HEIGHT;
    float const crop_x = ((float) CAMERA_OV5640_WIDTH - crop_size) * 0.5f;
    float const scale = crop_size / (float) DET_INPUT_SIZE;

    *p_x1 = (int32_t) ((p_model_box->x1 * scale) + crop_x);
    *p_y1 = (int32_t) (p_model_box->y1 * scale);
    *p_x2 = (int32_t) ((p_model_box->x2 * scale) + crop_x);
    *p_y2 = (int32_t) (p_model_box->y2 * scale);
}

static int32_t det_clampi32(int32_t value, int32_t low, int32_t high)
{
    return (value < low) ? low : ((value > high) ? high : value);
}

static void det_enforce_min_interval(int32_t * p_low,
                                     int32_t * p_high,
                                     int32_t   bound_low,
                                     int32_t   bound_high)
{
    if ((*p_high - *p_low) >= DET_MIN_BOX_SIDE_PX)
    {
        return;
    }

    int32_t const center = (*p_low + *p_high) / 2;
    int32_t low = center - (DET_MIN_BOX_SIDE_PX / 2);
    int32_t high = low + DET_MIN_BOX_SIDE_PX;

    if (low < bound_low)
    {
        low = bound_low;
        high = bound_low + DET_MIN_BOX_SIDE_PX;
    }
    if (high > bound_high)
    {
        high = bound_high;
        low = bound_high - DET_MIN_BOX_SIDE_PX;
    }

    *p_low = low;
    *p_high = high;
}

static void __attribute__((unused)) det_enforce_min_box_area(int32_t * p_x1,
                                                             int32_t * p_y1,
                                                             int32_t * p_x2,
                                                             int32_t * p_y2)
{
    int32_t const crop_x1 = ((int32_t) CAMERA_OV5640_WIDTH - (int32_t) CAMERA_OV5640_HEIGHT) / 2;
    int32_t const crop_x2 = crop_x1 + (int32_t) CAMERA_OV5640_HEIGHT;

    det_enforce_min_interval(p_x1, p_x2, crop_x1, crop_x2);
    det_enforce_min_interval(p_y1, p_y2, 0, (int32_t) CAMERA_OV5640_HEIGHT);
}

static uint32_t det_isqrt_u64(uint64_t value)
{
    uint64_t result = 0U;
    uint64_t bit = (uint64_t) 1U << 62U;

    while (bit > value)
    {
        bit >>= 2U;
    }

    while (0U != bit)
    {
        if (value >= (result + bit))
        {
            value -= result + bit;
            result = (result >> 1U) + bit;
        }
        else
        {
            result >>= 1U;
        }
        bit >>= 2U;
    }

    return (uint32_t) result;
}

static int32_t det_pixel_pigment(uint32_t r, uint32_t g, uint32_t b)
{
    uint32_t const sum = r + g + b;
    uint32_t const max_rgb = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);

    if ((r >= 64U) && (r >= (g + 24U)) && (r >= (b + 18U)) &&
        ((r * 100U) >= (sum * 42U)))
    {
        return (int32_t) DET_PIGMENT_RED;
    }

    if ((g >= 48U) && (g >= (r + 12U)) && (g >= (b + 18U)) &&
        ((g * 100U) >= (sum * 38U)))
    {
        return (int32_t) DET_PIGMENT_GREEN;
    }

    if ((max_rgb <= 90U) && (sum >= 36U))
    {
        return (int32_t) DET_PIGMENT_DARK;
    }

    return -1;
}

static void det_pigment_cell_bounds(int32_t box_x1,
                                    int32_t box_y1,
                                    int32_t box_w,
                                    int32_t box_h,
                                    uint32_t cell,
                                    int32_t * p_x1,
                                    int32_t * p_y1,
                                    int32_t * p_x2,
                                    int32_t * p_y2)
{
    int32_t const col = (int32_t) (cell % 3U) - 1;
    int32_t const row = (int32_t) (cell / 3U) - 1;
    int32_t const crop_x1 = ((int32_t) CAMERA_OV5640_WIDTH - (int32_t) CAMERA_OV5640_HEIGHT) / 2;
    int32_t const crop_x2 = crop_x1 + (int32_t) CAMERA_OV5640_HEIGHT;

    *p_x1 = det_clampi32(box_x1 + (col * box_w), crop_x1, crop_x2);
    *p_y1 = det_clampi32(box_y1 + (row * box_h), 0, (int32_t) CAMERA_OV5640_HEIGHT);
    *p_x2 = det_clampi32(box_x1 + ((col + 1) * box_w), crop_x1, crop_x2);
    *p_y2 = det_clampi32(box_y1 + ((row + 1) * box_h), 0, (int32_t) CAMERA_OV5640_HEIGHT);
}

static void det_add_pigment_pixel(pigment_stats_t * p_stats,
                                  uint32_t x,
                                  uint32_t y,
                                  uint32_t r,
                                  uint32_t g,
                                  uint32_t b)
{
    p_stats->count++;
    p_stats->sum_x += x;
    p_stats->sum_y += y;
    p_stats->sum_x2 += (uint64_t) x * x;
    p_stats->sum_y2 += (uint64_t) y * y;
    p_stats->sum_r += r;
    p_stats->sum_g += g;
    p_stats->sum_b += b;
}

static bool det_refine_by_pigment(uint8_t const * p_rgb565_frame,
                                  uint32_t original_class,
                                  int32_t * p_x1,
                                  int32_t * p_y1,
                                  int32_t * p_x2,
                                  int32_t * p_y2,
                                  uint32_t * p_class_id,
                                  int32_t * p_mean_r,
                                  int32_t * p_mean_g,
                                  int32_t * p_mean_b,
                                  int32_t * p_green_ratio_0p1)
{
    int32_t const box_w = *p_x2 - *p_x1;
    int32_t const box_h = *p_y2 - *p_y1;
    uint32_t best_score = 0U;
    uint32_t best_pigment = 0U;
    uint32_t best_cell = 0U;

    *p_mean_r = -1;
    *p_mean_g = -1;
    *p_mean_b = -1;
    *p_green_ratio_0p1 = -1;
    *p_class_id = original_class;

    if ((box_w < 3) || (box_h < 3))
    {
        return false;
    }

    for (uint32_t pigment = 0U; pigment < DET_PIGMENT_TYPE_COUNT; pigment++)
    {
        for (uint32_t cell = 0U; cell < DET_PIGMENT_CELL_COUNT; cell++)
        {
            g_pigment_stats[pigment][cell] = (pigment_stats_t) {0};
        }
    }

    for (uint32_t cell = 0U; cell < DET_PIGMENT_CELL_COUNT; cell++)
    {
        int32_t cell_x1;
        int32_t cell_y1;
        int32_t cell_x2;
        int32_t cell_y2;
        uint32_t const step = ((box_w < 40) || (box_h < 40)) ? 1U : 2U;

        g_pigment_cell_samples[cell] = 0U;
        det_pigment_cell_bounds(*p_x1, *p_y1, box_w, box_h, cell,
                                &cell_x1, &cell_y1, &cell_x2, &cell_y2);

        for (int32_t y = cell_y1; y < cell_y2; y += (int32_t) step)
        {
            for (int32_t x = cell_x1; x < cell_x2; x += (int32_t) step)
            {
                uint32_t const pixel_index = (((uint32_t) y * CAMERA_OV5640_WIDTH) + (uint32_t) x) *
                                             CAMERA_OV5640_BYTES_PER_PIXEL;
                uint16_t const pixel = (uint16_t) (p_rgb565_frame[pixel_index] |
                                                   ((uint16_t) p_rgb565_frame[pixel_index + 1U] << 8));
                uint32_t const r = det_rgb565_r(pixel);
                uint32_t const g = det_rgb565_g(pixel);
                uint32_t const b = det_rgb565_b(pixel);
                int32_t const pigment = det_pixel_pigment(r, g, b);

                g_pigment_cell_samples[cell]++;
                if (pigment >= 0)
                {
                    det_add_pigment_pixel(&g_pigment_stats[(uint32_t) pigment][cell],
                                          (uint32_t) x, (uint32_t) y, r, g, b);
                }
            }
        }
    }

    for (uint32_t pigment = 0U; pigment < DET_PIGMENT_TYPE_COUNT; pigment++)
    {
        for (uint32_t cell = 0U; cell < DET_PIGMENT_CELL_COUNT; cell++)
        {
            pigment_stats_t const * stats = &g_pigment_stats[pigment][cell];
            uint32_t const samples = g_pigment_cell_samples[cell];

            if ((stats->count < DET_PIGMENT_MIN_PIXELS) || (0U == samples))
            {
                continue;
            }

            uint32_t const density = (stats->count * 1000U) / samples;
            uint32_t const col_distance = (cell % 3U == 1U) ? 0U : 1U;
            uint32_t const row_distance = (cell / 3U == 1U) ? 0U : 1U;
            uint32_t const position_weight = 120U - ((col_distance + row_distance) * 20U);
            uint32_t score;

            if (((pigment != DET_PIGMENT_DARK) && (density < DET_CHROMA_MIN_PERMILLE)) ||
                ((pigment == DET_PIGMENT_DARK) && (density < DET_DARK_MIN_PERMILLE)))
            {
                continue;
            }

            if (pigment == DET_PIGMENT_GREEN)
            {
                uint64_t const rgb_sum = stats->sum_r + stats->sum_g + stats->sum_b;
                if ((0U == rgb_sum) ||
                    ((stats->sum_g * 100U) < (rgb_sum * g_green_ratio_threshold)))
                {
                    continue;
                }
            }

            score = density * position_weight;
            if (pigment == DET_PIGMENT_DARK)
            {
                score = (score * 70U) / 100U;
            }

            uint32_t const pigment_class = (pigment == DET_PIGMENT_RED) ? APP_DETECTION_CLASS_TOMATO :
                                            (pigment == DET_PIGMENT_GREEN) ? APP_DETECTION_CLASS_GREEN_GRAPE :
                                                                            APP_DETECTION_CLASS_PURPLE_GRAPE;
            if (pigment_class == original_class)
            {
                score = (score * 110U) / 100U;
            }

            if (score > best_score)
            {
                best_score = score;
                best_pigment = pigment;
                best_cell = cell;
            }
        }
    }

    if (0U == best_score)
    {
        return false;
    }

    pigment_stats_t const * best = &g_pigment_stats[best_pigment][best_cell];
    uint64_t const count = best->count;
    uint32_t const mean_x = (uint32_t) ((best->sum_x + (count / 2U)) / count);
    uint32_t const mean_y = (uint32_t) ((best->sum_y + (count / 2U)) / count);
    uint64_t const mean_x2 = (uint64_t) mean_x * mean_x;
    uint64_t const mean_y2 = (uint64_t) mean_y * mean_y;
    uint64_t const avg_x2 = best->sum_x2 / count;
    uint64_t const avg_y2 = best->sum_y2 / count;
    uint32_t half_w = det_isqrt_u64((avg_x2 > mean_x2) ? (avg_x2 - mean_x2) : 0U);
    uint32_t half_h = det_isqrt_u64((avg_y2 > mean_y2) ? (avg_y2 - mean_y2) : 0U);
    int32_t cell_x1;
    int32_t cell_y1;
    int32_t cell_x2;
    int32_t cell_y2;

    half_w = (half_w < 2U) ? 2U : half_w;
    half_h = (half_h < 2U) ? 2U : half_h;
    half_w = ((half_w * DET_REFINED_BOX_SCALE_NUM) + DET_REFINED_BOX_SCALE_DEN - 1U) /
             DET_REFINED_BOX_SCALE_DEN;
    half_h = ((half_h * DET_REFINED_BOX_SCALE_NUM) + DET_REFINED_BOX_SCALE_DEN - 1U) /
             DET_REFINED_BOX_SCALE_DEN;
    det_pigment_cell_bounds(*p_x1, *p_y1, box_w, box_h, best_cell,
                            &cell_x1, &cell_y1, &cell_x2, &cell_y2);
    *p_x1 = det_clampi32((int32_t) mean_x - (int32_t) half_w, cell_x1, cell_x2 - 1);
    *p_y1 = det_clampi32((int32_t) mean_y - (int32_t) half_h, cell_y1, cell_y2 - 1);
    *p_x2 = det_clampi32((int32_t) mean_x + (int32_t) half_w + 1, *p_x1 + 1, cell_x2);
    *p_y2 = det_clampi32((int32_t) mean_y + (int32_t) half_h + 1, *p_y1 + 1, cell_y2);

    *p_class_id = (best_pigment == DET_PIGMENT_RED) ? APP_DETECTION_CLASS_TOMATO :
                  (best_pigment == DET_PIGMENT_GREEN) ? APP_DETECTION_CLASS_GREEN_GRAPE :
                                                        APP_DETECTION_CLASS_PURPLE_GRAPE;
    *p_mean_r = (int32_t) ((best->sum_r + (count / 2U)) / count);
    *p_mean_g = (int32_t) ((best->sum_g + (count / 2U)) / count);
    *p_mean_b = (int32_t) ((best->sum_b + (count / 2U)) / count);

    uint64_t const rgb_sum = best->sum_r + best->sum_g + best->sum_b;
    *p_green_ratio_0p1 = (0U == rgb_sum) ? 0 :
                            (int32_t) (((best->sum_g * 1000U) + (rgb_sum / 2U)) / rgb_sum);
    return true;
}

static void det_correct_center_by_pigment(uint8_t const          * p_rgb565_frame,
                                          detection_box_t const  * p_model_box,
                                          int32_t                * p_x1,
                                          int32_t                * p_y1,
                                          int32_t                * p_x2,
                                          int32_t                * p_y2,
                                          app_detection_result_t * p_result)
{
    int32_t refined_x1 = *p_x1;
    int32_t refined_y1 = *p_y1;
    int32_t refined_x2 = *p_x2;
    int32_t refined_y2 = *p_y2;
    uint32_t refined_class = p_model_box->class_id;

    bool const refined = det_refine_by_pigment(p_rgb565_frame,
                                                p_model_box->class_id,
                                                &refined_x1,
                                                &refined_y1,
                                                &refined_x2,
                                                &refined_y2,
                                                &refined_class,
                                                &p_result->mean_r,
                                                &p_result->mean_g,
                                                &p_result->mean_b,
                                                &p_result->green_ratio_0p1);

    /* Pigment is only a bounded center hint.  It never replaces the model
     * class or size, which keeps isolated colour noise from moving the box. */
    if (!refined || (refined_class != p_model_box->class_id))
    {
        p_result->mean_r = -1;
        p_result->mean_g = -1;
        p_result->mean_b = -1;
        p_result->green_ratio_0p1 = -1;
        return;
    }

    int32_t const width = *p_x2 - *p_x1;
    int32_t const height = *p_y2 - *p_y1;
    int32_t const model_center_x = (*p_x1 + *p_x2) / 2;
    int32_t const model_center_y = (*p_y1 + *p_y2) / 2;
    int32_t const refined_center_x = (refined_x1 + refined_x2) / 2;
    int32_t const refined_center_y = (refined_y1 + refined_y2) / 2;
    int32_t const max_dx = ((width / DET_PIGMENT_CENTER_LIMIT_DIVISOR) > 2) ?
                           (width / DET_PIGMENT_CENTER_LIMIT_DIVISOR) : 2;
    int32_t const max_dy = ((height / DET_PIGMENT_CENTER_LIMIT_DIVISOR) > 2) ?
                           (height / DET_PIGMENT_CENTER_LIMIT_DIVISOR) : 2;
    int32_t const dx = det_clampi32(refined_center_x - model_center_x, -max_dx, max_dx);
    int32_t const dy = det_clampi32(refined_center_y - model_center_y, -max_dy, max_dy);
    int32_t const crop_x1 = ((int32_t) CAMERA_OV5640_WIDTH - (int32_t) CAMERA_OV5640_HEIGHT) / 2;
    int32_t const crop_x2 = crop_x1 + (int32_t) CAMERA_OV5640_HEIGHT;

    *p_x1 = det_clampi32(*p_x1 + dx, crop_x1, crop_x2 - width);
    *p_y1 = det_clampi32(*p_y1 + dy, 0, (int32_t) CAMERA_OV5640_HEIGHT - height);
    *p_x2 = *p_x1 + width;
    *p_y2 = *p_y1 + height;
}

bool app_detection_init(void)
{
    app_detection_settings_init();
    if (!g_performance_cycle_counter_ready)
    {
        performance_cycle_counter_init();
    }
    fsp_err_t const err = g_rm_ethosu0.p_api->open(g_rm_ethosu0.p_ctrl, g_rm_ethosu0.p_cfg);
    g_detection_initialized = (FSP_SUCCESS == err) || (FSP_ERR_ALREADY_OPEN == err);

    return g_detection_initialized;
}

bool app_detection_run_frame(uint8_t const                 * p_rgb565_frame,
                             app_detection_result_t        * p_results,
                             uint32_t                        result_capacity,
                             uint32_t                      * p_result_count,
                             app_detection_text_writer_t     write_text)
{
    if ((NULL == p_rgb565_frame) || (NULL == p_results) || (NULL == p_result_count))
    {
        return false;
    }

    *p_result_count = 0U;

    if (!g_detection_initialized && !app_detection_init())
    {
        if (NULL != write_text)
        {
            write_text("DET_ERR npu_open\r\n");
        }

        return false;
    }

    g_performance_frame_start_tick = xTaskGetTickCount();
    det_preprocess_rgb565(p_rgb565_frame);
    uint32_t const inference_start_cycles = DWT->CYCCNT;
    TickType_t const inference_start_tick = xTaskGetTickCount();
    if (!RunModel())
    {
        if (NULL != write_text)
        {
            write_text("DET_ERR model_invoke\r\n");
        }

        return false;
    }
    performance_record_frame(inference_start_cycles, inference_start_tick);

    uint32_t candidate_count = 0U;
    det_decode_nanodet(GetModelOutputPtr_output_70634(), &candidate_count);

    uint32_t const output_count = det_nms(candidate_count);
    uint32_t const result_count = (output_count < result_capacity) ? output_count : result_capacity;

    for (uint32_t i = 0U; i < result_count; i++)
    {
        int32_t x1;
        int32_t y1;
        int32_t x2;
        int32_t y2;
        det_to_camera_coords(&g_outputs[i], &x1, &y1, &x2, &y2);

        p_results[i].mean_r = -1;
        p_results[i].mean_g = -1;
        p_results[i].mean_b = -1;
        p_results[i].green_ratio_0p1 = -1;
        det_correct_center_by_pigment(p_rgb565_frame,
                                      &g_outputs[i],
                                      &x1,
                                      &y1,
                                      &x2,
                                      &y2,
                                      &p_results[i]);

        /* NanoDet remains authoritative for class and box size. */
        p_results[i].class_id = g_outputs[i].class_id;
        p_results[i].x        = (x1 + x2) / 2;
        p_results[i].y        = (y1 + y2) / 2;
        p_results[i].x1       = x1;
        p_results[i].y1       = y1;
        p_results[i].x2       = x2;
        p_results[i].y2       = y2;

        if (NULL != write_text)
        {
            int const line_count = snprintf(g_det_line,
                                            sizeof(g_det_line),
                                            "DET class=%lu x=%ld y=%ld\r\n",
                                            (unsigned long) p_results[i].class_id,
                                            (long) p_results[i].x,
                                            (long) p_results[i].y);

            if ((line_count > 0) && ((size_t) line_count < sizeof(g_det_line)))
            {
                write_text(g_det_line);
            }
        }
    }

    *p_result_count = result_count;

    return true;
}

void app_detection_get_performance(app_performance_metrics_t * p_metrics)
{
    if (NULL == p_metrics)
    {
        return;
    }

    taskENTER_CRITICAL();
    p_metrics->inference_0p1ms = g_performance_inference_0p1ms;
    p_metrics->fps_0p1 = g_performance_fps_0p1;
    p_metrics->vision_to_execution_0p1ms =
        g_performance_vision_to_execution_0p1ms;
    p_metrics->ipc_latency_0p1ms = g_performance_ipc_latency_0p1ms;
    p_metrics->cpu1_ram_bytes = g_performance_cpu1_ram_bytes;
    p_metrics->cpu1_flash_bytes = g_performance_cpu1_flash_bytes;
    p_metrics->cpu1_sdram_bytes = g_performance_cpu1_sdram_bytes;
    p_metrics->inference_valid = g_performance_inference_valid;
    p_metrics->fps_valid = g_performance_fps_valid;
    p_metrics->vision_to_execution_valid =
        g_performance_vision_to_execution_valid;
    p_metrics->ipc_latency_valid = g_performance_ipc_latency_valid;
    p_metrics->cpu1_memory_valid = g_performance_cpu1_memory_valid;
    taskEXIT_CRITICAL();
}

void app_detection_performance_set_ipc_latency(uint32_t latency_0p1ms)
{
    g_performance_ipc_latency_0p1ms = latency_0p1ms;
    g_performance_ipc_latency_valid = true;
}

void app_detection_performance_track_execution(uint32_t batch_id)
{
    taskENTER_CRITICAL();
    g_performance_execution_origin_tick = g_performance_frame_start_tick;
    g_performance_execution_batch_id = batch_id;
    g_performance_execution_pending = true;
    taskEXIT_CRITICAL();
}

void app_detection_performance_finish_execution(uint32_t batch_id)
{
    if (g_performance_execution_pending &&
        ((g_performance_execution_batch_id & 0xFFFFU) ==
         (batch_id & 0xFFFFU)))
    {
        TickType_t const elapsed =
            xTaskGetTickCountFromISR() - g_performance_execution_origin_tick;
        g_performance_vision_to_execution_0p1ms =
            (uint32_t) pdTICKS_TO_MS(elapsed) * 10U;
        g_performance_vision_to_execution_valid = true;
        g_performance_execution_pending = false;
    }
}

void app_detection_performance_set_cpu1_memory(uint32_t message_prefix,
                                               uint32_t bytes)
{
    if (IPC_PERFORMANCE_RAM_PREFIX == message_prefix)
    {
        g_performance_cpu1_ram_bytes = bytes;
    }
    else if (IPC_PERFORMANCE_FLASH_PREFIX == message_prefix)
    {
        g_performance_cpu1_flash_bytes = bytes;
    }
    else if (IPC_PERFORMANCE_SDRAM_PREFIX == message_prefix)
    {
        g_performance_cpu1_sdram_bytes = bytes;
        g_performance_cpu1_memory_valid = true;
    }
}
