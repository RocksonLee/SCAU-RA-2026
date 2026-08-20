#include "camera_stream.h"

#include <stdbool.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "app_detection.h"
#include "camera_ov5640.h"
#include "fruit_ui.h"
#include "hal_data.h"
#include "ipc_detection_tx.h"
#include "ospi_flash.h"

#define CAMERA_UART_CHUNK_BYTES (1024U)
#define CAMERA_FRAME_HEADER_BYTES (24U)
#define CAMERA_FRAME_FORMAT_RGB565_LE (1U)
#define CAMERA_AE_SETTLE_MS     (1500U)
#define CAMERA_DISCARD_FRAMES   (5U)
#define CAMERA_UART_LINE_BYTES  (192U)
#define CAMERA_CAPTURE_ATTEMPTS (3U)
#define CAMERA_CAPTURE_RETRY_MS (100U)
#define CAMERA_MUX_SETTLE_MS     (5U)
#define CAMERA_SWITCH_SETTLE_MS  (1000U)
#define CAMERA_SIDE_MUX_SETTLE_MS (50U)
#define CAMERA_SIDE_SWITCH_SETTLE_MS (2500U)
#define CAMERA_FAST_SWITCH_SETTLE_MS (250U)
#define CAMERA_FAST_PRIME_ATTEMPTS (3U)
#define CAMERA_CAPTURE_RECOVERY_THRESHOLD (2U)
#define CAMERA_SIDE_READY_ATTEMPTS (20U)
#define CAMERA_SIDE_READY_FRAMES   (3U)
#define CAMERA_SIDE_READY_GAP_MS   (100U)
#define CAMERA_SIDE_VISIBLE_LEVEL  (8U)
#define CAMERA_LED_WRITE_ATTEMPTS  (3U)
#define CAMERA_LED_RETRY_MS        (20U)
#define CAMERA_DEBUG_LIGHT_SETTLE_MS (400U)
#define CAMERA_DEBUG_LIGHT_DISCARD_FRAMES (3U)
#define CAMERA_TASK_PREVIEW_PERIOD_MS (150U)
#define CAMERA_TASK_PREVIEW_MIN_MS (1000U)
#define CAMERA_BOX_FILTER_SAMPLES (3U)
#define CAMERA_BOX_PADDING_DIVISOR (8)
#define CAMERA_BOX_VIEW_COUNT     (2U)
#define CAMERA_TOP_SAMPLES       (5U)
#define CAMERA_PURPLE_TOP_SAMPLES (3U)
#define CAMERA_PURPLE_TOP_WINDOW_FRAMES (10U)
#define CAMERA_PURPLE_TOP_MAX_DEVIATION_PX (35)
#define CAMERA_TOP_BATCH_MIN_FRAMES (5U)
#define CAMERA_TOP_BATCH_MAX_FRAMES (10U)
#define CAMERA_SIDE_SAMPLES      (5U)
#define CAMERA_SIDE_SAMPLE_MAX_DEVIATION_PX (60)
#define CAMERA_SIDE_MAX_FRAMES   (90U)
#define CAMERA_SIDE_Y_MIN_PX     (50)
#define CAMERA_SIDE_Z_SLOPE_DEFAULT  (0.39999866495781267)
#define CAMERA_SIDE_Z_OFFSET_DEFAULT (278.64030625867775)
#define CAMERA_TOP_ONLY_Z_MM     (100.0)
#define CAMERA_TOP_CAL_Z_LOW_MM  (325.0)
#define CAMERA_TOP_CAL_Z_HIGH_MM (385.0)
#define CAMERA_HOMOGRAPHY_EPSILON (1.0e-9)
#define CAMERA_CALIBRATION_MAGIC (0x4843414CU) /* "HCAL" */
#define CAMERA_CALIBRATION_VERSION (2U)
#define CAMERA_CALIBRATION_SLOT_A (0x00013000UL)
#define CAMERA_CALIBRATION_SLOT_B (0x00014000UL)
#define CAMERA_CALIBRATION_TOP_POINTS_PER_LAYER (9U)
#define CAMERA_CALIBRATION_TOP_POINTS (18U)
#define CAMERA_CALIBRATION_POINT_COUNT (18U)
#define CAMERA_CALIBRATION_OBSERVATIONS (9U)
#define CAMERA_CALIBRATION_MIN_INLIERS (5U)
#define CAMERA_CALIBRATION_MAX_DEVIATION_PX (15)
#define CAMERA_CALIBRATION_MAX_FRAMES (90U)
#define CAMERA_CALIBRATION_ARM_TIMEOUT_MS (10000U)
#define CAMERA_CALIBRATION_TOP_MAX_RMSE_MM (12.0)

/* Keep these coefficients synchronized with CPU1 handeye_transform.c. */
static double const g_camera_to_arm_homography_z0[3][3] =
{
    {0.031114,  0.800983,  -64.142100},
    {0.954234, -0.013392, -101.298640},
    {0.000247, -0.000043,    1.000000},
};

static double g_camera_to_arm_homography_z_low[3][3] =
{
    {-0.0050620485,  0.2640889468, 38.1704684559},
    { 0.3836043009, -0.0242952309, 75.9593991709},
    { 0.0002450191, -0.0002921146,  1.0000000000},
};

static double g_camera_to_arm_homography_z_high[3][3] =
{
    {0.1914500083, 0.4343906439, 62.3737818695},
    {0.9075307303, 0.2533455801, 42.1344710727},
    {0.0025644552, 0.0008938283,  1.0000000000},
};

static double g_camera_top_cal_z_low_mm = CAMERA_TOP_CAL_Z_LOW_MM;
static double g_camera_top_cal_z_high_mm = CAMERA_TOP_CAL_Z_HIGH_MM;
static double g_camera_side_z_slope_mm_px = CAMERA_SIDE_Z_SLOPE_DEFAULT;
static double g_camera_side_z_offset_mm = CAMERA_SIDE_Z_OFFSET_DEFAULT;

typedef struct st_camera_calibration_target
{
    int32_t x_0p1mm;
    int32_t y_0p1mm;
    int32_t z_0p1mm;
    uint8_t camera;
    uint8_t layer;
    uint16_t reserved;
} camera_calibration_target_t;

typedef struct st_camera_calibration_sample
{
    int32_t pixel_x;
    int32_t pixel_y;
    int32_t arm_x_0p1mm;
    int32_t arm_y_0p1mm;
    int32_t arm_z_0p1mm;
    uint8_t camera;
    uint8_t layer;
    uint16_t reserved;
} camera_calibration_sample_t;

typedef struct st_camera_calibration_record
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t sequence;
    uint32_t sample_count;
    ipc_handeye_calibration_t config;
    camera_calibration_sample_t samples[CAMERA_CALIBRATION_POINT_COUNT];
    float top_low_rmse_mm;
    float top_high_rmse_mm;
    float side_rmse_mm;
    uint32_t crc32;
} camera_calibration_record_t;

_Static_assert(sizeof(camera_calibration_record_t) <= OSPI_FLASH_SECTOR_SIZE,
               "Calibration record must fit in one OSPI sector.");

typedef enum e_camera_calibration_state
{
    CAMERA_CALIBRATION_IDLE = 0,
    CAMERA_CALIBRATION_SEND_MOVE,
    CAMERA_CALIBRATION_WAIT_ARM,
    CAMERA_CALIBRATION_WAIT_CONFIRM,
    CAMERA_CALIBRATION_COLLECT,
} camera_calibration_state_t;

static camera_calibration_target_t const g_camera_calibration_targets[CAMERA_CALIBRATION_POINT_COUNT] =
{
    /* Z=325 mm: selected from and inside the previous calibrated workspace. */
    {IPC_CALIBRATION_FIRST_X_0P1MM,
     IPC_CALIBRATION_FIRST_Y_0P1MM,
     IPC_CALIBRATION_FIRST_Z_0P1MM, 0U, 0U, 0U},
    { 950, 2450, 3250, 0U, 0U, 0U},
    { 550, 2350, 3250, 0U, 0U, 0U},
    {1550, 2030, 3250, 0U, 0U, 0U},
    {1400, 1900, 3250, 0U, 0U, 0U},
    { 600, 2300, 3250, 0U, 0U, 0U},
    {1200, 2300, 3250, 0U, 0U, 0U},
    {1000, 2200, 3250, 0U, 0U, 0U},
    { 750, 2250, 3250, 0U, 0U, 0U},
    /* Z=385 mm. */
    {1150, 2230, 3850, 0U, 1U, 0U},
    { 830, 2230, 3850, 0U, 1U, 0U},
    {1190, 2020, 3850, 0U, 1U, 0U},
    {1510, 1700, 3850, 0U, 1U, 0U},
    {1100, 2000, 3850, 0U, 1U, 0U},
    {1400, 1900, 3850, 0U, 1U, 0U},
    { 900, 2200, 3850, 0U, 1U, 0U},
    {1300, 2100, 3850, 0U, 1U, 0U},
    {1000, 2100, 3850, 0U, 1U, 0U},
};

static camera_calibration_sample_t g_camera_calibration_samples[CAMERA_CALIBRATION_POINT_COUNT];
static camera_calibration_state_t g_camera_calibration_state;
static bool g_camera_calibration_active;
static uint32_t g_camera_calibration_index;
static uint32_t g_camera_calibration_sequence;
static TickType_t g_camera_calibration_deadline;
static uint32_t g_camera_calibration_frame_count;
static int32_t g_camera_calibration_observation_x[CAMERA_CALIBRATION_OBSERVATIONS];
static int32_t g_camera_calibration_observation_y[CAMERA_CALIBRATION_OBSERVATIONS];
static uint32_t g_camera_calibration_observation_count;
static int32_t g_camera_calibration_arm_x_0p1mm;
static int32_t g_camera_calibration_arm_y_0p1mm;
static int32_t g_camera_calibration_arm_z_0p1mm;
static uint32_t g_camera_calibration_flash_sequence;
static volatile bool g_camera_calibration_arm_result_pending;
static volatile uint32_t g_camera_calibration_arm_result_sequence;
static volatile bool g_camera_calibration_arm_result_success;
static volatile int32_t g_camera_calibration_arm_result_x_0p1mm;
static volatile int32_t g_camera_calibration_arm_result_y_0p1mm;
static volatile int32_t g_camera_calibration_arm_result_z_0p1mm;

#if APP_DETECTION_MAX_RESULTS != FRUIT_UI_MAX_DETECTIONS
 #error "Detection result capacity does not match the UI capacity."
#endif

static uint8_t g_camera_frame[CAMERA_OV5640_FRAME_BYTES] BSP_PLACE_IN_SECTION(".sdram_nocache") BSP_ALIGN_VARIABLE(32);

static volatile bool g_uart_tx_busy;
static uart_callback_args_t g_uart_callback_memory;
static char g_uart_line[CAMERA_UART_LINE_BYTES];
static app_detection_result_t g_detection_results[APP_DETECTION_MAX_RESULTS];
static bool g_camera_illumination_enabled;
static bool g_camera_illumination_state_valid;
static bool g_camera_initialized[2];

typedef struct st_camera_top_samples
{
    int32_t  x[CAMERA_TOP_SAMPLES];
    int32_t  y[CAMERA_TOP_SAMPLES];
    uint32_t frame_id[CAMERA_TOP_SAMPLES];
    uint32_t count;
    uint32_t next;
} camera_top_samples_t;

typedef struct st_camera_box_filter
{
    int32_t  x1[CAMERA_BOX_FILTER_SAMPLES];
    int32_t  y1[CAMERA_BOX_FILTER_SAMPLES];
    int32_t  x2[CAMERA_BOX_FILTER_SAMPLES];
    int32_t  y2[CAMERA_BOX_FILTER_SAMPLES];
    uint32_t count;
    uint32_t next;
    uint32_t class_id;
    uint32_t missed_frames;
    bool     valid;
} camera_box_filter_t;

static camera_top_samples_t g_camera_top_samples[APP_DETECTION_MAX_RESULTS];
static uint32_t g_camera_top_frame_id;
static camera_box_filter_t g_camera_box_filters[CAMERA_BOX_VIEW_COUNT][APP_DETECTION_MAX_RESULTS];

static bool camera_pair_to_ui_detection(ipc_camera_coordinate_pair_t const * p_pair,
                                         fruit_ui_detection_t                * p_detection);
static bool camera_frame_has_visible_content(uint8_t const * p_rgb565_frame);
static bool camera_set_illumination(bool enabled);
static bool camera_project_top(double const homography[3][3],
                               double u,
                               double v,
                               double * p_x_mm,
                               double * p_y_mm);

static fruit_ui_target_t camera_stream_target_from_class(uint32_t class_id)
{
    switch (class_id)
    {
        case APP_DETECTION_CLASS_TOMATO:
            return FRUIT_UI_TARGET_TOMATO;

        case APP_DETECTION_CLASS_GREEN_GRAPE:
            return FRUIT_UI_TARGET_GREEN_GRAPE;

        case APP_DETECTION_CLASS_PURPLE_GRAPE:
            return FRUIT_UI_TARGET_PURPLE_GRAPE;

        default:
            return FRUIT_UI_TARGET_NONE;
    }
}

static void camera_uart_callback(uart_callback_args_t * p_args)
{
    if ((NULL != p_args) && (UART_EVENT_TX_COMPLETE == p_args->event))
    {
        g_uart_tx_busy = false;
    }
}

static bool camera_uart_write(uint8_t const * p_data, uint32_t bytes)
{
    TickType_t const start = xTaskGetTickCount();

    while (1)
    {
        bool acquired = false;

        taskENTER_CRITICAL();
        if (!g_uart_tx_busy)
        {
            g_uart_tx_busy = true;
            acquired = true;
        }
        taskEXIT_CRITICAL();

        if (acquired)
        {
            break;
        }

        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(1000U))
        {
            return false;
        }

        vTaskDelay(1);
    }

    if (FSP_SUCCESS != g_uart9.p_api->write(g_uart9.p_ctrl, p_data, bytes))
    {
        g_uart_tx_busy = false;
        return false;
    }

    while (g_uart_tx_busy)
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(1000U))
        {
            g_uart_tx_busy = false;
            return false;
        }

        vTaskDelay(1);
    }

    return true;
}

static bool camera_uart_send_bytes(uint8_t const * p_data, uint32_t bytes)
{
    uint32_t sent = 0U;

    while (sent < bytes)
    {
        uint32_t const remaining = bytes - sent;
        uint32_t const chunk = (remaining > CAMERA_UART_CHUNK_BYTES) ? CAMERA_UART_CHUNK_BYTES : remaining;

        if (!camera_uart_write(&p_data[sent], chunk))
        {
            return false;
        }

        sent += chunk;
    }

    return true;
}

bool camera_debug_send_text(char const * p_text)
{
    if (NULL == p_text)
    {
        return false;
    }

    return camera_uart_write((uint8_t const *) p_text, (uint32_t) strlen(p_text));
}

bool camera_debug_uart_init(void)
{
    fsp_err_t const err = g_uart9.p_api->open(g_uart9.p_ctrl, g_uart9.p_cfg);

    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        return false;
    }

    return (FSP_SUCCESS == g_uart9.p_api->callbackSet(g_uart9.p_ctrl,
                                                       camera_uart_callback,
                                                       NULL,
                                                       &g_uart_callback_memory));
}

static void camera_uart_send_text(char const * p_text)
{
    (void) camera_debug_send_text(p_text);
}

static uint32_t camera_crc32(uint8_t const * p_data, uint32_t bytes)
{
    uint32_t crc = 0xFFFFFFFFU;

    for (uint32_t i = 0U; i < bytes; i++)
    {
        crc ^= p_data[i];
        for (uint32_t bit = 0U; bit < 8U; bit++)
        {
            uint32_t const mask = 0U - (crc & 1U);
            crc = (crc >> 1) ^ (0xEDB88320U & mask);
        }
    }

    return ~crc;
}

static bool camera_calibration_config_valid(ipc_handeye_calibration_t const * p_config)
{
    if ((NULL == p_config) ||
        !isfinite(p_config->top_z_low_mm) ||
        !isfinite(p_config->top_z_high_mm) ||
        !isfinite(p_config->side_z_slope_mm_px) ||
        !isfinite(p_config->side_z_offset_mm) ||
        !(p_config->top_z_high_mm > p_config->top_z_low_mm) ||
        !(p_config->side_z_slope_mm_px > 0.0f))
    {
        return false;
    }

    for (uint32_t row = 0U; row < 3U; row++)
    {
        for (uint32_t column = 0U; column < 3U; column++)
        {
            if (!isfinite(p_config->top_h_low[row][column]) ||
                !isfinite(p_config->top_h_high[row][column]))
            {
                return false;
            }
        }
    }
    return true;
}

static void camera_calibration_apply(ipc_handeye_calibration_t const * p_config)
{
    for (uint32_t row = 0U; row < 3U; row++)
    {
        for (uint32_t column = 0U; column < 3U; column++)
        {
            g_camera_to_arm_homography_z_low[row][column] =
                (double) p_config->top_h_low[row][column];
            g_camera_to_arm_homography_z_high[row][column] =
                (double) p_config->top_h_high[row][column];
        }
    }
    g_camera_top_cal_z_low_mm = (double) p_config->top_z_low_mm;
    g_camera_top_cal_z_high_mm = (double) p_config->top_z_high_mm;
    g_camera_side_z_slope_mm_px = (double) p_config->side_z_slope_mm_px;
    g_camera_side_z_offset_mm = (double) p_config->side_z_offset_mm;
}

static bool camera_calibration_record_valid(camera_calibration_record_t const * p_record)
{
    return (NULL != p_record) &&
           (CAMERA_CALIBRATION_MAGIC == p_record->magic) &&
           (CAMERA_CALIBRATION_VERSION == p_record->version) &&
           (sizeof(*p_record) == p_record->size) &&
           (CAMERA_CALIBRATION_POINT_COUNT == p_record->sample_count) &&
           camera_calibration_config_valid(&p_record->config) &&
           (p_record->crc32 ==
            camera_crc32((uint8_t const *) p_record,
                         (uint32_t) offsetof(camera_calibration_record_t, crc32)));
}

static bool camera_calibration_load(void)
{
    static camera_calibration_record_t slot_a;
    static camera_calibration_record_t slot_b;
    bool const valid_a = ospi_flash_read(CAMERA_CALIBRATION_SLOT_A,
                                         &slot_a,
                                         sizeof(slot_a)) &&
                         camera_calibration_record_valid(&slot_a);
    bool const valid_b = ospi_flash_read(CAMERA_CALIBRATION_SLOT_B,
                                         &slot_b,
                                         sizeof(slot_b)) &&
                         camera_calibration_record_valid(&slot_b);
    camera_calibration_record_t const * p_selected = NULL;

    if (valid_a && valid_b)
    {
        p_selected = ((int32_t) (slot_b.sequence - slot_a.sequence) > 0) ?
                     &slot_b : &slot_a;
    }
    else if (valid_a)
    {
        p_selected = &slot_a;
    }
    else if (valid_b)
    {
        p_selected = &slot_b;
    }

    if (NULL == p_selected)
    {
        return false;
    }

    camera_calibration_apply(&p_selected->config);
    g_camera_calibration_flash_sequence = p_selected->sequence;
    (void) ipc_detection_send_calibration_config(&p_selected->config);
    return true;
}

static bool camera_calibration_store(ipc_handeye_calibration_t const * p_config,
                                     double top_low_rmse_mm,
                                     double top_high_rmse_mm,
                                     double side_rmse_mm)
{
    static camera_calibration_record_t record;
    static camera_calibration_record_t verify;
    uint32_t const sequence = g_camera_calibration_flash_sequence + 1U;
    uint32_t const offset = (0U != (sequence & 1U)) ?
                            CAMERA_CALIBRATION_SLOT_A :
                            CAMERA_CALIBRATION_SLOT_B;

    memset(&record, 0, sizeof(record));
    record.magic = CAMERA_CALIBRATION_MAGIC;
    record.version = CAMERA_CALIBRATION_VERSION;
    record.size = (uint16_t) sizeof(record);
    record.sequence = sequence;
    record.sample_count = CAMERA_CALIBRATION_POINT_COUNT;
    record.config = *p_config;
    memcpy(record.samples,
           g_camera_calibration_samples,
           sizeof(g_camera_calibration_samples));
    record.top_low_rmse_mm = (float) top_low_rmse_mm;
    record.top_high_rmse_mm = (float) top_high_rmse_mm;
    record.side_rmse_mm = (float) side_rmse_mm;
    record.crc32 = camera_crc32((uint8_t const *) &record,
                                (uint32_t) offsetof(camera_calibration_record_t, crc32));

    if (!ospi_flash_is_ready() ||
        !ospi_flash_erase(offset, OSPI_FLASH_SECTOR_SIZE) ||
        !ospi_flash_write(offset, &record, sizeof(record)) ||
        !ospi_flash_read(offset, &verify, sizeof(verify)) ||
        !camera_calibration_record_valid(&verify) ||
        (verify.sequence != sequence))
    {
        return false;
    }

    g_camera_calibration_flash_sequence = sequence;
    return true;
}

static bool camera_solve_8x8(double matrix[8][9], double solution[8])
{
    for (uint32_t column = 0U; column < 8U; column++)
    {
        uint32_t pivot = column;
        double pivot_abs = fabs(matrix[pivot][column]);
        for (uint32_t row = column + 1U; row < 8U; row++)
        {
            double const value_abs = fabs(matrix[row][column]);
            if (value_abs > pivot_abs)
            {
                pivot = row;
                pivot_abs = value_abs;
            }
        }
        if (pivot_abs < 1.0e-10)
        {
            return false;
        }
        if (pivot != column)
        {
            for (uint32_t item = column; item < 9U; item++)
            {
                double const swap = matrix[column][item];
                matrix[column][item] = matrix[pivot][item];
                matrix[pivot][item] = swap;
            }
        }

        double const divisor = matrix[column][column];
        for (uint32_t item = column; item < 9U; item++)
        {
            matrix[column][item] /= divisor;
        }
        for (uint32_t row = 0U; row < 8U; row++)
        {
            if (row == column)
            {
                continue;
            }
            double const factor = matrix[row][column];
            for (uint32_t item = column; item < 9U; item++)
            {
                matrix[row][item] -= factor * matrix[column][item];
            }
        }
    }

    for (uint32_t i = 0U; i < 8U; i++)
    {
        solution[i] = matrix[i][8];
    }
    return true;
}

static void camera_matrix_3x3_multiply(double const left[3][3],
                                       double const right[3][3],
                                       double out[3][3])
{
    for (uint32_t row = 0U; row < 3U; row++)
    {
        for (uint32_t column = 0U; column < 3U; column++)
        {
            out[row][column] = 0.0;
            for (uint32_t k = 0U; k < 3U; k++)
            {
                out[row][column] += left[row][k] * right[k][column];
            }
        }
    }
}

static bool camera_fit_homography(uint32_t start,
                                  uint32_t count,
                                  double out[3][3],
                                  double * p_rmse_mm,
                                  double * p_mean_z_mm)
{
    double mean_u = 0.0;
    double mean_v = 0.0;
    double mean_x = 0.0;
    double mean_y = 0.0;
    double mean_z = 0.0;

    if ((count < 4U) || ((start + count) > CAMERA_CALIBRATION_TOP_POINTS))
    {
        return false;
    }

    for (uint32_t i = start; i < (start + count); i++)
    {
        mean_u += (double) g_camera_calibration_samples[i].pixel_x;
        mean_v += (double) g_camera_calibration_samples[i].pixel_y;
        mean_x += (double) g_camera_calibration_samples[i].arm_x_0p1mm / 10.0;
        mean_y += (double) g_camera_calibration_samples[i].arm_y_0p1mm / 10.0;
        mean_z += (double) g_camera_calibration_samples[i].arm_z_0p1mm / 10.0;
    }
    mean_u /= (double) count;
    mean_v /= (double) count;
    mean_x /= (double) count;
    mean_y /= (double) count;
    mean_z /= (double) count;

    double src_energy = 0.0;
    double dst_energy = 0.0;
    for (uint32_t i = start; i < (start + count); i++)
    {
        double const du = (double) g_camera_calibration_samples[i].pixel_x - mean_u;
        double const dv = (double) g_camera_calibration_samples[i].pixel_y - mean_v;
        double const dx = ((double) g_camera_calibration_samples[i].arm_x_0p1mm / 10.0) - mean_x;
        double const dy = ((double) g_camera_calibration_samples[i].arm_y_0p1mm / 10.0) - mean_y;
        src_energy += (du * du) + (dv * dv);
        dst_energy += (dx * dx) + (dy * dy);
    }
    if ((src_energy < 1.0e-6) || (dst_energy < 1.0e-6))
    {
        return false;
    }

    double const src_scale = sqrt((2.0 * (double) count) / src_energy);
    double const dst_scale = sqrt((2.0 * (double) count) / dst_energy);
    double normal[8][9] = {{0.0}};

    for (uint32_t i = start; i < (start + count); i++)
    {
        double const u = src_scale *
                         ((double) g_camera_calibration_samples[i].pixel_x - mean_u);
        double const v = src_scale *
                         ((double) g_camera_calibration_samples[i].pixel_y - mean_v);
        double const x = dst_scale *
                         (((double) g_camera_calibration_samples[i].arm_x_0p1mm / 10.0) - mean_x);
        double const y = dst_scale *
                         (((double) g_camera_calibration_samples[i].arm_y_0p1mm / 10.0) - mean_y);
        double const rows[2][8] =
        {
            {u, v, 1.0, 0.0, 0.0, 0.0, -x * u, -x * v},
            {0.0, 0.0, 0.0, u, v, 1.0, -y * u, -y * v},
        };
        double const values[2] = {x, y};

        for (uint32_t equation = 0U; equation < 2U; equation++)
        {
            for (uint32_t row = 0U; row < 8U; row++)
            {
                for (uint32_t column = 0U; column < 8U; column++)
                {
                    normal[row][column] += rows[equation][row] * rows[equation][column];
                }
                normal[row][8] += rows[equation][row] * values[equation];
            }
        }
    }

    double parameters[8];
    if (!camera_solve_8x8(normal, parameters))
    {
        return false;
    }

    double const normalized[3][3] =
    {
        {parameters[0], parameters[1], parameters[2]},
        {parameters[3], parameters[4], parameters[5]},
        {parameters[6], parameters[7], 1.0},
    };
    double const source_transform[3][3] =
    {
        {src_scale, 0.0, -src_scale * mean_u},
        {0.0, src_scale, -src_scale * mean_v},
        {0.0, 0.0, 1.0},
    };
    double const destination_inverse[3][3] =
    {
        {1.0 / dst_scale, 0.0, mean_x},
        {0.0, 1.0 / dst_scale, mean_y},
        {0.0, 0.0, 1.0},
    };
    double temporary[3][3];
    camera_matrix_3x3_multiply(normalized, source_transform, temporary);
    camera_matrix_3x3_multiply(destination_inverse, temporary, out);
    if (fabs(out[2][2]) < CAMERA_HOMOGRAPHY_EPSILON)
    {
        return false;
    }
    double const scale = out[2][2];
    for (uint32_t row = 0U; row < 3U; row++)
    {
        for (uint32_t column = 0U; column < 3U; column++)
        {
            out[row][column] /= scale;
        }
    }

    double squared_error = 0.0;
    for (uint32_t i = start; i < (start + count); i++)
    {
        double projected_x;
        double projected_y;
        if (!camera_project_top(out,
                                (double) g_camera_calibration_samples[i].pixel_x,
                                (double) g_camera_calibration_samples[i].pixel_y,
                                &projected_x,
                                &projected_y))
        {
            return false;
        }
        double const dx = projected_x -
                          ((double) g_camera_calibration_samples[i].arm_x_0p1mm / 10.0);
        double const dy = projected_y -
                          ((double) g_camera_calibration_samples[i].arm_y_0p1mm / 10.0);
        squared_error += (dx * dx) + (dy * dy);
    }
    *p_rmse_mm = sqrt(squared_error / (double) count);
    *p_mean_z_mm = mean_z;
    return isfinite(*p_rmse_mm);
}

static bool camera_calibration_fit_save_apply(int32_t * p_rmse_0p1mm)
{
    double low_h[3][3];
    double high_h[3][3];
    double low_rmse;
    double high_rmse;
    double low_z;
    double high_z;

    if (!camera_fit_homography(0U,
                               CAMERA_CALIBRATION_TOP_POINTS_PER_LAYER,
                               low_h,
                               &low_rmse,
                               &low_z) ||
        !camera_fit_homography(CAMERA_CALIBRATION_TOP_POINTS_PER_LAYER,
                               CAMERA_CALIBRATION_TOP_POINTS_PER_LAYER,
                               high_h,
                               &high_rmse,
                               &high_z) ||
        !(high_z > low_z) ||
        (low_rmse > CAMERA_CALIBRATION_TOP_MAX_RMSE_MM) ||
        (high_rmse > CAMERA_CALIBRATION_TOP_MAX_RMSE_MM))
    {
        return false;
    }

    ipc_handeye_calibration_t config;
    memset(&config, 0, sizeof(config));
    config.top_z_low_mm = (float) low_z;
    config.top_z_high_mm = (float) high_z;
    /* This workflow calibrates XY only. Preserve the active side-camera Z
     * conversion exactly as it was before calibration. */
    config.side_z_slope_mm_px = (float) g_camera_side_z_slope_mm_px;
    config.side_z_offset_mm = (float) g_camera_side_z_offset_mm;
    for (uint32_t row = 0U; row < 3U; row++)
    {
        for (uint32_t column = 0U; column < 3U; column++)
        {
            config.top_h_low[row][column] = (float) low_h[row][column];
            config.top_h_high[row][column] = (float) high_h[row][column];
        }
    }

    if (!camera_calibration_config_valid(&config) ||
        !camera_calibration_store(&config, low_rmse, high_rmse, 0.0))
    {
        return false;
    }

    int const diagnostic_count = snprintf(
        g_uart_line,
        sizeof(g_uart_line),
        "CAL_XY_FIT low=%ld high=%ld (0.1mm) flash_seq=%lu\r\n",
        (long) round(low_rmse * 10.0),
        (long) round(high_rmse * 10.0),
        (unsigned long) g_camera_calibration_flash_sequence);
    if ((diagnostic_count > 0) &&
        ((size_t) diagnostic_count < sizeof(g_uart_line)))
    {
        camera_uart_send_text(g_uart_line);
    }

    bool sent = false;
    for (uint32_t attempt = 0U; (attempt < 3U) && !sent; attempt++)
    {
        sent = ipc_detection_send_calibration_config(&config);
        if (!sent)
        {
            vTaskDelay(pdMS_TO_TICKS(100U));
        }
    }
    if (!sent)
    {
        return false;
    }
    camera_calibration_apply(&config);

    double maximum_rmse = low_rmse;
    if (high_rmse > maximum_rmse)
    {
        maximum_rmse = high_rmse;
    }
    *p_rmse_0p1mm = (int32_t) round(maximum_rmse * 10.0);
    return true;
}

void camera_calibration_notify_arm_result(uint32_t sequence,
                                          bool success,
                                          int32_t x_0p1mm,
                                          int32_t y_0p1mm,
                                          int32_t z_0p1mm)
{
    g_camera_calibration_arm_result_sequence = sequence;
    g_camera_calibration_arm_result_success = success;
    g_camera_calibration_arm_result_x_0p1mm = x_0p1mm;
    g_camera_calibration_arm_result_y_0p1mm = y_0p1mm;
    g_camera_calibration_arm_result_z_0p1mm = z_0p1mm;
    __DMB();
    g_camera_calibration_arm_result_pending = true;
}

static bool camera_calibration_take_arm_result(uint32_t * p_sequence,
                                               bool * p_success,
                                               int32_t * p_x_0p1mm,
                                               int32_t * p_y_0p1mm,
                                               int32_t * p_z_0p1mm)
{
    bool pending;
    taskENTER_CRITICAL();
    pending = g_camera_calibration_arm_result_pending;
    if (pending)
    {
        *p_sequence = g_camera_calibration_arm_result_sequence;
        *p_success = g_camera_calibration_arm_result_success;
        *p_x_0p1mm = g_camera_calibration_arm_result_x_0p1mm;
        *p_y_0p1mm = g_camera_calibration_arm_result_y_0p1mm;
        *p_z_0p1mm = g_camera_calibration_arm_result_z_0p1mm;
        g_camera_calibration_arm_result_pending = false;
    }
    taskEXIT_CRITICAL();
    return pending;
}

static void camera_calibration_finish(fruit_ui_calibration_state_t state,
                                      int32_t result)
{
    g_camera_calibration_active = false;
    g_camera_calibration_state = CAMERA_CALIBRATION_IDLE;
    g_camera_calibration_observation_count = 0U;
    fruit_ui_set_calibration_camera(false);
    (void) ipc_detection_send_arm_zero();
    fruit_ui_set_calibration_status(state,
                                    g_camera_calibration_index,
                                    CAMERA_CALIBRATION_POINT_COUNT,
                                    result);
}

static void camera_calibration_process_control(bool camera_side_active)
{
    if (!g_camera_calibration_active &&
        fruit_ui_take_calibration_start_request())
    {
        memset(g_camera_calibration_samples, 0,
               sizeof(g_camera_calibration_samples));
        g_camera_calibration_active = true;
        g_camera_calibration_index = 0U;
        g_camera_calibration_sequence = IPC_CALIBRATION_FIRST_SEQUENCE;
        g_camera_calibration_state = CAMERA_CALIBRATION_SEND_MOVE;
        g_camera_calibration_arm_result_pending = false;
        (void) fruit_ui_take_calibration_confirm_request();
        fruit_ui_set_calibration_camera(false);
        fruit_ui_set_calibration_status(FRUIT_UI_CALIBRATION_RUNNING,
                                        0U,
                                        CAMERA_CALIBRATION_POINT_COUNT,
                                        0);
        fruit_ui_set_calibration_moving_target(
            IPC_CALIBRATION_FIRST_X_0P1MM,
            IPC_CALIBRATION_FIRST_Y_0P1MM,
            IPC_CALIBRATION_FIRST_Z_0P1MM);
        camera_uart_send_text("CAL_START green_grape_fixed_grip keep_workspace_clear\r\n");
    }

    if (!g_camera_calibration_active)
    {
        (void) fruit_ui_take_calibration_cancel_request();
        return;
    }

    if (fruit_ui_take_calibration_cancel_request() ||
        !fruit_ui_is_debug_mode_active())
    {
        camera_calibration_finish(FRUIT_UI_CALIBRATION_CANCELLED, 0);
        return;
    }

    camera_calibration_target_t const * p_target =
        &g_camera_calibration_targets[g_camera_calibration_index];

    if (CAMERA_CALIBRATION_SEND_MOVE == g_camera_calibration_state)
    {
        bool const first_point = (0U == g_camera_calibration_index);
        bool const sent = first_point ?
            ipc_detection_send_calibration_first() :
            (((0U != p_target->camera) == camera_side_active) &&
             ipc_detection_send_calibration_move(g_camera_calibration_sequence,
                                                 p_target->x_0p1mm,
                                                 p_target->y_0p1mm,
                                                 p_target->z_0p1mm));
        if (sent)
        {
            int const diagnostic_count = snprintf(
                g_uart_line,
                sizeof(g_uart_line),
                "CAL_MOVE_SENT P%lu/%lu target=(%ld,%ld,%ld) 0.1mm\r\n",
                (unsigned long) (g_camera_calibration_index + 1U),
                (unsigned long) CAMERA_CALIBRATION_POINT_COUNT,
                (long) p_target->x_0p1mm,
                (long) p_target->y_0p1mm,
                (long) p_target->z_0p1mm);
            if ((diagnostic_count > 0) &&
                ((size_t) diagnostic_count < sizeof(g_uart_line)))
            {
                camera_uart_send_text(g_uart_line);
            }
            g_camera_calibration_deadline =
                xTaskGetTickCount() +
                pdMS_TO_TICKS(CAMERA_CALIBRATION_ARM_TIMEOUT_MS);
            g_camera_calibration_state = CAMERA_CALIBRATION_WAIT_ARM;
        }
        return;
    }

    if (CAMERA_CALIBRATION_WAIT_ARM == g_camera_calibration_state)
    {
        uint32_t sequence;
        bool success;
        int32_t x_0p1mm;
        int32_t y_0p1mm;
        int32_t z_0p1mm;
        if (camera_calibration_take_arm_result(&sequence,
                                               &success,
                                               &x_0p1mm,
                                               &y_0p1mm,
                                               &z_0p1mm))
        {
            if (sequence != g_camera_calibration_sequence)
            {
                return;
            }
            if (!success)
            {
                camera_calibration_finish(FRUIT_UI_CALIBRATION_ERROR, 1);
                return;
            }
            g_camera_calibration_arm_x_0p1mm = x_0p1mm;
            g_camera_calibration_arm_y_0p1mm = y_0p1mm;
            g_camera_calibration_arm_z_0p1mm = z_0p1mm;
            g_camera_calibration_observation_count = 0U;
            g_camera_calibration_frame_count = 0U;
            g_camera_calibration_state = CAMERA_CALIBRATION_COLLECT;
            (void) fruit_ui_take_calibration_confirm_request();
            fruit_ui_set_calibration_endpoint(x_0p1mm,
                                              y_0p1mm,
                                              z_0p1mm);
        }
        else if ((int32_t) (xTaskGetTickCount() -
                            g_camera_calibration_deadline) >= 0)
        {
            camera_calibration_finish(FRUIT_UI_CALIBRATION_ERROR, 2);
        }
        return;
    }

    if (CAMERA_CALIBRATION_WAIT_CONFIRM == g_camera_calibration_state)
    {
        if (fruit_ui_take_calibration_confirm_request())
        {
            /* XY calibration always samples the top camera.  If the operator
             * inspected SIDE while waiting, switch back before moving. */
            fruit_ui_set_calibration_camera(false);
            fruit_ui_set_calibration_moving_target(p_target->x_0p1mm,
                                                   p_target->y_0p1mm,
                                                   p_target->z_0p1mm);
            g_camera_calibration_state = CAMERA_CALIBRATION_SEND_MOVE;
        }
    }
}

static void camera_sort_i32(int32_t * p_values, uint32_t count)
{
    for (uint32_t i = 1U; i < count; i++)
    {
        int32_t const value = p_values[i];
        uint32_t position = i;
        while ((position > 0U) && (p_values[position - 1U] > value))
        {
            p_values[position] = p_values[position - 1U];
            position--;
        }
        p_values[position] = value;
    }
}

static bool camera_calibration_green_center(app_detection_result_t const * p_results,
                                            uint32_t count,
                                            int32_t * p_x,
                                            int32_t * p_y)
{
    int64_t largest_area = -1;
    uint32_t selected = count;
    for (uint32_t i = 0U; i < count; i++)
    {
        if (APP_DETECTION_CLASS_GREEN_GRAPE != p_results[i].class_id)
        {
            continue;
        }
        int64_t const width = (int64_t) p_results[i].x2 - p_results[i].x1;
        int64_t const height = (int64_t) p_results[i].y2 - p_results[i].y1;
        int64_t const area = width * height;
        if (area > largest_area)
        {
            largest_area = area;
            selected = i;
        }
    }
    if (selected == count)
    {
        return false;
    }
    *p_x = p_results[selected].x;
    *p_y = p_results[selected].y;
    return true;
}

static void camera_calibration_collect_frame(app_detection_result_t const * p_results,
                                             uint32_t result_count,
                                             bool camera_side_active)
{
    if (!g_camera_calibration_active ||
        (CAMERA_CALIBRATION_COLLECT != g_camera_calibration_state))
    {
        return;
    }

    camera_calibration_target_t const * p_target =
        &g_camera_calibration_targets[g_camera_calibration_index];
    if ((0U != p_target->camera) != camera_side_active)
    {
        return;
    }

    g_camera_calibration_frame_count++;
    int32_t x;
    int32_t y;
    if (camera_calibration_green_center(p_results, result_count, &x, &y) &&
        (g_camera_calibration_observation_count < CAMERA_CALIBRATION_OBSERVATIONS))
    {
        uint32_t const index = g_camera_calibration_observation_count++;
        g_camera_calibration_observation_x[index] = x;
        g_camera_calibration_observation_y[index] = y;
    }

    if (g_camera_calibration_observation_count >= CAMERA_CALIBRATION_OBSERVATIONS)
    {
        int32_t sorted_x[CAMERA_CALIBRATION_OBSERVATIONS];
        int32_t sorted_y[CAMERA_CALIBRATION_OBSERVATIONS];
        memcpy(sorted_x, g_camera_calibration_observation_x, sizeof(sorted_x));
        memcpy(sorted_y, g_camera_calibration_observation_y, sizeof(sorted_y));
        camera_sort_i32(sorted_x, CAMERA_CALIBRATION_OBSERVATIONS);
        camera_sort_i32(sorted_y, CAMERA_CALIBRATION_OBSERVATIONS);
        int32_t const median_x = sorted_x[CAMERA_CALIBRATION_OBSERVATIONS / 2U];
        int32_t const median_y = sorted_y[CAMERA_CALIBRATION_OBSERVATIONS / 2U];
        int64_t sum_x = 0;
        int64_t sum_y = 0;
        uint32_t inliers = 0U;

        for (uint32_t i = 0U; i < CAMERA_CALIBRATION_OBSERVATIONS; i++)
        {
            if ((abs(g_camera_calibration_observation_x[i] - median_x) <=
                 CAMERA_CALIBRATION_MAX_DEVIATION_PX) &&
                (abs(g_camera_calibration_observation_y[i] - median_y) <=
                 CAMERA_CALIBRATION_MAX_DEVIATION_PX))
            {
                sum_x += g_camera_calibration_observation_x[i];
                sum_y += g_camera_calibration_observation_y[i];
                inliers++;
            }
        }

        if (inliers >= CAMERA_CALIBRATION_MIN_INLIERS)
        {
            camera_calibration_sample_t * p_sample =
                &g_camera_calibration_samples[g_camera_calibration_index];
            p_sample->pixel_x = (int32_t) ((sum_x + (int64_t) (inliers / 2U)) /
                                           (int64_t) inliers);
            p_sample->pixel_y = (int32_t) ((sum_y + (int64_t) (inliers / 2U)) /
                                           (int64_t) inliers);
            p_sample->arm_x_0p1mm = g_camera_calibration_arm_x_0p1mm;
            p_sample->arm_y_0p1mm = g_camera_calibration_arm_y_0p1mm;
            p_sample->arm_z_0p1mm = g_camera_calibration_arm_z_0p1mm;
            p_sample->camera = p_target->camera;
            p_sample->layer = p_target->layer;

            int const diagnostic_count = snprintf(
                g_uart_line,
                sizeof(g_uart_line),
                "CAL_SAMPLE %lu cam=%s pixel=(%ld,%ld) arm=(%ld,%ld,%ld) 0.1mm\r\n",
                (unsigned long) (g_camera_calibration_index + 1U),
                (0U != p_sample->camera) ? "side" : "top",
                (long) p_sample->pixel_x,
                (long) p_sample->pixel_y,
                (long) p_sample->arm_x_0p1mm,
                (long) p_sample->arm_y_0p1mm,
                (long) p_sample->arm_z_0p1mm);
            if ((diagnostic_count > 0) &&
                ((size_t) diagnostic_count < sizeof(g_uart_line)))
            {
                camera_uart_send_text(g_uart_line);
            }

            g_camera_calibration_index++;
            if (g_camera_calibration_index >= CAMERA_CALIBRATION_POINT_COUNT)
            {
                int32_t rmse_0p1mm;
                if (camera_calibration_fit_save_apply(&rmse_0p1mm))
                {
                    camera_calibration_finish(FRUIT_UI_CALIBRATION_SUCCESS,
                                              rmse_0p1mm);
                }
                else
                {
                    camera_calibration_finish(FRUIT_UI_CALIBRATION_ERROR, 4);
                }
                return;
            }

            g_camera_calibration_sequence++;
            g_camera_calibration_state = CAMERA_CALIBRATION_WAIT_CONFIRM;
            g_camera_calibration_observation_count = 0U;
            g_camera_calibration_frame_count = 0U;
            fruit_ui_set_calibration_status(FRUIT_UI_CALIBRATION_RUNNING,
                                            g_camera_calibration_index,
                                            CAMERA_CALIBRATION_POINT_COUNT,
                                            0);
            fruit_ui_set_calibration_next_target(
                g_camera_calibration_targets[g_camera_calibration_index].x_0p1mm,
                g_camera_calibration_targets[g_camera_calibration_index].y_0p1mm,
                g_camera_calibration_targets[g_camera_calibration_index].z_0p1mm);
            return;
        }
        g_camera_calibration_observation_count = 0U;
    }

    if (g_camera_calibration_frame_count >= CAMERA_CALIBRATION_MAX_FRAMES)
    {
        camera_calibration_finish(FRUIT_UI_CALIBRATION_ERROR, 3);
    }
}

static void camera_store_u16_le(uint8_t * p_dst, uint16_t value)
{
    p_dst[0] = (uint8_t) value;
    p_dst[1] = (uint8_t) (value >> 8U);
}

static void camera_store_u32_le(uint8_t * p_dst, uint32_t value)
{
    p_dst[0] = (uint8_t) value;
    p_dst[1] = (uint8_t) (value >> 8U);
    p_dst[2] = (uint8_t) (value >> 16U);
    p_dst[3] = (uint8_t) (value >> 24U);
}

static void camera_uart_send_crc_line(char const * p_label, uint32_t crc)
{
    int const count = snprintf(g_uart_line,
                               sizeof(g_uart_line),
                               "%s bytes=%lu crc32=0x%08lX\r\n",
                               p_label,
                               (unsigned long) CAMERA_OV5640_FRAME_BYTES,
                               (unsigned long) crc);
    if ((count > 0) && ((size_t) count < sizeof(g_uart_line)))
    {
        camera_uart_send_text(g_uart_line);
    }
}

static void camera_uart_send_ceu_events_line(void)
{
    uint32_t const events = camera_ov5640_last_ceu_events();
    camera_ov5640_ceu_debug_t debug;
    camera_ov5640_get_ceu_debug(&debug);
    int const count = snprintf(g_uart_line,
                               sizeof(g_uart_line),
                               "CEU_HW events=%08lX start=%lu caps=%08lX csts=%08lX "
                               "flags=%08lX size=%08lX cam=%08lX cyc=%08lX written=%lu rows=%lu\r\n",
                               (unsigned long) events,
                               (unsigned long) debug.capture_start_error,
                               (unsigned long) debug.caps,
                               (unsigned long) debug.status,
                               (unsigned long) debug.events,
                               (unsigned long) debug.data_size,
                               (unsigned long) debug.interface_control,
                               (unsigned long) debug.interface_cycle,
                               (unsigned long) debug.written_bytes,
                               (unsigned long) debug.written_rows);
    if ((count > 0) && ((size_t) count < sizeof(g_uart_line)))
    {
        camera_uart_send_text(g_uart_line);
    }
}

static void camera_uart_send_camera_init_error(camera_ov5640_result_t result)
{
    uint16_t const chip_id = camera_ov5640_chip_id();
    int const count = snprintf(g_uart_line,
                               sizeof(g_uart_line),
                               "CAMERA_INIT_ERR code=%lu step=%lu reg=0x%04X chip=0x%04X\r\n",
                               (unsigned long) result,
                               (unsigned long) camera_ov5640_last_error_step(),
                               (unsigned int) camera_ov5640_last_failed_reg(),
                               (unsigned int) chip_id);

    if ((count > 0) && ((size_t) count < sizeof(g_uart_line)))
    {
        camera_uart_send_text(g_uart_line);
    }
}

static void camera_uart_send_reg_line(uint16_t reg)
{
    uint8_t val = 0U;
    bool const ok = camera_ov5640_read_reg(reg, &val);

    int const count = snprintf(g_uart_line,
                               sizeof(g_uart_line),
                               "OV5640_REG 0x%04X=%s0x%02X\r\n",
                               (unsigned int) reg,
                               ok ? "" : "ERR/",
                               (unsigned int) val);
    if ((count > 0) && ((size_t) count < sizeof(g_uart_line)))
    {
        camera_uart_send_text(g_uart_line);
    }
}

static void camera_uart_send_ov5640_diagnostics(void)
{
    camera_uart_send_text("OV5640_DIAG_BEGIN\r\n");
    camera_uart_send_reg_line(0x300AU);
    camera_uart_send_reg_line(0x300BU);
    camera_uart_send_reg_line(0x4300U);
    camera_uart_send_reg_line(0x501FU);
    camera_uart_send_reg_line(0x4740U);
    camera_uart_send_reg_line(0x4741U);
    camera_uart_send_reg_line(0x503DU);
    camera_uart_send_reg_line(0x3035U);
    camera_uart_send_reg_line(0x3036U);
    camera_uart_send_reg_line(0x3820U);
    camera_uart_send_reg_line(0x3821U);
    camera_uart_send_reg_line(0x3808U);
    camera_uart_send_reg_line(0x3809U);
    camera_uart_send_reg_line(0x380AU);
    camera_uart_send_reg_line(0x380BU);
    camera_uart_send_reg_line(0x380CU);
    camera_uart_send_reg_line(0x380DU);
    camera_uart_send_reg_line(0x380EU);
    camera_uart_send_reg_line(0x380FU);
    camera_uart_send_reg_line(0x3824U);
    camera_uart_send_reg_line(0x460CU);
    camera_uart_send_reg_line(0x4837U);
    camera_uart_send_reg_line(0x3406U);
    camera_uart_send_reg_line(0x5181U);
    camera_uart_send_reg_line(0x5186U);
    camera_uart_send_reg_line(0x5187U);
    camera_uart_send_reg_line(0x5188U);
    camera_uart_send_reg_line(0x5189U);
    camera_uart_send_reg_line(0x518AU);
    camera_uart_send_reg_line(0x518BU);
    camera_uart_send_reg_line(0x518CU);
    camera_uart_send_reg_line(0x518DU);
    camera_uart_send_reg_line(0x518EU);
    camera_uart_send_reg_line(0x518FU);
    camera_uart_send_reg_line(0x5190U);
    camera_uart_send_text("OV5640_DIAG_END\r\n");
}

static camera_ov5640_result_t camera_capture_frame_with_retry(uint8_t * p_frame)
{
    camera_ov5640_result_t result = CAMERA_OV5640_ERR_CAPTURE;

    for (uint32_t attempt = 0U; attempt < CAMERA_CAPTURE_ATTEMPTS; attempt++)
    {
        result = camera_ov5640_capture_frame(p_frame);
        if (CAMERA_OV5640_OK == result)
        {
            return result;
        }

        if ((attempt + 1U) < CAMERA_CAPTURE_ATTEMPTS)
        {
            vTaskDelay(pdMS_TO_TICKS(CAMERA_CAPTURE_RETRY_MS));
        }
    }

    return result;
}

static int32_t camera_median_samples(int32_t const * samples, uint32_t count)
{
    int32_t sorted[CAMERA_TOP_SAMPLES];

    for (uint32_t i = 0U; i < count; i++)
    {
        sorted[i] = samples[i];
    }

    for (uint32_t i = 1U; i < count; i++)
    {
        int32_t const value = sorted[i];
        uint32_t j = i;

        while ((j > 0U) && (sorted[j - 1U] > value))
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }

        sorted[j] = value;
    }

    return sorted[count / 2U];
}

static int32_t camera_median_recent_3(int32_t const samples[CAMERA_BOX_FILTER_SAMPLES],
                                     uint32_t      count)
{
    int32_t sorted[CAMERA_BOX_FILTER_SAMPLES];

    for (uint32_t i = 0U; i < count; i++)
    {
        sorted[i] = samples[i];
    }

    for (uint32_t i = 1U; i < count; i++)
    {
        int32_t const value = sorted[i];
        uint32_t j = i;

        while ((j > 0U) && (sorted[j - 1U] > value))
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = value;
    }

    if (2U == count)
    {
        return (sorted[0] + sorted[1]) / 2;
    }

    return sorted[count / 2U];
}

static void camera_reset_box_filters(bool side_camera)
{
    uint32_t const view = side_camera ? 1U : 0U;
    memset(g_camera_box_filters[view], 0, sizeof(g_camera_box_filters[view]));
}

static int32_t camera_abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static camera_box_filter_t * camera_select_box_filter(app_detection_result_t const * p_result,
                                                       bool                            side_camera,
                                                       bool                            track_used[APP_DETECTION_MAX_RESULTS])
{
    uint32_t const view = side_camera ? 1U : 0U;
    camera_box_filter_t * const filters = g_camera_box_filters[view];
    uint64_t best_distance = UINT64_MAX;
    uint32_t selected = APP_DETECTION_MAX_RESULTS;

    for (uint32_t track = 0U; track < APP_DETECTION_MAX_RESULTS; track++)
    {
        camera_box_filter_t const * const filter = &filters[track];
        if (track_used[track] || !filter->valid || (filter->class_id != p_result->class_id))
        {
            continue;
        }

        uint32_t const last = (filter->next + CAMERA_BOX_FILTER_SAMPLES - 1U) %
                              CAMERA_BOX_FILTER_SAMPLES;
        int32_t const old_center_x = (filter->x1[last] + filter->x2[last]) / 2;
        int32_t const old_center_y = (filter->y1[last] + filter->y2[last]) / 2;
        int32_t const new_center_x = (p_result->x1 + p_result->x2) / 2;
        int32_t const new_center_y = (p_result->y1 + p_result->y2) / 2;
        int32_t const dx = new_center_x - old_center_x;
        int32_t const dy = new_center_y - old_center_y;
        int32_t const old_width = filter->x2[last] - filter->x1[last];
        int32_t const old_height = filter->y2[last] - filter->y1[last];
        int32_t const new_width = p_result->x2 - p_result->x1;
        int32_t const new_height = p_result->y2 - p_result->y1;
        int32_t const gate_x = (old_width > new_width) ? old_width : new_width;
        int32_t const gate_y = (old_height > new_height) ? old_height : new_height;

        if ((camera_abs_i32(dx) > gate_x) || (camera_abs_i32(dy) > gate_y))
        {
            continue;
        }

        uint64_t const distance = ((uint64_t) camera_abs_i32(dx) * (uint64_t) camera_abs_i32(dx)) +
                                  ((uint64_t) camera_abs_i32(dy) * (uint64_t) camera_abs_i32(dy));
        if (distance < best_distance)
        {
            best_distance = distance;
            selected = track;
        }
    }

    if (APP_DETECTION_MAX_RESULTS == selected)
    {
        for (uint32_t track = 0U; track < APP_DETECTION_MAX_RESULTS; track++)
        {
            if (!track_used[track] && !filters[track].valid)
            {
                selected = track;
                break;
            }
        }
    }

    if (APP_DETECTION_MAX_RESULTS == selected)
    {
        uint32_t oldest_missed = 0U;
        for (uint32_t track = 0U; track < APP_DETECTION_MAX_RESULTS; track++)
        {
            if (!track_used[track] &&
                ((APP_DETECTION_MAX_RESULTS == selected) ||
                 (filters[track].missed_frames >= oldest_missed)))
            {
                oldest_missed = filters[track].missed_frames;
                selected = track;
            }
        }
    }

    if (APP_DETECTION_MAX_RESULTS == selected)
    {
        return NULL;
    }

    if (!filters[selected].valid || (UINT64_MAX == best_distance))
    {
        memset(&filters[selected], 0, sizeof(filters[selected]));
        filters[selected].valid = true;
        filters[selected].class_id = p_result->class_id;
    }

    filters[selected].missed_frames = 0U;
    track_used[selected] = true;
    return &filters[selected];
}

static void camera_filter_box(app_detection_result_t const * p_result,
                              fruit_ui_detection_t          * p_detection,
                              bool                            side_camera,
                              bool                            track_used[APP_DETECTION_MAX_RESULTS])
{
    if (p_result->class_id >= APP_DETECTION_MAX_RESULTS)
    {
        return;
    }

    camera_box_filter_t * const p_filter = camera_select_box_filter(p_result,
                                                                    side_camera,
                                                                    track_used);
    if (NULL == p_filter)
    {
        return;
    }
    uint32_t const sample = p_filter->next;
    p_filter->x1[sample] = p_result->x1;
    p_filter->y1[sample] = p_result->y1;
    p_filter->x2[sample] = p_result->x2;
    p_filter->y2[sample] = p_result->y2;
    p_filter->next = (sample + 1U) % CAMERA_BOX_FILTER_SAMPLES;
    if (p_filter->count < CAMERA_BOX_FILTER_SAMPLES)
    {
        p_filter->count++;
    }

    p_detection->x1 = camera_median_recent_3(p_filter->x1, p_filter->count);
    p_detection->y1 = camera_median_recent_3(p_filter->y1, p_filter->count);
    p_detection->x2 = camera_median_recent_3(p_filter->x2, p_filter->count);
    p_detection->y2 = camera_median_recent_3(p_filter->y2, p_filter->count);

    int32_t const center_x = (p_detection->x1 + p_detection->x2) / 2;
    int32_t const center_y = (p_detection->y1 + p_detection->y2) / 2;
    int32_t const padding_x = ((p_detection->x2 - p_detection->x1) +
                               CAMERA_BOX_PADDING_DIVISOR - 1) /
                              CAMERA_BOX_PADDING_DIVISOR;
    int32_t const padding_y = ((p_detection->y2 - p_detection->y1) +
                               CAMERA_BOX_PADDING_DIVISOR - 1) /
                              CAMERA_BOX_PADDING_DIVISOR;
    int32_t const crop_x1 = ((int32_t) CAMERA_OV5640_WIDTH - (int32_t) CAMERA_OV5640_HEIGHT) / 2;
    int32_t const crop_x2 = crop_x1 + (int32_t) CAMERA_OV5640_HEIGHT;
    p_detection->x1 = ((p_detection->x1 - padding_x) > crop_x1) ?
                      (p_detection->x1 - padding_x) : crop_x1;
    p_detection->y1 = (p_detection->y1 > padding_y) ? (p_detection->y1 - padding_y) : 0;
    p_detection->x2 = ((p_detection->x2 + padding_x) < crop_x2) ?
                      (p_detection->x2 + padding_x) : crop_x2;
    p_detection->y2 = ((p_detection->y2 + padding_y) < (int32_t) CAMERA_OV5640_HEIGHT) ?
                      (p_detection->y2 + padding_y) : (int32_t) CAMERA_OV5640_HEIGHT;
    p_detection->x = center_x;
    p_detection->y = center_y;
}

static void camera_finish_box_filter_frame(bool side_camera,
                                           bool const track_used[APP_DETECTION_MAX_RESULTS])
{
    uint32_t const view = side_camera ? 1U : 0U;

    for (uint32_t track = 0U; track < APP_DETECTION_MAX_RESULTS; track++)
    {
        camera_box_filter_t * const filter = &g_camera_box_filters[view][track];
        if (filter->valid && !track_used[track])
        {
            filter->missed_frames++;
            if (filter->missed_frames >= CAMERA_BOX_FILTER_SAMPLES)
            {
                memset(filter, 0, sizeof(*filter));
            }
        }
    }
}

static void camera_reset_top_samples(void)
{
    memset(g_camera_top_samples, 0, sizeof(g_camera_top_samples));
    g_camera_top_frame_id = 0U;
}

static bool camera_detection_pigment_confirmed(app_detection_result_t const * p_result)
{
    return (NULL != p_result) &&
           (p_result->mean_r >= 0) &&
           (p_result->mean_g >= 0) &&
           (p_result->mean_b >= 0);
}

static void camera_record_top_samples(app_detection_result_t const * p_results,
                                      uint32_t                       result_count)
{
    g_camera_top_frame_id++;

    for (uint32_t class_id = 0U; class_id < APP_DETECTION_MAX_RESULTS; class_id++)
    {
        uint32_t selected = result_count;
        bool selected_confirmed = false;

        for (uint32_t i = 0U; i < result_count; i++)
        {
            if (p_results[i].class_id != class_id)
            {
                continue;
            }

            bool const confirmed = camera_detection_pigment_confirmed(&p_results[i]);
            if ((result_count == selected) || (confirmed && !selected_confirmed))
            {
                selected = i;
                selected_confirmed = confirmed;
            }
        }

        if (result_count == selected)
        {
            continue;
        }

        camera_top_samples_t * p_samples = &g_camera_top_samples[class_id];
        uint32_t const sample = p_samples->next;
        p_samples->x[sample] = p_results[selected].x;
        p_samples->y[sample] = p_results[selected].y;
        p_samples->frame_id[sample] = g_camera_top_frame_id;
        p_samples->next = (sample + 1U) % CAMERA_TOP_SAMPLES;
        if (p_samples->count < CAMERA_TOP_SAMPLES)
        {
            p_samples->count++;
        }
    }
}

static uint32_t camera_get_stable_top_results(app_detection_result_t * p_results)
{
    uint32_t result_count = 0U;

    /* The relaxed purple rule must not finalise the whole batch before
     * tomato and green grape have had enough frames to reach five samples. */
    if (g_camera_top_frame_id < CAMERA_TOP_BATCH_MIN_FRAMES)
    {
        return 0U;
    }

    if (g_camera_top_frame_id < CAMERA_TOP_BATCH_MAX_FRAMES)
    {
        for (uint32_t class_id = 0U; class_id < APP_DETECTION_MAX_RESULTS; class_id++)
        {
            camera_top_samples_t const * const p_samples = &g_camera_top_samples[class_id];
            uint32_t const required_samples =
                (APP_DETECTION_CLASS_PURPLE_GRAPE == class_id) ?
                CAMERA_PURPLE_TOP_SAMPLES : CAMERA_TOP_SAMPLES;

            /* Once a class appears, give it up to ten top frames to satisfy
             * its own stability rule before finalising this detection batch. */
            if ((p_samples->count > 0U) && (p_samples->count < required_samples))
            {
                return 0U;
            }
        }
    }

    for (uint32_t class_id = 0U;
         (class_id < APP_DETECTION_MAX_RESULTS) && (result_count < APP_DETECTION_MAX_RESULTS);
         class_id++)
    {
        camera_top_samples_t const * p_samples = &g_camera_top_samples[class_id];
        bool const purple = (APP_DETECTION_CLASS_PURPLE_GRAPE == class_id);
        uint32_t const required_samples = purple ? CAMERA_PURPLE_TOP_SAMPLES : CAMERA_TOP_SAMPLES;
        int32_t recent_x[CAMERA_TOP_SAMPLES];
        int32_t recent_y[CAMERA_TOP_SAMPLES];
        uint32_t recent_count = 0U;

        for (uint32_t sample = 0U; sample < p_samples->count; sample++)
        {
            uint32_t const age = g_camera_top_frame_id - p_samples->frame_id[sample];
            if (purple && (age >= CAMERA_PURPLE_TOP_WINDOW_FRAMES))
            {
                continue;
            }

            recent_x[recent_count] = p_samples->x[sample];
            recent_y[recent_count] = p_samples->y[sample];
            recent_count++;
        }

        if (recent_count < required_samples)
        {
            continue;
        }

        if (purple)
        {
            int32_t const median_x = camera_median_samples(recent_x, recent_count);
            int32_t const median_y = camera_median_samples(recent_y, recent_count);
            int32_t consistent_x[CAMERA_TOP_SAMPLES];
            int32_t consistent_y[CAMERA_TOP_SAMPLES];
            uint32_t consistent_count = 0U;

            for (uint32_t sample = 0U; sample < recent_count; sample++)
            {
                if ((camera_abs_i32(recent_x[sample] - median_x) <=
                     CAMERA_PURPLE_TOP_MAX_DEVIATION_PX) &&
                    (camera_abs_i32(recent_y[sample] - median_y) <=
                     CAMERA_PURPLE_TOP_MAX_DEVIATION_PX))
                {
                    consistent_x[consistent_count] = recent_x[sample];
                    consistent_y[consistent_count] = recent_y[sample];
                    consistent_count++;
                }
            }

            if (consistent_count < CAMERA_PURPLE_TOP_SAMPLES)
            {
                continue;
            }

            memcpy(recent_x, consistent_x, consistent_count * sizeof(consistent_x[0]));
            memcpy(recent_y, consistent_y, consistent_count * sizeof(consistent_y[0]));
            recent_count = consistent_count;
        }

        p_results[result_count] = (app_detection_result_t) {0};
        p_results[result_count].class_id = class_id;
        p_results[result_count].x = camera_median_samples(recent_x, recent_count);
        p_results[result_count].y = camera_median_samples(recent_y, recent_count);
        result_count++;
    }

    return result_count;
}

static bool camera_prime_selected(uint32_t settle_ms,
                                  uint32_t attempts,
                                  bool require_visible_content)
{
    vTaskDelay(pdMS_TO_TICKS(settle_ms));

    for (uint32_t frame = 0U; frame < attempts; frame++)
    {
        if ((CAMERA_OV5640_OK == camera_capture_frame_with_retry(g_camera_frame)) &&
            (!require_visible_content || camera_frame_has_visible_content(g_camera_frame)))
        {
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(CAMERA_CAPTURE_RETRY_MS));
    }

    return false;
}

static bool camera_full_recover_selected(bool side_camera,
                                         bool illumination_enabled)
{
    uint32_t const camera_index = side_camera ? 1U : 0U;

    (void) camera_ov5640_stop();
    g_camera_initialized[0] = false;
    g_camera_initialized[1] = false;
    g_camera_illumination_state_valid = false;

    if (FSP_SUCCESS != g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                                 CAMERA_MUX_SEL,
                                                 side_camera ? BSP_IO_LEVEL_LOW :
                                                               BSP_IO_LEVEL_HIGH))
    {
        camera_uart_send_text("CAM_RECOVER_ERR select\r\n");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(side_camera ? CAMERA_SIDE_MUX_SETTLE_MS :
                                           CAMERA_MUX_SETTLE_MS));

    camera_ov5640_result_t const init_result = camera_ov5640_init();
    if (CAMERA_OV5640_OK != init_result)
    {
        camera_uart_send_camera_init_error(init_result);
        return false;
    }

    g_camera_initialized[camera_index] = true;
    if (!camera_set_illumination(illumination_enabled))
    {
        return false;
    }

    if (!camera_prime_selected(side_camera ? CAMERA_SIDE_SWITCH_SETTLE_MS :
                                             CAMERA_SWITCH_SETTLE_MS,
                               CAMERA_FAST_PRIME_ATTEMPTS,
                               illumination_enabled))
    {
        return false;
    }

    camera_reset_box_filters(side_camera);
    camera_uart_send_text(side_camera ? "CAM_ACTIVE SIDE RECOVERED\r\n" :
                                        "CAM_ACTIVE TOP RECOVERED\r\n");
    return true;
}

static bool camera_set_illumination(bool enabled)
{
    if (g_camera_illumination_state_valid &&
        (enabled == g_camera_illumination_enabled))
    {
        return true;
    }

    for (uint32_t attempt = 0U; attempt < CAMERA_LED_WRITE_ATTEMPTS; attempt++)
    {
        if (camera_ov5640_set_strobe_led(enabled))
        {
            g_camera_illumination_enabled = enabled;
            g_camera_illumination_state_valid = true;
            return true;
        }

        if ((attempt + 1U) < CAMERA_LED_WRITE_ATTEMPTS)
        {
            vTaskDelay(pdMS_TO_TICKS(CAMERA_LED_RETRY_MS));
        }
    }

    g_camera_illumination_state_valid = false;
    camera_uart_send_text(enabled ? "CAM_LED_ERR on_retry_exhausted\r\n" :
                                    "CAM_LED_ERR off_retry_exhausted\r\n");
    return false;
}

static bool camera_settle_debug_light(bool expected_enabled)
{
    uint32_t remaining_ms = CAMERA_DEBUG_LIGHT_SETTLE_MS;

    while (remaining_ms > 0U)
    {
        uint32_t const delay_ms = (remaining_ms > 20U) ? 20U : remaining_ms;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        remaining_ms -= delay_ms;

        if (!fruit_ui_is_debug_mode_active() ||
            (fruit_ui_debug_light_requested() != expected_enabled))
        {
            return false;
        }
    }

    for (uint32_t frame = 0U; frame < CAMERA_DEBUG_LIGHT_DISCARD_FRAMES; frame++)
    {
        if (!fruit_ui_is_debug_mode_active() ||
            (fruit_ui_debug_light_requested() != expected_enabled))
        {
            return false;
        }

        (void) camera_capture_frame_with_retry(g_camera_frame);
        vTaskDelay(pdMS_TO_TICKS(CAMERA_CAPTURE_RETRY_MS));
    }

    return true;
}

static bool camera_switch_to(bool side_camera, bool illumination_enabled)
{
    uint32_t const camera_index = side_camera ? 1U : 0U;
    bool fast_path = g_camera_initialized[camera_index];

    /* Ensure the currently selected module's illumination is off before it is paused. */
    if (!camera_set_illumination(false))
    {
        camera_uart_send_text("CAM_SWITCH_ERR current_light_off\r\n");
        return false;
    }

    camera_ov5640_result_t const stop_result = camera_ov5640_pause();

    if (CAMERA_OV5640_OK != stop_result)
    {
        camera_uart_send_text("CAM_SWITCH_ERR stop\r\n");
    }

    if (FSP_SUCCESS != g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                                 CAMERA_MUX_SEL,
                                                 side_camera ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH))
    {
        camera_uart_send_text("CAM_SWITCH_ERR select\r\n");
        return false;
    }

    /* The mux now addresses another OV5640, whose STROBE state is unknown. */
    g_camera_illumination_state_valid = false;

    vTaskDelay(pdMS_TO_TICKS(side_camera ? CAMERA_SIDE_MUX_SETTLE_MS : CAMERA_MUX_SETTLE_MS));

    camera_ov5640_result_t const start_result = fast_path ?
        camera_ov5640_resume() : camera_ov5640_init_warm();
    if (CAMERA_OV5640_OK != start_result)
    {
        camera_uart_send_text(fast_path ? "CAM_FAST_RESUME_ERR\r\n" :
                                           "CAM_WARM_INIT_ERR\r\n");
        return camera_full_recover_selected(side_camera, illumination_enabled);
    }

    g_camera_initialized[camera_index] = true;

    if (!camera_set_illumination(illumination_enabled))
    {
        camera_uart_send_text(side_camera ? "CAM_SWITCH_ERR side_light\r\n" :
                                            "CAM_SWITCH_ERR top_light\r\n");
        return false;
    }

    if (!camera_prime_selected(fast_path ? CAMERA_FAST_SWITCH_SETTLE_MS :
                                           (side_camera ? CAMERA_SIDE_SWITCH_SETTLE_MS :
                                                          CAMERA_SWITCH_SETTLE_MS),
                               CAMERA_FAST_PRIME_ATTEMPTS,
                               illumination_enabled))
    {
        camera_uart_send_text("CAM_FAST_PRIME_ERR full_recover\r\n");
        return camera_full_recover_selected(side_camera, illumination_enabled);
    }

    camera_reset_box_filters(side_camera);
    camera_uart_send_text(side_camera ? (fast_path ? "CAM_ACTIVE SIDE FAST\r\n" :
                                                     "CAM_ACTIVE SIDE WARM\r\n") :
                                        (fast_path ? "CAM_ACTIVE TOP FAST\r\n" :
                                                     "CAM_ACTIVE TOP WARM\r\n"));
    return true;
}

static bool camera_publish_snapshot(uint8_t const                 * p_rgb565_frame,
                                    app_detection_result_t const * p_results,
                                    uint32_t                       result_count,
                                    bool                           side_camera,
                                    bool                           debug_snapshot)
{
    fruit_ui_detection_t detections[FRUIT_UI_MAX_DETECTIONS];
    bool box_track_used[APP_DETECTION_MAX_RESULTS] = {false};
    uint32_t detection_count = 0U;

    for (uint32_t i = 0U; (i < result_count) && (detection_count < FRUIT_UI_MAX_DETECTIONS); i++)
    {
        fruit_ui_target_t const target = camera_stream_target_from_class(p_results[i].class_id);
        if (FRUIT_UI_TARGET_NONE == target)
        {
            continue;
        }

        detections[detection_count].target = target;
        detections[detection_count].x = p_results[i].x;
        detections[detection_count].y = p_results[i].y;
        detections[detection_count].z = 0;
        detections[detection_count].x1 = p_results[i].x1;
        detections[detection_count].y1 = p_results[i].y1;
        detections[detection_count].x2 = p_results[i].x2;
        detections[detection_count].y2 = p_results[i].y2;
        detections[detection_count].mean_r = p_results[i].mean_r;
        detections[detection_count].mean_g = p_results[i].mean_g;
        detections[detection_count].mean_b = p_results[i].mean_b;
        detections[detection_count].green_ratio_0p1 = p_results[i].green_ratio_0p1;
        camera_filter_box(&p_results[i],
                          &detections[detection_count],
                          side_camera,
                          box_track_used);
        detection_count++;
    }

    camera_finish_box_filter_frame(side_camera, box_track_used);

    return debug_snapshot ?
           fruit_ui_publish_debug_snapshot(p_rgb565_frame,
                                           detections,
                                           detection_count,
                                           side_camera) :
           fruit_ui_publish_task_snapshot(p_rgb565_frame,
                                          detections,
                                          detection_count,
                                           side_camera);
}

static bool camera_frame_has_visible_content(uint8_t const * p_rgb565_frame)
{
    uint32_t sampled_pixels = 0U;
    uint32_t visible_pixels = 0U;

    if (NULL == p_rgb565_frame)
    {
        return false;
    }

    /*
     * A failed side-camera transition commonly produces a valid CEU transfer
     * whose RGB565 payload is almost entirely zero.  Sample the whole image so
     * a small dark object does not make an otherwise illuminated scene fail.
     */
    for (uint32_t y = 0U; y < CAMERA_OV5640_HEIGHT; y += 16U)
    {
        for (uint32_t x = 0U; x < CAMERA_OV5640_WIDTH; x += 16U)
        {
            uint32_t const offset = ((y * CAMERA_OV5640_WIDTH) + x) *
                                    CAMERA_OV5640_BYTES_PER_PIXEL;
            uint16_t const pixel = (uint16_t) (p_rgb565_frame[offset] |
                                               ((uint16_t) p_rgb565_frame[offset + 1U] << 8));
            uint32_t const level = ((pixel >> 11) & 0x1FU) +
                                   ((pixel >> 5) & 0x3FU) +
                                   (pixel & 0x1FU);

            sampled_pixels++;
            if (level >= CAMERA_SIDE_VISIBLE_LEVEL)
            {
                visible_pixels++;
            }
        }
    }

    /* One percent is enough to reject an all-black/stale CEU buffer safely. */
    return (visible_pixels * 100U) >= sampled_pixels;
}

static bool camera_wait_for_task_side_preview(void)
{
    uint32_t ready_frames = 0U;
    uint32_t black_frames = 0U;
    uint32_t capture_failures = 0U;

    for (uint32_t attempt = 0U; attempt < CAMERA_SIDE_READY_ATTEMPTS; attempt++)
    {
        if (fruit_ui_is_debug_mode_active() ||
            (FRUIT_UI_TASK_TOP_AND_SIDE != fruit_ui_get_task_mode()))
        {
            return false;
        }

        if (CAMERA_OV5640_OK != camera_capture_frame_with_retry(g_camera_frame))
        {
            capture_failures++;
            ready_frames = 0U;
            continue;
        }

        if (!camera_frame_has_visible_content(g_camera_frame))
        {
            black_frames++;
            ready_frames = 0U;
            vTaskDelay(pdMS_TO_TICKS(CAMERA_SIDE_READY_GAP_MS));
            continue;
        }

        uint32_t result_count = 0U;
        if (!app_detection_run_frame(g_camera_frame,
                                     g_detection_results,
                                     APP_DETECTION_MAX_RESULTS,
                                     &result_count,
                                     NULL))
        {
            ready_frames = 0U;
            vTaskDelay(pdMS_TO_TICKS(CAMERA_SIDE_READY_GAP_MS));
            continue;
        }

        (void) camera_publish_snapshot(g_camera_frame,
                                       g_detection_results,
                                       result_count,
                                       true,
                                       false);

        ready_frames++;
        if (ready_frames >= CAMERA_SIDE_READY_FRAMES)
        {
            if ((black_frames > 0U) || (capture_failures > 0U))
            {
                int const count = snprintf(g_uart_line,
                                           sizeof(g_uart_line),
                                           "SIDE_READY black=%lu capture_fail=%lu\r\n",
                                           (unsigned long) black_frames,
                                           (unsigned long) capture_failures);
                if ((count > 0) && ((size_t) count < sizeof(g_uart_line)))
                {
                    camera_uart_send_text(g_uart_line);
                }
            }
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(CAMERA_SIDE_READY_GAP_MS));
    }

    int const count = snprintf(g_uart_line,
                               sizeof(g_uart_line),
                               "SIDE_READY_ERR black=%lu capture_fail=%lu\r\n",
                               (unsigned long) black_frames,
                               (unsigned long) capture_failures);
    if ((count > 0) && ((size_t) count < sizeof(g_uart_line)))
    {
        camera_uart_send_text(g_uart_line);
    }
    return false;
}

static void camera_process_task_joint5_request(void)
{
    int32_t angle_deg;

    if (!fruit_ui_take_task_joint5_request(&angle_deg))
    {
        return;
    }

    while (!ipc_detection_send_task_joint5(angle_deg))
    {
        camera_uart_send_text("TASK_JOINT5_ERR ipc_send_retry\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }

    int const count = snprintf(g_uart_line,
                               sizeof(g_uart_line),
                               "TASK_JOINT5_IPC_SENT angle=%ld\r\n",
                               (long) angle_deg);
    if ((count > 0) && ((size_t) count < sizeof(g_uart_line)))
    {
        camera_uart_send_text(g_uart_line);
    }
}

static int32_t camera_select_side_detection(app_detection_result_t const * p_results,
                                            uint32_t                       result_count,
                                            uint32_t                       class_id,
                                            int32_t const                  x_samples[CAMERA_SIDE_SAMPLES],
                                            int32_t const                  y_samples[CAMERA_SIDE_SAMPLES],
                                            uint32_t                       sample_count)
{
    bool has_confirmed = false;
    for (uint32_t i = 0U; i < result_count; i++)
    {
        if ((p_results[i].class_id == class_id) &&
            (p_results[i].y >= CAMERA_SIDE_Y_MIN_PX) &&
            camera_detection_pigment_confirmed(&p_results[i]))
        {
            has_confirmed = true;
            break;
        }
    }

    int32_t reference_x = 0;
    int32_t reference_y = 0;
    if (sample_count > 0U)
    {
        reference_x = camera_median_samples(x_samples, sample_count);
        reference_y = camera_median_samples(y_samples, sample_count);
    }

    uint64_t best_distance = UINT64_MAX;
    int32_t selected = -1;
    for (uint32_t i = 0U; i < result_count; i++)
    {
        app_detection_result_t const * const p_result = &p_results[i];
        if ((p_result->class_id != class_id) ||
            (p_result->y < CAMERA_SIDE_Y_MIN_PX) ||
            (has_confirmed && !camera_detection_pigment_confirmed(p_result)))
        {
            continue;
        }

        if (0U == sample_count)
        {
            return (int32_t) i;
        }

        int32_t const dx = p_result->x - reference_x;
        int32_t const dy = p_result->y - reference_y;
        if ((camera_abs_i32(dx) > CAMERA_SIDE_SAMPLE_MAX_DEVIATION_PX) ||
            (camera_abs_i32(dy) > CAMERA_SIDE_SAMPLE_MAX_DEVIATION_PX))
        {
            continue;
        }

        uint64_t const distance = ((uint64_t) camera_abs_i32(dx) * (uint64_t) camera_abs_i32(dx)) +
                                  ((uint64_t) camera_abs_i32(dy) * (uint64_t) camera_abs_i32(dy));
        if (distance < best_distance)
        {
            best_distance = distance;
            selected = (int32_t) i;
        }
    }

    return selected;
}

static bool camera_collect_side_samples(app_detection_result_t const * p_top_results,
                                        uint32_t                       target_count,
                                        int32_t                      * p_side_x,
                                        int32_t                      * p_side_y,
                                        bool                         * p_side_valid)
{
    int32_t x_samples[APP_DETECTION_MAX_RESULTS][CAMERA_SIDE_SAMPLES];
    int32_t y_samples[APP_DETECTION_MAX_RESULTS][CAMERA_SIDE_SAMPLES];
    uint32_t sample_count[APP_DETECTION_MAX_RESULTS] = {0U};
    uint32_t valid_frame_count = 0U;
    uint32_t dropped_frame_count = 0U;
    uint32_t black_frame_count = 0U;

    for (uint32_t target = 0U; target < target_count; target++)
    {
        p_side_valid[target] = false;
    }

    for (uint32_t frame = 0U; frame < CAMERA_SIDE_MAX_FRAMES; frame++)
    {
        if (fruit_ui_is_debug_mode_active())
        {
            return false;
        }

        if (FRUIT_UI_TASK_TOP_AND_SIDE != fruit_ui_get_task_mode())
        {
            (void) camera_set_illumination(false);
            return false;
        }

        bool all_complete = true;
        for (uint32_t target = 0U; target < target_count; target++)
        {
            if (sample_count[target] < CAMERA_SIDE_SAMPLES)
            {
                all_complete = false;
                break;
            }
        }

        if (all_complete)
        {
            break;
        }

        if (CAMERA_OV5640_OK != camera_capture_frame_with_retry(g_camera_frame))
        {
            dropped_frame_count++;
            continue;
        }

        if (!camera_frame_has_visible_content(g_camera_frame))
        {
            black_frame_count++;
            continue;
        }

        valid_frame_count++;

        uint32_t result_count = 0U;
        if (!app_detection_run_frame(g_camera_frame,
                                     g_detection_results,
                                     APP_DETECTION_MAX_RESULTS,
                                     &result_count,
                                     NULL))
        {
            camera_uart_send_text("SIDE_DET_ERR run\r\n");
            continue;
        }

        (void) camera_publish_snapshot(g_camera_frame,
                                       g_detection_results,
                                       result_count,
                                       true,
                                       false);

        for (uint32_t i = 0U; i < result_count; i++)
        {
            if (g_detection_results[i].y < CAMERA_SIDE_Y_MIN_PX)
            {
                continue;
            }

            int const det_count = snprintf(g_uart_line,
                                           sizeof(g_uart_line),
                                           "SIDE_DET class=%lu x=%ld y=%ld\r\n",
                                           (unsigned long) g_detection_results[i].class_id,
                                           (long) g_detection_results[i].x,
                                           (long) g_detection_results[i].y);
            if ((det_count > 0) && ((size_t) det_count < sizeof(g_uart_line)))
            {
                camera_uart_send_text(g_uart_line);
            }

        }

        for (uint32_t target = 0U; target < target_count; target++)
        {
            if (sample_count[target] >= CAMERA_SIDE_SAMPLES)
            {
                continue;
            }

            int32_t const selected = camera_select_side_detection(g_detection_results,
                                                                   result_count,
                                                                   p_top_results[target].class_id,
                                                                   x_samples[target],
                                                                   y_samples[target],
                                                                   sample_count[target]);
            if (selected >= 0)
            {
                uint32_t const sample = sample_count[target];
                x_samples[target][sample] = g_detection_results[(uint32_t) selected].x;
                y_samples[target][sample] = g_detection_results[(uint32_t) selected].y;
                sample_count[target]++;
            }
        }
    }

    if ((dropped_frame_count > 0U) || (black_frame_count > 0U))
    {
        int const count = snprintf(g_uart_line,
                                   sizeof(g_uart_line),
                                   "SIDE_CAPTURE valid=%lu dropped=%lu black=%lu\r\n",
                                   (unsigned long) valid_frame_count,
                                   (unsigned long) dropped_frame_count,
                                   (unsigned long) black_frame_count);
        if ((count > 0) && ((size_t) count < sizeof(g_uart_line)))
        {
            camera_uart_send_text(g_uart_line);
        }
    }

    for (uint32_t target = 0U; target < target_count; target++)
    {
        if (CAMERA_SIDE_SAMPLES == sample_count[target])
        {
            p_side_x[target] = camera_median_samples(x_samples[target], CAMERA_SIDE_SAMPLES);
            p_side_y[target] = camera_median_samples(y_samples[target], CAMERA_SIDE_SAMPLES);
            p_side_valid[target] = true;
        }
        else
        {
            int const count = snprintf(g_uart_line,
                                       sizeof(g_uart_line),
                                       "SIDE_DET_ERR class=%lu samples=%lu\r\n",
                                       (unsigned long) p_top_results[target].class_id,
                                       (unsigned long) sample_count[target]);

            if ((count > 0) && ((size_t) count < sizeof(g_uart_line)))
            {
                camera_uart_send_text(g_uart_line);
            }
        }
    }

    return true;
}

static bool camera_stream_send_frame(void)
{
    return camera_uart_send_bytes(g_camera_frame, CAMERA_OV5640_FRAME_BYTES);
}

static bool camera_stream_send_frame_dump(void)
{
    static uint8_t const magic[8] = {'C', 'E', 'U', '5', '6', '5', '0', '1'};
    uint8_t header[CAMERA_FRAME_HEADER_BYTES];
    uint32_t const crc = camera_crc32(g_camera_frame, CAMERA_OV5640_FRAME_BYTES);

    memcpy(header, magic, sizeof(magic));
    camera_store_u16_le(&header[8], (uint16_t) CAMERA_OV5640_WIDTH);
    camera_store_u16_le(&header[10], (uint16_t) CAMERA_OV5640_HEIGHT);
    camera_store_u32_le(&header[12], CAMERA_FRAME_FORMAT_RGB565_LE);
    camera_store_u32_le(&header[16], CAMERA_OV5640_FRAME_BYTES);
    camera_store_u32_le(&header[20], crc);

    camera_uart_send_text("FRAME_DUMP_BEGIN RGB565LE 640x480 wait_about_55s\r\n");

    if (!camera_uart_send_bytes(header, sizeof(header)) || !camera_stream_send_frame())
    {
        camera_uart_send_text("FRAME_DUMP_ERROR\r\n");
        return false;
    }

    camera_uart_send_crc_line("FRAME_DUMP_END", crc);
    return true;
}

void camera_stream_task(void)
{
    uint32_t batch_id = 0U;
    TickType_t last_task_preview_tick = 0U;
    TickType_t task_preview_start_tick = 0U;
    uint32_t active_task_generation = 0U;
    bool recognition_complete = false;
    bool camera_side_active = false;
    bool task_preview_started = false;
    uint32_t consecutive_capture_failures = 0U;

    if (camera_debug_uart_init())
    {
        camera_uart_send_text("FW=CPU0_CEU_V3\r\n");
    }

    /* Do not start the camera until the operator selects a task or opens Settings. */
    while ((FRUIT_UI_TASK_NONE == fruit_ui_get_task_mode()) &&
           !fruit_ui_is_debug_mode_active())
    {
        vTaskDelay(pdMS_TO_TICKS(20U));
    }

    /* Move joint 5 to the selected task's view before camera recognition. */
    camera_process_task_joint5_request();

    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, CAMERA_MUX_SEL, BSP_IO_LEVEL_HIGH);
    vTaskDelay(pdMS_TO_TICKS(CAMERA_MUX_SETTLE_MS));

    camera_ov5640_result_t camera_init_result;
    do
    {
        camera_init_result = camera_ov5640_init();
        if (CAMERA_OV5640_OK != camera_init_result)
        {
            camera_uart_send_camera_init_error(camera_init_result);
            vTaskDelay(pdMS_TO_TICKS(500U));
        }
    } while (CAMERA_OV5640_OK != camera_init_result);
    g_camera_initialized[0] = true;
    g_camera_initialized[1] = false;

    bool const initial_task_light =
        (FRUIT_UI_TASK_NONE != fruit_ui_get_task_mode()) &&
        app_detection_get_top_camera_light_default();
    if (!camera_set_illumination(initial_task_light))
    {
        camera_uart_send_text("CAM_INIT_ERR top_light\r\n");
    }

    if (!app_detection_init())
    {
        camera_uart_send_text("DET_ERR init\r\n");

        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }

    if (ospi_flash_is_ready() && camera_calibration_load())
    {
        camera_uart_send_text("CAL_LOAD_OK\r\n");
    }

    camera_uart_send_ov5640_diagnostics();

    vTaskDelay(pdMS_TO_TICKS(CAMERA_AE_SETTLE_MS));

    for (uint32_t i = 0U; i < CAMERA_DISCARD_FRAMES; i++)
    {
        (void) camera_capture_frame_with_retry(g_camera_frame);
        vTaskDelay(pdMS_TO_TICKS(100U));
    }

    fruit_ui_set_task_camera(false);

    while (1)
    {
        camera_process_task_joint5_request();
        camera_calibration_process_control(camera_side_active);

        uint32_t const task_generation = fruit_ui_get_task_generation();
        if (task_generation != active_task_generation)
        {
            active_task_generation = task_generation;
            recognition_complete = false;
            task_preview_started = false;
            task_preview_start_tick = 0U;
            last_task_preview_tick = 0U;
            camera_reset_top_samples();
            camera_reset_box_filters(false);
            camera_reset_box_filters(true);
        }

        bool const debug_active = fruit_ui_is_debug_mode_active();
        bool const debug_side_requested = debug_active &&
                                           fruit_ui_debug_side_camera_requested();
        if (debug_side_requested != camera_side_active)
        {
            bool const debug_light_requested = fruit_ui_debug_light_requested();

            if (!camera_switch_to(debug_side_requested, debug_light_requested))
            {
                camera_uart_send_text(debug_side_requested ?
                                      "CAM_SWITCH_ERR debug_side\r\n" :
                                      "CAM_SWITCH_ERR debug_top\r\n");

                if (debug_side_requested &&
                    camera_switch_to(false, debug_light_requested))
                {
                    camera_side_active = false;
                }

                vTaskDelay(pdMS_TO_TICKS(100U));
                continue;
            }

            camera_side_active = debug_side_requested;
        }

        if (debug_active)
        {
            bool const debug_light_requested = fruit_ui_debug_light_requested();
            bool const light_changed = !g_camera_illumination_state_valid ||
                                       (debug_light_requested != g_camera_illumination_enabled);

            if (light_changed)
            {
                if (!camera_set_illumination(debug_light_requested))
                {
                    vTaskDelay(pdMS_TO_TICKS(100U));
                    continue;
                }

                camera_uart_send_text(debug_light_requested ?
                                      "CAM_LED DEBUG ON\r\n" :
                                      "CAM_LED DEBUG OFF\r\n");
                (void) camera_settle_debug_light(debug_light_requested);
                continue;
            }
        }
        else
        {
            bool const task_light_requested =
                (FRUIT_UI_TASK_NONE != fruit_ui_get_task_mode()) &&
                !recognition_complete &&
                app_detection_get_top_camera_light_default();
            bool const light_changed = !g_camera_illumination_state_valid ||
                                       (task_light_requested !=
                                        g_camera_illumination_enabled);

            if (light_changed)
            {
                if (!camera_set_illumination(task_light_requested))
                {
                    vTaskDelay(pdMS_TO_TICKS(100U));
                    continue;
                }

                camera_uart_send_text(task_light_requested ?
                                      "CAM_LED TASK TOP ON\r\n" :
                                      "CAM_LED TASK TOP OFF\r\n");
                if (task_light_requested)
                {
                    vTaskDelay(pdMS_TO_TICKS(CAMERA_DEBUG_LIGHT_SETTLE_MS));
                    for (uint32_t frame = 0U;
                         frame < CAMERA_DEBUG_LIGHT_DISCARD_FRAMES;
                         frame++)
                    {
                        (void) camera_capture_frame_with_retry(g_camera_frame);
                        vTaskDelay(pdMS_TO_TICKS(CAMERA_CAPTURE_RETRY_MS));
                    }
                }
                continue;
            }
        }

        if ((FRUIT_UI_TASK_NONE == fruit_ui_get_task_mode()) &&
            !fruit_ui_is_debug_mode_active())
        {
            vTaskDelay(pdMS_TO_TICKS(100U));
            continue;
        }

        /*
         * A production recognition batch is single-shot.  After every item
         * from the first batch has been picked, keep the normal camera flow
         * idle instead of discovering the scene again.  Settings debug mode
         * may still capture and publish preview frames; leaving Settings
         * returns here and remains idle.
         */
        if (recognition_complete && !fruit_ui_is_debug_mode_active())
        {
            vTaskDelay(pdMS_TO_TICKS(100U));
            continue;
        }

        if (g_camera_calibration_active &&
            ((CAMERA_CALIBRATION_SEND_MOVE == g_camera_calibration_state) ||
             (CAMERA_CALIBRATION_WAIT_ARM == g_camera_calibration_state)))
        {
            vTaskDelay(pdMS_TO_TICKS(20U));
            continue;
        }

        if (CAMERA_OV5640_OK != camera_capture_frame_with_retry(g_camera_frame))
        {
            camera_uart_send_ceu_events_line();
            consecutive_capture_failures++;
            if (consecutive_capture_failures >= CAMERA_CAPTURE_RECOVERY_THRESHOLD)
            {
                bool const recovery_light = fruit_ui_is_debug_mode_active() ?
                    fruit_ui_debug_light_requested() :
                    (camera_side_active ? app_detection_get_side_camera_light_default() :
                                          app_detection_get_top_camera_light_default());

                camera_uart_send_text("CAM_CAPTURE_ERR full_recover\r\n");
                if (camera_full_recover_selected(camera_side_active, recovery_light))
                {
                    consecutive_capture_failures = 0U;
                }
                else
                {
                    vTaskDelay(pdMS_TO_TICKS(500U));
                }
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(10U));
            }
            continue;
        }
        consecutive_capture_failures = 0U;

        uint32_t result_count = 0U;
        bool const detection_ok = app_detection_run_frame(g_camera_frame,
                                                           g_detection_results,
                                                           APP_DETECTION_MAX_RESULTS,
                                                           &result_count,
                                                           camera_uart_send_text);

        if (!detection_ok)
        {
            camera_uart_send_text("DET_ERR run\r\n");
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

        if (fruit_ui_is_debug_mode_active())
        {
            camera_calibration_collect_frame(g_detection_results,
                                             result_count,
                                             camera_side_active);
            (void) camera_publish_snapshot(g_camera_frame,
                                           g_detection_results,
                                           result_count,
                                           camera_side_active,
                                           true);
            if (fruit_ui_take_frame_dump_request())
            {
                (void) camera_stream_send_frame_dump();
            }
            continue;
        }

        camera_record_top_samples(g_detection_results, result_count);

        TickType_t const task_preview_now = xTaskGetTickCount();
        if (!task_preview_started)
        {
            task_preview_start_tick = task_preview_now;
            task_preview_started = true;
        }

        if ((task_preview_now - last_task_preview_tick) >=
            pdMS_TO_TICKS(CAMERA_TASK_PREVIEW_PERIOD_MS))
        {
            if (camera_publish_snapshot(g_camera_frame,
                                        g_detection_results,
                                        result_count,
                                        false,
                                        false))
            {
                last_task_preview_tick = task_preview_now;
            }
        }

        if ((task_preview_now - task_preview_start_tick) <
            pdMS_TO_TICKS(CAMERA_TASK_PREVIEW_MIN_MS))
        {
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

        app_detection_result_t top_results[APP_DETECTION_MAX_RESULTS];
        uint32_t const top_result_count = camera_get_stable_top_results(top_results);

        if (0U == top_result_count)
        {
            camera_uart_send_text("TOP_DET_WAIT stable_samples\r\n");
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

        int32_t side_x[APP_DETECTION_MAX_RESULTS] = {0};
        int32_t side_y[APP_DETECTION_MAX_RESULTS] = {0};
        bool side_valid[APP_DETECTION_MAX_RESULTS] = {false};
        fruit_ui_detection_t paired_detections[FRUIT_UI_MAX_DETECTIONS];
        ipc_camera_coordinate_item_t selectable_items[FRUIT_UI_MAX_DETECTIONS];
        uint32_t paired_detection_count = 0U;
        fruit_ui_task_mode_t const task_mode = fruit_ui_get_task_mode();
        bool const task_top_light_enabled =
            app_detection_get_top_camera_light_default();
        bool const task_side_light_enabled =
            app_detection_get_side_camera_light_default();

        if (fruit_ui_is_debug_mode_active())
        {
            continue;
        }

        if (FRUIT_UI_TASK_TOP_ONLY == task_mode)
        {
            /* Task 1 no longer needs illumination after top detection. */
            (void) camera_set_illumination(false);

            for (uint32_t i = 0U;
                 (i < top_result_count) && (paired_detection_count < FRUIT_UI_MAX_DETECTIONS);
                 i++)
            {
                ipc_camera_coordinate_item_t const item =
                {
                    .class_id = top_results[i].class_id,
                    .top_x = top_results[i].x,
                    .top_y = top_results[i].y,
                    .side_x = 0,
                    .side_y = 0,
                };
                ipc_camera_coordinate_pair_t const pair =
                {
                    .pair_id = batch_id + 1U,
                    .class_id = item.class_id,
                    .top_x = item.top_x,
                    .top_y = item.top_y,
                    .side_x = item.side_x,
                    .side_y = item.side_y,
                };

                if (camera_pair_to_ui_detection(&pair,
                                                &paired_detections[paired_detection_count]))
                {
                    selectable_items[paired_detection_count] = item;
                    paired_detection_count++;
                }
            }
        }
        else if (FRUIT_UI_TASK_TOP_AND_SIDE == task_mode)
        {
            fruit_ui_set_task_camera(true);

            if (!camera_switch_to(true, task_side_light_enabled))
            {
                camera_uart_send_text("CAM_SWITCH_ERR side\r\n");
                (void) camera_switch_to(false, task_top_light_enabled);
                fruit_ui_set_task_camera(false);
                continue;
            }
            camera_side_active = true;

            /* Settings owns the camera as soon as debug mode becomes active. */
            if (fruit_ui_is_debug_mode_active())
            {
                continue;
            }

            /*
             * Settings continuously captures until the side sensor has a
             * usable image.  Give Task2 the same protection before its short,
             * fixed-size coordinate sampling window starts.
             */
            if (!camera_wait_for_task_side_preview())
            {
                if (fruit_ui_is_debug_mode_active())
                {
                    continue;
                }

                while (!camera_switch_to(false, task_top_light_enabled))
                {
                    vTaskDelay(pdMS_TO_TICKS(1000U));
                }
                camera_side_active = false;
                fruit_ui_set_task_camera(false);
                continue;
            }

            if (!camera_collect_side_samples(top_results,
                                             top_result_count,
                                             side_x,
                                             side_y,
                                             side_valid))
            {
                if (fruit_ui_is_debug_mode_active())
                {
                    continue;
                }

                while (!camera_switch_to(false, task_top_light_enabled))
                {
                    vTaskDelay(pdMS_TO_TICKS(1000U));
                }
                camera_side_active = false;
                fruit_ui_set_task_camera(false);
                continue;
            }

            if (fruit_ui_is_debug_mode_active())
            {
                continue;
            }

            ipc_camera_coordinate_batch_t batch =
            {
                .batch_id = batch_id + 1U,
                .count = 0U,
            };

            for (uint32_t target = 0U; target < top_result_count; target++)
            {
                if (!side_valid[target])
                {
                    continue;
                }

                ipc_camera_coordinate_item_t * p_item = &batch.items[batch.count];
                p_item->class_id = top_results[target].class_id;
                p_item->top_x = top_results[target].x;
                p_item->top_y = top_results[target].y;
                p_item->side_x = side_x[target];
                p_item->side_y = side_y[target];
                batch.count++;
            }

            for (uint32_t i = 0U; i < batch.count; i++)
            {
                ipc_camera_coordinate_item_t const * p_item = &batch.items[i];
                ipc_camera_coordinate_pair_t const pair =
                {
                    .pair_id = batch.batch_id,
                    .class_id = p_item->class_id,
                    .top_x = p_item->top_x,
                    .top_y = p_item->top_y,
                    .side_x = p_item->side_x,
                    .side_y = p_item->side_y,
                };

                if ((paired_detection_count < FRUIT_UI_MAX_DETECTIONS) &&
                    camera_pair_to_ui_detection(&pair, &paired_detections[paired_detection_count]))
                {
                    selectable_items[paired_detection_count] = *p_item;
                    paired_detection_count++;
                }
                else
                {
                    camera_uart_send_text("UI_COORD_ERR transform\r\n");
                }
            }

            if (fruit_ui_is_debug_mode_active())
            {
                continue;
            }

            if (!camera_switch_to(false, false))
            {
                camera_uart_send_text("CAM_SWITCH_ERR top\r\n");

                while (!camera_switch_to(false, false))
                {
                    vTaskDelay(pdMS_TO_TICKS(1000U));
                }
            }
            camera_side_active = false;
            fruit_ui_set_task_camera(false);
        }

        /* Do not publish a result from a task that HOME cancelled mid-frame. */
        if (fruit_ui_is_debug_mode_active() ||
            (task_mode != fruit_ui_get_task_mode()))
        {
            (void) camera_set_illumination(false);
            continue;
        }

        if (paired_detection_count > 0U)
        {
            (void) camera_set_illumination(false);
            fruit_ui_set_detections(paired_detections, paired_detection_count);
        }

        if (paired_detection_count > 0U)
        {
            bool item_sent[FRUIT_UI_MAX_DETECTIONS] = {false};
            uint32_t remaining_count = paired_detection_count;
            bool debug_interrupted = false;

            while (remaining_count > 0U)
            {
                if (fruit_ui_is_debug_mode_active())
                {
                    debug_interrupted = true;
                    break;
                }

                fruit_ui_target_t requested_target;

                if (!fruit_ui_take_pick_request(&requested_target))
                {
                    vTaskDelay(pdMS_TO_TICKS(10U));
                    continue;
                }

                uint32_t selected_index = paired_detection_count;
                for (uint32_t i = 0U; i < paired_detection_count; i++)
                {
                    if (!item_sent[i] &&
                        (camera_stream_target_from_class(selectable_items[i].class_id) == requested_target))
                    {
                        selected_index = i;
                        break;
                    }
                }

                if (selected_index == paired_detection_count)
                {
                    camera_uart_send_text("PICK_ERR target_not_found\r\n");
                    continue;
                }

                ipc_camera_coordinate_batch_t selected_batch =
                {
                    .batch_id = batch_id + 1U,
                    .count = 1U,
                    .items = {selectable_items[selected_index]},
                };

                while (!ipc_detection_send_coordinate_batch(&selected_batch))
                {
                    camera_uart_send_text("PICK_ERR ipc_send_retry\r\n");
                    vTaskDelay(pdMS_TO_TICKS(1000U));
                }

                fruit_ui_notify_pick_sent(requested_target);

                batch_id = selected_batch.batch_id;
                item_sent[selected_index] = true;
                remaining_count--;

                int const count = snprintf(g_uart_line,
                                           sizeof(g_uart_line),
                                           "PICK_IPC_SENT id=%lu class=%lu top=(%ld,%ld) side=(%ld,%ld)\r\n",
                                           (unsigned long) selected_batch.batch_id,
                                           (unsigned long) selectable_items[selected_index].class_id,
                                           (long) selectable_items[selected_index].top_x,
                                           (long) selectable_items[selected_index].top_y,
                                           (long) selectable_items[selected_index].side_x,
                                           (long) selectable_items[selected_index].side_y);
                if ((count > 0) && ((size_t) count < sizeof(g_uart_line)))
                {
                    camera_uart_send_text(g_uart_line);
                }
            }

            if (debug_interrupted)
            {
                continue;
            }

            recognition_complete = true;
            camera_uart_send_text("PICK_BATCH_COMPLETE detection_stopped\r\n");
        }
    }
}

static int32_t camera_round_mm(double value)
{
    return (int32_t) (value >= 0.0 ? value + 0.5 : value - 0.5);
}

static bool camera_project_top(double const homography[3][3],
                               double       u,
                               double       v,
                               double     * p_x_mm,
                               double     * p_y_mm)
{
    double const denominator = (homography[2][0] * u) +
                               (homography[2][1] * v) +
                                homography[2][2];

    if ((denominator > -CAMERA_HOMOGRAPHY_EPSILON) &&
        (denominator < CAMERA_HOMOGRAPHY_EPSILON))
    {
        return false;
    }

    *p_x_mm = ((homography[0][0] * u) +
               (homography[0][1] * v) +
                homography[0][2]) / denominator;
    *p_y_mm = ((homography[1][0] * u) +
               (homography[1][1] * v) +
                homography[1][2]) / denominator;
    return true;
}

static bool camera_top_pixel_to_arm_xy(double   u,
                                        double   v,
                                        double   z_mm,
                                        double * p_x_mm,
                                        double * p_y_mm)
{
    double x_low_mm;
    double y_low_mm;
    double x_high_mm;
    double y_high_mm;

    if (!camera_project_top(g_camera_to_arm_homography_z_low, u, v, &x_low_mm, &y_low_mm) ||
        !camera_project_top(g_camera_to_arm_homography_z_high, u, v, &x_high_mm, &y_high_mm))
    {
        return false;
    }

    double const ratio = (z_mm - g_camera_top_cal_z_low_mm) /
                         (g_camera_top_cal_z_high_mm - g_camera_top_cal_z_low_mm);
    *p_x_mm = x_low_mm + (ratio * (x_high_mm - x_low_mm));
    *p_y_mm = y_low_mm + (ratio * (y_high_mm - y_low_mm));
    return true;
}

static bool camera_pair_to_ui_detection(ipc_camera_coordinate_pair_t const * p_pair,
                                        fruit_ui_detection_t                * p_detection)
{
    bool const top_only = (NULL != p_pair) &&
                          (0 == p_pair->side_x) && (0 == p_pair->side_y);

    if ((NULL == p_pair) || (NULL == p_detection) ||
        (p_pair->top_x < 0) || (p_pair->top_x >= (int32_t) CAMERA_OV5640_WIDTH) ||
        (p_pair->top_y < 0) || (p_pair->top_y >= (int32_t) CAMERA_OV5640_HEIGHT) ||
        (!top_only && ((p_pair->side_y < CAMERA_SIDE_Y_MIN_PX) ||
                       (p_pair->side_x < 0) ||
                       (p_pair->side_x >= (int32_t) CAMERA_OV5640_WIDTH))))
    {
        return false;
    }

    fruit_ui_target_t const target = camera_stream_target_from_class(p_pair->class_id);
    if (FRUIT_UI_TARGET_NONE == target)
    {
        return false;
    }

    double const z_mm = top_only ? CAMERA_TOP_ONLY_Z_MM :
                       ((g_camera_side_z_slope_mm_px * (double) p_pair->side_x) +
                         g_camera_side_z_offset_mm);
    double x_mm;
    double y_mm;

    bool const xy_valid = top_only ?
                          camera_project_top(g_camera_to_arm_homography_z0,
                                             (double) p_pair->top_x,
                                             (double) p_pair->top_y,
                                             &x_mm,
                                             &y_mm) :
                          camera_top_pixel_to_arm_xy((double) p_pair->top_x,
                                                     (double) p_pair->top_y,
                                                     z_mm,
                                                     &x_mm,
                                                     &y_mm);

    if (!xy_valid)
    {
        return false;
    }

    p_detection->target = target;
    p_detection->x = camera_round_mm(x_mm);
    p_detection->y = camera_round_mm(y_mm);
    p_detection->z = camera_round_mm(z_mm);
    p_detection->x1 = -1;
    p_detection->y1 = -1;
    p_detection->x2 = -1;
    p_detection->y2 = -1;
    p_detection->mean_r = -1;
    p_detection->mean_g = -1;
    p_detection->mean_b = -1;
    p_detection->green_ratio_0p1 = -1;
    return true;
}
