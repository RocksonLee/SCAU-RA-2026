#include "camera_stream.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "app_detection.h"
#include "camera_ov5640.h"
#include "fruit_ui.h"
#include "hal_data.h"
#include "ipc_detection_tx.h"

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
#define CAMERA_SIDE_WARMUP_FRAMES (5U)
#define CAMERA_PREVIEW_PERIOD_MS (500U)
#define CAMERA_SIDE_SAMPLES      (5U)
#define CAMERA_SIDE_MAX_FRAMES   (90U)
#define CAMERA_SIDE_Y_MIN_PX     (50)
#define CAMERA_SIDE_CAL_X_MIN_PX (0)
#define CAMERA_SIDE_CAL_X_MAX_PX (640)
#define CAMERA_SIDE_Z_SLOPE      (0.5631433347759307)
#define CAMERA_SIDE_Z_OFFSET     (278.6319722003004)
#define CAMERA_TOP_ONLY_Z_MM     (280.0)
#define CAMERA_HOMOGRAPHY_EPSILON (1.0e-9)

/* Keep these coefficients synchronized with CPU1 handeye_transform.c. */
static double const g_camera_to_arm_homography[3][3] =
{
    {0.038658,  0.386610, 54.505698},
    {0.548603, -0.045075, 62.271670},
    {0.000383, -0.000106,  1.000000},
};

#if APP_DETECTION_MAX_RESULTS != FRUIT_UI_MAX_DETECTIONS
 #error "Detection result capacity does not match the UI capacity."
#endif

static uint8_t g_camera_frame[CAMERA_OV5640_FRAME_BYTES] BSP_PLACE_IN_SECTION(".sdram_nocache") BSP_ALIGN_VARIABLE(32);

static volatile bool g_uart_tx_busy;
static uart_callback_args_t g_uart_callback_memory;
static char g_uart_line[CAMERA_UART_LINE_BYTES];
static app_detection_result_t g_detection_results[APP_DETECTION_MAX_RESULTS];

static bool camera_pair_to_ui_detection(ipc_camera_coordinate_pair_t const * p_pair,
                                        fruit_ui_detection_t                * p_detection);

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

static int32_t camera_median_5(int32_t const samples[CAMERA_SIDE_SAMPLES])
{
    int32_t sorted[CAMERA_SIDE_SAMPLES];

    for (uint32_t i = 0U; i < CAMERA_SIDE_SAMPLES; i++)
    {
        sorted[i] = samples[i];
    }

    for (uint32_t i = 1U; i < CAMERA_SIDE_SAMPLES; i++)
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

    return sorted[CAMERA_SIDE_SAMPLES / 2U];
}

static void camera_discard_settle_frames(void)
{
    vTaskDelay(pdMS_TO_TICKS(CAMERA_SWITCH_SETTLE_MS));

    for (uint32_t i = 0U; i < CAMERA_DISCARD_FRAMES; i++)
    {
        (void) camera_capture_frame_with_retry(g_camera_frame);
        vTaskDelay(pdMS_TO_TICKS(100U));
    }
}

static void camera_warmup_side_camera(void)
{
    vTaskDelay(pdMS_TO_TICKS(CAMERA_SIDE_SWITCH_SETTLE_MS));

    for (uint32_t frame = 0U; frame < CAMERA_SIDE_WARMUP_FRAMES; frame++)
    {
        (void) camera_capture_frame_with_retry(g_camera_frame);
        vTaskDelay(pdMS_TO_TICKS(CAMERA_CAPTURE_RETRY_MS));
    }
}

static bool camera_switch_to(bool side_camera)
{
    camera_ov5640_result_t const stop_result = camera_ov5640_stop();

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

    vTaskDelay(pdMS_TO_TICKS(side_camera ? CAMERA_SIDE_MUX_SETTLE_MS : CAMERA_MUX_SETTLE_MS));

    if (!side_camera)
    {
        camera_ov5640_result_t const init_result = camera_ov5640_init();
        if (CAMERA_OV5640_OK != init_result)
        {
            camera_uart_send_camera_init_error(init_result);
            return false;
        }

        camera_discard_settle_frames();
        camera_uart_send_text("CAM_ACTIVE TOP\r\n");
        return true;
    }

    /* Preserve the 508cd09 power-cycle -> mux -> full OV5640 init order. */
    camera_ov5640_result_t const init_result = camera_ov5640_init();
    if (CAMERA_OV5640_OK != init_result)
    {
        camera_uart_send_camera_init_error(init_result);
        return false;
    }

    /* Bad warm-up frames are discarded, but they do not block recognition. */
    camera_warmup_side_camera();
    camera_uart_send_text("CAM_ACTIVE SIDE\r\n");
    return true;
}

static bool camera_publish_debug_snapshot(uint8_t const                 * p_rgb565_frame,
                                          app_detection_result_t const * p_results,
                                          uint32_t                       result_count)
{
    fruit_ui_detection_t detections[FRUIT_UI_MAX_DETECTIONS];
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
        detection_count++;
    }

    return fruit_ui_publish_debug_snapshot(p_rgb565_frame, detections, detection_count);
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

        bool sampled_this_frame[APP_DETECTION_MAX_RESULTS] = {false};
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

            for (uint32_t target = 0U; target < target_count; target++)
            {
                if (!sampled_this_frame[target] &&
                    (sample_count[target] < CAMERA_SIDE_SAMPLES) &&
                    (g_detection_results[i].class_id == p_top_results[target].class_id))
                {
                    uint32_t const sample = sample_count[target];
                    x_samples[target][sample] = g_detection_results[i].x;
                    y_samples[target][sample] = g_detection_results[i].y;
                    sample_count[target]++;
                    sampled_this_frame[target] = true;
                    break;
                }
            }
        }
    }

    if (dropped_frame_count > 0U)
    {
        int const count = snprintf(g_uart_line,
                                   sizeof(g_uart_line),
                                   "SIDE_CAPTURE valid=%lu dropped=%lu\r\n",
                                   (unsigned long) valid_frame_count,
                                   (unsigned long) dropped_frame_count);
        if ((count > 0) && ((size_t) count < sizeof(g_uart_line)))
        {
            camera_uart_send_text(g_uart_line);
        }
    }

    for (uint32_t target = 0U; target < target_count; target++)
    {
        if (CAMERA_SIDE_SAMPLES == sample_count[target])
        {
            p_side_x[target] = camera_median_5(x_samples[target]);
            p_side_y[target] = camera_median_5(y_samples[target]);
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
    TickType_t last_preview_tick = 0U;
    bool recognition_complete = false;

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

    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, CAMERA_MUX_SEL, BSP_IO_LEVEL_HIGH);
    vTaskDelay(pdMS_TO_TICKS(CAMERA_MUX_SETTLE_MS));

    camera_ov5640_result_t const camera_init_result = camera_ov5640_init();
    if (CAMERA_OV5640_OK != camera_init_result)
    {
        camera_uart_send_camera_init_error(camera_init_result);

        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }

    if (!app_detection_init())
    {
        camera_uart_send_text("DET_ERR init\r\n");

        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }

    camera_uart_send_ov5640_diagnostics();

    vTaskDelay(pdMS_TO_TICKS(CAMERA_AE_SETTLE_MS));

    for (uint32_t i = 0U; i < CAMERA_DISCARD_FRAMES; i++)
    {
        (void) camera_capture_frame_with_retry(g_camera_frame);
        vTaskDelay(pdMS_TO_TICKS(100U));
    }

    while (1)
    {
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

        if (CAMERA_OV5640_OK != camera_capture_frame_with_retry(g_camera_frame))
        {
            camera_uart_send_ceu_events_line();
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

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
            TickType_t const now = xTaskGetTickCount();
            if ((now - last_preview_tick) >= pdMS_TO_TICKS(CAMERA_PREVIEW_PERIOD_MS))
            {
                if (camera_publish_debug_snapshot(g_camera_frame,
                                                  g_detection_results,
                                                  result_count))
                {
                    last_preview_tick = now;
                }
            }
            if (fruit_ui_take_frame_dump_request())
            {
                (void) camera_stream_send_frame_dump();
            }
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

        if (0U == result_count)
        {
            camera_uart_send_text("FRAME_OK NO_DET\r\n");
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

        app_detection_result_t top_results[APP_DETECTION_MAX_RESULTS];
        uint32_t top_result_count = 0U;

        for (uint32_t i = 0U; i < result_count; i++)
        {
            if (FRUIT_UI_TARGET_NONE == camera_stream_target_from_class(g_detection_results[i].class_id))
            {
                continue;
            }

            bool class_already_added = false;
            for (uint32_t saved = 0U; saved < top_result_count; saved++)
            {
                if (top_results[saved].class_id == g_detection_results[i].class_id)
                {
                    class_already_added = true;
                    break;
                }
            }

            if (!class_already_added)
            {
                top_results[top_result_count] = g_detection_results[i];
                top_result_count++;
            }
        }

        if (0U == top_result_count)
        {
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

        if (fruit_ui_is_debug_mode_active())
        {
            continue;
        }

        if (FRUIT_UI_TASK_TOP_ONLY == task_mode)
        {
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
            if (!camera_switch_to(true))
            {
                camera_uart_send_text("CAM_SWITCH_ERR side\r\n");
                (void) camera_switch_to(false);
                continue;
            }

            if (!camera_collect_side_samples(top_results,
                                             top_result_count,
                                             side_x,
                                             side_y,
                                             side_valid))
            {
                while (!camera_switch_to(false))
                {
                    vTaskDelay(pdMS_TO_TICKS(1000U));
                }
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

            if (!camera_switch_to(false))
            {
                camera_uart_send_text("CAM_SWITCH_ERR top\r\n");

                while (!camera_switch_to(false))
                {
                    vTaskDelay(pdMS_TO_TICKS(1000U));
                }
            }
        }

        if (paired_detection_count > 0U)
        {
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

static bool camera_pair_to_ui_detection(ipc_camera_coordinate_pair_t const * p_pair,
                                        fruit_ui_detection_t                * p_detection)
{
    bool const top_only = (NULL != p_pair) &&
                          (0 == p_pair->side_x) && (0 == p_pair->side_y);

    if ((NULL == p_pair) || (NULL == p_detection) ||
        (p_pair->top_x < 0) || (p_pair->top_x >= (int32_t) CAMERA_OV5640_WIDTH) ||
        (p_pair->top_y < 0) || (p_pair->top_y >= (int32_t) CAMERA_OV5640_HEIGHT) ||
        (!top_only && ((p_pair->side_y < CAMERA_SIDE_Y_MIN_PX) ||
                       (p_pair->side_x < CAMERA_SIDE_CAL_X_MIN_PX) ||
                       (p_pair->side_x > CAMERA_SIDE_CAL_X_MAX_PX))))
    {
        return false;
    }

    fruit_ui_target_t const target = camera_stream_target_from_class(p_pair->class_id);
    if (FRUIT_UI_TARGET_NONE == target)
    {
        return false;
    }

    double const u = (double) p_pair->top_x;
    double const v = (double) p_pair->top_y;
    double const denominator = (g_camera_to_arm_homography[2][0] * u) +
                               (g_camera_to_arm_homography[2][1] * v) +
                                g_camera_to_arm_homography[2][2];

    if ((denominator > -CAMERA_HOMOGRAPHY_EPSILON) &&
        (denominator < CAMERA_HOMOGRAPHY_EPSILON))
    {
        return false;
    }

    double const x_mm = ((g_camera_to_arm_homography[0][0] * u) +
                         (g_camera_to_arm_homography[0][1] * v) +
                          g_camera_to_arm_homography[0][2]) / denominator;
    double const y_mm = ((g_camera_to_arm_homography[1][0] * u) +
                         (g_camera_to_arm_homography[1][1] * v) +
                          g_camera_to_arm_homography[1][2]) / denominator;
    double const z_mm = top_only ? CAMERA_TOP_ONLY_Z_MM :
                       ((CAMERA_SIDE_Z_SLOPE * (double) p_pair->side_x) +
                         CAMERA_SIDE_Z_OFFSET);

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
