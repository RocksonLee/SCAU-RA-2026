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
#define CAMERA_AE_SETTLE_MS     (1500U)
#define CAMERA_DISCARD_FRAMES   (5U)
#define CAMERA_UART_LINE_BYTES  (192U)
#define CAMERA_CAPTURE_ATTEMPTS (3U)
#define CAMERA_CAPTURE_RETRY_MS (100U)

#if APP_DETECTION_MAX_RESULTS != FRUIT_UI_MAX_DETECTIONS
 #error "Detection result capacity does not match the UI capacity."
#endif

static uint8_t g_camera_frame[CAMERA_OV5640_FRAME_BYTES] BSP_PLACE_IN_SECTION(".sdram_nocache") BSP_ALIGN_VARIABLE(32);

static volatile bool g_uart_tx_busy;
static uart_callback_args_t g_uart_callback_memory;
static char g_uart_line[CAMERA_UART_LINE_BYTES];
static app_detection_result_t g_detection_results[APP_DETECTION_MAX_RESULTS];

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

static bool camera_stream_send_frame(void)
{
    return camera_uart_send_bytes(g_camera_frame, CAMERA_OV5640_FRAME_BYTES);
}

void camera_stream_task(void)
{
    bool detection_sent = true;

    if (camera_debug_uart_init())
    {
        camera_uart_send_text("FW=CPU0_CEU_V3\r\n");
    }

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

    while (detection_sent)
    {
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

        fruit_ui_detection_t ui_detections[FRUIT_UI_MAX_DETECTIONS];
        uint32_t ui_detection_count = 0U;

        for (uint32_t i = 0U; i < result_count; i++)
        {
            fruit_ui_target_t const target = camera_stream_target_from_class(g_detection_results[i].class_id);

            if (target != FRUIT_UI_TARGET_NONE)
            {
                ui_detections[ui_detection_count].target = target;
                ui_detections[ui_detection_count].x = g_detection_results[i].x;
                ui_detections[ui_detection_count].y = g_detection_results[i].y;
                ui_detections[ui_detection_count].z = 0;
                ui_detection_count++;
            }
        }

        fruit_ui_set_detections(ui_detections, ui_detection_count);

        if (0U == result_count)
        {
            camera_uart_send_text("FRAME_OK NO_DET\r\n");
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }

        if (ipc_detection_send_results(g_detection_results, result_count))
        {
            detection_sent = true;

            int const count = snprintf(g_uart_line,
                                       sizeof(g_uart_line),
                                       "DET_IPC_SENT count=%lu\r\n",
                                       (unsigned long) result_count);

            if ((count > 0) && ((size_t) count < sizeof(g_uart_line)))
            {
                camera_uart_send_text(g_uart_line);
            }
        }
        else
        {
            camera_uart_send_text("DET_ERR ipc_send\r\n");
            vTaskDelay(pdMS_TO_TICKS(10U));
        }
    }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
