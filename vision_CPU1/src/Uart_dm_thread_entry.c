#include "Uart_dm_thread.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ipc_detection_rx.h"
#include "uart_coordinate_protocol.h"

#define DM_UART_TX_TIMEOUT_MS (20U)

#if DM_COORDINATE_UART_ENABLE
static volatile bool g_uart_tx_complete;
static uint8_t g_uart_coordinate_frame[DM_UART_COORDINATE_FRAME_SIZE];

static void dm_uart_put_u32_le(uint8_t * p_dest, uint32_t value)
{
    p_dest[0] = (uint8_t) value;
    p_dest[1] = (uint8_t) (value >> 8);
    p_dest[2] = (uint8_t) (value >> 16);
    p_dest[3] = (uint8_t) (value >> 24);
}

static uint16_t dm_uart_crc16_ccitt_false(uint8_t const * p_data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint32_t i = 0U; i < length; i++)
    {
        crc ^= (uint16_t) ((uint16_t) p_data[i] << 8);

        for (uint32_t bit = 0U; bit < 8U; bit++)
        {
            uint16_t const shifted = (uint16_t) (crc << 1);
            crc = (0U != (crc & 0x8000U)) ?
                  (uint16_t) (shifted ^ (uint16_t) 0x1021U) : shifted;
        }
    }

    return crc;
}

static bool dm_uart_coordinate_to_fixed(double coordinate_mm, int32_t * p_fixed)
{
    if ((NULL == p_fixed) || !isfinite(coordinate_mm))
    {
        return false;
    }

    double const scaled = coordinate_mm * DM_UART_COORDINATE_SCALE_PER_MM;
    if ((scaled > (double) INT32_MAX) || (scaled < (double) INT32_MIN))
    {
        return false;
    }

    *p_fixed = (int32_t) ((scaled >= 0.0) ? (scaled + 0.5) : (scaled - 0.5));
    return true;
}

static bool dm_uart_build_coordinate_frame(handeye_arm_point_t const * p_point,
                                           uint32_t                    pair_id,
                                           uint32_t                    class_id)
{
    int32_t x_fixed;
    int32_t y_fixed;
    int32_t z_fixed;

    if ((NULL == p_point) || (class_id > UINT8_MAX) ||
        !dm_uart_coordinate_to_fixed(p_point->x_mm, &x_fixed) ||
        !dm_uart_coordinate_to_fixed(p_point->y_mm, &y_fixed) ||
        !dm_uart_coordinate_to_fixed(p_point->z_mm, &z_fixed))
    {
        return false;
    }

    g_uart_coordinate_frame[DM_UART_OFFSET_SOF0] = DM_UART_FRAME_SOF0;
    g_uart_coordinate_frame[DM_UART_OFFSET_SOF1] = DM_UART_FRAME_SOF1;
    g_uart_coordinate_frame[DM_UART_OFFSET_VERSION] = DM_UART_PROTOCOL_VERSION;
    g_uart_coordinate_frame[DM_UART_OFFSET_MESSAGE_TYPE] = DM_UART_MESSAGE_COORDINATE;
    dm_uart_put_u32_le(&g_uart_coordinate_frame[DM_UART_OFFSET_PAIR_ID], pair_id);
    g_uart_coordinate_frame[DM_UART_OFFSET_CLASS_ID] = (uint8_t) class_id;
    g_uart_coordinate_frame[DM_UART_OFFSET_FLAGS] = 0U;
    dm_uart_put_u32_le(&g_uart_coordinate_frame[DM_UART_OFFSET_X], (uint32_t) x_fixed);
    dm_uart_put_u32_le(&g_uart_coordinate_frame[DM_UART_OFFSET_Y], (uint32_t) y_fixed);
    dm_uart_put_u32_le(&g_uart_coordinate_frame[DM_UART_OFFSET_Z], (uint32_t) z_fixed);

    uint16_t const crc = dm_uart_crc16_ccitt_false(g_uart_coordinate_frame,
                                                    DM_UART_COORDINATE_CRC_OFFSET);
    g_uart_coordinate_frame[DM_UART_COORDINATE_CRC_OFFSET] = (uint8_t) crc;
    g_uart_coordinate_frame[DM_UART_COORDINATE_CRC_OFFSET + 1U] = (uint8_t) (crc >> 8);
    return true;
}

static bool dm_uart_send_coordinate(handeye_arm_point_t const * p_point,
                                    uint32_t                    pair_id,
                                    uint32_t                    class_id)
{
    if (!dm_uart_build_coordinate_frame(p_point, pair_id, class_id))
    {
        return false;
    }

    g_uart_tx_complete = false;
    if (FSP_SUCCESS != g_uart0.p_api->write(g_uart0.p_ctrl,
                                            g_uart_coordinate_frame,
                                            DM_UART_COORDINATE_FRAME_SIZE))
    {
        return false;
    }

    TickType_t const start = xTaskGetTickCount();
    while (!g_uart_tx_complete)
    {
        if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(DM_UART_TX_TIMEOUT_MS))
        {
            (void) g_uart0.p_api->communicationAbort(g_uart0.p_ctrl, UART_DIR_TX);
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    return true;
}
#endif

void uart0_callback(uart_callback_args_t * p_args)
{
#if DM_COORDINATE_UART_ENABLE
    if ((NULL != p_args) && (UART_EVENT_TX_COMPLETE == p_args->event))
    {
        g_uart_tx_complete = true;
    }
#else
    FSP_PARAMETER_NOT_USED(p_args);
#endif
}

/* Uart_dm_thread entry function */
/* pvParameters contains TaskHandle_t */
void Uart_dm_thread_entry(void *pvParameters) {
	FSP_PARAMETER_NOT_USED(pvParameters);

#if DM_COORDINATE_UART_ENABLE
    if (FSP_SUCCESS != g_uart0.p_api->open(g_uart0.p_ctrl, g_uart0.p_cfg))
    {
        vTaskDelete(NULL);
    }

    while (1)
    {
        (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        handeye_arm_point_t arm_point;
        uint32_t pair_id;
        uint32_t class_id;

        while (ipc_arm_point_take(&arm_point, &pair_id, &class_id))
        {
            (void) dm_uart_send_coordinate(&arm_point, pair_id, class_id);
        }
    }
#else
    vTaskSuspend(NULL);
    while (1)
    {
    }
#endif
}
