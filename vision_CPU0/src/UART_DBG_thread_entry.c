#include <UART_DBG_thread.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "hardware/camera_ov5640.h"
#include "uart_debug.h"

#define UART_DEBUG_TEXT_BYTES             (192U)
#define UART_DEBUG_TEXT_QUEUE_LENGTH      (32U)
#define UART_DEBUG_CHUNK_BYTES            (1024U)
#define UART_DEBUG_TX_TIMEOUT_MS          (1000U)
#define UART_DEBUG_FRAME_HEADER_BYTES     (24U)
#define UART_DEBUG_FRAME_FORMAT_RGB565_LE (1U)

typedef struct st_uart_debug_text_message
{
    uint16_t length;
    uint8_t  data[UART_DEBUG_TEXT_BYTES];
} uart_debug_text_message_t;

extern TaskHandle_t UART_DBG_thread;

static uart_debug_text_message_t g_text_queue[UART_DEBUG_TEXT_QUEUE_LENGTH];
static volatile uint32_t g_text_queue_write;
static volatile uint32_t g_text_queue_read;
static volatile bool g_logs_enabled;
static uint8_t g_frame_snapshot[CAMERA_OV5640_FRAME_BYTES]
    BSP_PLACE_IN_SECTION(".sdram_nocache") BSP_ALIGN_VARIABLE(32);
static volatile bool g_frame_copy_reserved;
static volatile bool g_frame_dump_pending;
static volatile bool g_frame_dump_active;
static volatile bool g_uart_tx_busy;
static uart_callback_args_t g_uart_callback_memory;

static void uart_debug_notify_thread(void)
{
    if (NULL != UART_DBG_thread)
    {
        xTaskNotifyGive(UART_DBG_thread);
    }
}

bool uart_debug_send_text(char const * p_text)
{
    size_t length;
    uint32_t next;
    bool queued = false;
    bool logs_enabled;

    if (NULL == p_text)
    {
        return false;
    }

    length = strlen(p_text);
    if (0U == length)
    {
        return true;
    }
    if (length > UART_DEBUG_TEXT_BYTES)
    {
        return false;
    }

    taskENTER_CRITICAL();
    logs_enabled = g_logs_enabled;
    if (logs_enabled)
    {
        next = (g_text_queue_write + 1U) % UART_DEBUG_TEXT_QUEUE_LENGTH;
        if (next != g_text_queue_read)
        {
            uart_debug_text_message_t * p_message =
                &g_text_queue[g_text_queue_write];

            p_message->length = (uint16_t) length;
            memcpy(p_message->data, p_text, length);
            g_text_queue_write = next;
            queued = true;
        }
    }
    taskEXIT_CRITICAL();

    if (queued)
    {
        uart_debug_notify_thread();
    }
    /* Disabled logging intentionally discards text and is not an error. */
    return queued || !logs_enabled;
}

void uart_debug_set_logs_enabled(bool enabled)
{
    taskENTER_CRITICAL();
    g_logs_enabled = enabled;
    if (!enabled)
    {
        g_text_queue_read = g_text_queue_write;
    }
    taskEXIT_CRITICAL();
}

bool uart_debug_logs_enabled(void)
{
    return g_logs_enabled;
}

bool uart_debug_request_frame_dump(uint8_t const * p_rgb565_frame,
                                   uint32_t        frame_bytes)
{
    bool reserved = false;

    if ((NULL == p_rgb565_frame) ||
        (CAMERA_OV5640_FRAME_BYTES != frame_bytes))
    {
        return false;
    }

    taskENTER_CRITICAL();
    if (!g_frame_copy_reserved && !g_frame_dump_pending &&
        !g_frame_dump_active)
    {
        g_frame_copy_reserved = true;
        reserved = true;
    }
    taskEXIT_CRITICAL();

    if (!reserved)
    {
        return false;
    }

    memcpy(g_frame_snapshot, p_rgb565_frame, CAMERA_OV5640_FRAME_BYTES);

    taskENTER_CRITICAL();
    g_frame_copy_reserved = false;
    g_frame_dump_pending = true;
    taskEXIT_CRITICAL();
    uart_debug_notify_thread();
    return true;
}

static void uart_debug_callback(uart_callback_args_t * p_args)
{
    if ((NULL != p_args) && (UART_EVENT_TX_COMPLETE == p_args->event))
    {
        g_uart_tx_busy = false;
    }
}

static bool uart_debug_write(uint8_t const * p_data, uint32_t bytes)
{
    TickType_t const start = xTaskGetTickCount();

    if ((NULL == p_data) || (0U == bytes))
    {
        return false;
    }

    g_uart_tx_busy = true;
    if (FSP_SUCCESS != g_uart9.p_api->write(g_uart9.p_ctrl, p_data, bytes))
    {
        g_uart_tx_busy = false;
        return false;
    }

    while (g_uart_tx_busy)
    {
        if ((xTaskGetTickCount() - start) >=
            pdMS_TO_TICKS(UART_DEBUG_TX_TIMEOUT_MS))
        {
            (void) g_uart9.p_api->communicationAbort(g_uart9.p_ctrl,
                                                     UART_DIR_TX);
            g_uart_tx_busy = false;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return true;
}

static bool uart_debug_send_bytes(uint8_t const * p_data, uint32_t bytes)
{
    uint32_t sent = 0U;

    while (sent < bytes)
    {
        uint32_t const remaining = bytes - sent;
        uint32_t const chunk =
            (remaining > UART_DEBUG_CHUNK_BYTES) ?
            UART_DEBUG_CHUNK_BYTES : remaining;

        if (!uart_debug_write(&p_data[sent], chunk))
        {
            return false;
        }
        sent += chunk;
    }
    return true;
}

static bool uart_debug_take_text(uart_debug_text_message_t * p_message)
{
    bool available = false;

    if (NULL == p_message)
    {
        return false;
    }

    taskENTER_CRITICAL();
    if (g_text_queue_read != g_text_queue_write)
    {
        *p_message = g_text_queue[g_text_queue_read];
        g_text_queue_read = (g_text_queue_read + 1U) %
                            UART_DEBUG_TEXT_QUEUE_LENGTH;
        available = true;
    }
    taskEXIT_CRITICAL();
    return available;
}

static bool uart_debug_take_frame_dump(void)
{
    bool pending;

    taskENTER_CRITICAL();
    pending = g_frame_dump_pending;
    if (pending)
    {
        g_frame_dump_pending = false;
        g_frame_dump_active = true;
    }
    taskEXIT_CRITICAL();
    return pending;
}

static uint32_t uart_debug_crc32(uint8_t const * p_data, uint32_t bytes)
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

static void uart_debug_store_u16_le(uint8_t * p_dst, uint16_t value)
{
    p_dst[0] = (uint8_t) value;
    p_dst[1] = (uint8_t) (value >> 8U);
}

static void uart_debug_store_u32_le(uint8_t * p_dst, uint32_t value)
{
    p_dst[0] = (uint8_t) value;
    p_dst[1] = (uint8_t) (value >> 8U);
    p_dst[2] = (uint8_t) (value >> 16U);
    p_dst[3] = (uint8_t) (value >> 24U);
}

static bool uart_debug_send_frame_dump(void)
{
    static uint8_t const magic[8] =
        {'C', 'E', 'U', '5', '6', '5', '0', '1'};
    static char const begin[] =
        "FRAME_DUMP_BEGIN RGB565LE 640x480 wait_about_55s\r\n";
    uint8_t header[UART_DEBUG_FRAME_HEADER_BYTES];
    char end[64];
    uint32_t const crc =
        uart_debug_crc32(g_frame_snapshot, CAMERA_OV5640_FRAME_BYTES);

    memcpy(header, magic, sizeof(magic));
    uart_debug_store_u16_le(&header[8], (uint16_t) CAMERA_OV5640_WIDTH);
    uart_debug_store_u16_le(&header[10], (uint16_t) CAMERA_OV5640_HEIGHT);
    uart_debug_store_u32_le(&header[12], UART_DEBUG_FRAME_FORMAT_RGB565_LE);
    uart_debug_store_u32_le(&header[16], CAMERA_OV5640_FRAME_BYTES);
    uart_debug_store_u32_le(&header[20], crc);

    if (!uart_debug_write((uint8_t const *) begin,
                          (uint32_t) (sizeof(begin) - 1U)) ||
        !uart_debug_send_bytes(header, sizeof(header)) ||
        !uart_debug_send_bytes(g_frame_snapshot, CAMERA_OV5640_FRAME_BYTES))
    {
        static char const error[] = "FRAME_DUMP_ERROR\r\n";
        (void) uart_debug_write((uint8_t const *) error,
                                (uint32_t) (sizeof(error) - 1U));
        return false;
    }

    int const count = snprintf(end, sizeof(end),
                               "FRAME_DUMP_END bytes=%lu crc32=0x%08lX\r\n",
                               (unsigned long) CAMERA_OV5640_FRAME_BYTES,
                               (unsigned long) crc);
    return (count > 0) && ((size_t) count < sizeof(end)) &&
           uart_debug_write((uint8_t const *) end, (uint32_t) count);
}

/* UART debug thread entry function. */
void UART_DBG_thread_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    fsp_err_t const open_err =
        g_uart9.p_api->open(g_uart9.p_ctrl, g_uart9.p_cfg);
    if ((FSP_SUCCESS != open_err) && (FSP_ERR_ALREADY_OPEN != open_err))
    {
        vTaskDelete(NULL);
    }

    if (FSP_SUCCESS != g_uart9.p_api->callbackSet(g_uart9.p_ctrl,
                                                  uart_debug_callback,
                                                  NULL,
                                                  &g_uart_callback_memory))
    {
        vTaskDelete(NULL);
    }

    while (1)
    {
        uart_debug_text_message_t message;

        (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1)
        {
            /* Give a requested binary frame priority over queued text. Text
             * produced during the long dump remains queued and cannot be
             * interleaved into the framed RGB565 payload. */
            if (uart_debug_take_frame_dump())
            {
                (void) uart_debug_send_frame_dump();
                taskENTER_CRITICAL();
                g_frame_dump_active = false;
                taskEXIT_CRITICAL();
                continue;
            }

            if (!uart_debug_take_text(&message))
            {
                break;
            }
            (void) uart_debug_write(message.data, message.length);
        }
    }
}
