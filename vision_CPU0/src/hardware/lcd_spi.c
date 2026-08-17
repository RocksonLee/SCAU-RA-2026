#include "lcd_spi.h"
#include "FreeRTOS.h"
#include "task.h"

#define LCD_SPI_MAX_PIXELS_PER_TRANSFER (480U * 20U)
#define LCD_SPI_BYTES_PER_PIXEL         (3U)
#define LCD_SPI_WAIT_TIMEOUT_MS         (1000U)
/* MX + MV + BGR: landscape orientation, rotated 180 degrees from 0xA8. */
#define LCD_MADCTL_LANDSCAPE_180         (0x68U)

static uint8_t g_lcd_spi_tx_buffer[LCD_SPI_MAX_PIXELS_PER_TRANSFER *
                                    LCD_SPI_BYTES_PER_PIXEL] BSP_ALIGN_VARIABLE(32);
static TaskHandle_t volatile g_lcd_spi_wait_task;
static bool g_lcd_spi_is_open;
static volatile bool g_lcd_spi_transfer_pending;
static volatile bool g_lcd_spi_release_cs_on_complete;
static lcd_spi_async_callback_t volatile g_lcd_spi_async_callback;
static void * volatile g_lcd_spi_async_context;

volatile fsp_err_t   g_lcd_spi_last_error = FSP_SUCCESS;
volatile spi_event_t g_lcd_spi_last_event;

void lcd_spi_callback(spi_callback_args_t * p_args)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    lcd_spi_async_callback_t async_callback;
    void * async_context;

    if (NULL == p_args)
    {
        return;
    }

    g_lcd_spi_last_event = p_args->event;

    if (g_lcd_spi_release_cs_on_complete)
    {
        cs_high();
        g_lcd_spi_release_cs_on_complete = false;
    }

    async_callback = g_lcd_spi_async_callback;
    async_context = (void *) g_lcd_spi_async_context;
    g_lcd_spi_async_callback = NULL;
    g_lcd_spi_async_context = NULL;

    if (NULL != async_callback)
    {
        /* The LVGL completion hook only clears an IRQ-safe volatile flag. */
        async_callback(async_context);
    }

    if (NULL != g_lcd_spi_wait_task)
    {
        vTaskNotifyGiveFromISR(g_lcd_spi_wait_task, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

static bool lcd_spi_wait_for_transfer(void)
{
    spi_event_t event;

    if (!g_lcd_spi_transfer_pending)
    {
        return true;
    }

    if ((spi_event_t) 0 == g_lcd_spi_last_event)
    {
        if (0U == ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(LCD_SPI_WAIT_TIMEOUT_MS)))
        {
            g_lcd_spi_last_error = FSP_ERR_TIMEOUT;
            g_lcd_spi_wait_task = NULL;
            g_lcd_spi_transfer_pending = false;
            g_lcd_spi_release_cs_on_complete = false;
            g_lcd_spi_async_callback = NULL;
            g_lcd_spi_async_context = NULL;
            cs_high();
            return false;
        }
    }
    else
    {
        /* Discard the completion notification if the ISR ran before this wait. */
        (void) ulTaskNotifyTake(pdTRUE, 0U);
    }

    event = g_lcd_spi_last_event;
    g_lcd_spi_wait_task = NULL;
    g_lcd_spi_transfer_pending = false;

    if (SPI_EVENT_TRANSFER_COMPLETE != event)
    {
        g_lcd_spi_last_error = FSP_ERR_INTERNAL;
        return false;
    }

    return true;
}

static bool lcd_spi_start_write(const uint8_t          * data,
                                uint32_t                 length,
                                bool                     release_cs_on_complete,
                                lcd_spi_async_callback_t p_callback,
                                void                   * p_context)
{
    if ((!g_lcd_spi_is_open) || (NULL == data) || (0U == length))
    {
        return false;
    }

    if (length > UINT16_MAX)
    {
        g_lcd_spi_last_error = FSP_ERR_INVALID_SIZE;
        return false;
    }

    if (!lcd_spi_wait_for_transfer())
    {
        return false;
    }

#if BSP_CFG_DCACHE_ENABLED
    SCB_CleanDCache_by_Addr((uint32_t *) data, (int32_t) length);
#endif

    g_lcd_spi_wait_task = xTaskGetCurrentTaskHandle();
    (void) ulTaskNotifyTake(pdTRUE, 0U);
    g_lcd_spi_last_event = (spi_event_t) 0;
    g_lcd_spi_transfer_pending = true;
    g_lcd_spi_release_cs_on_complete = release_cs_on_complete;
    g_lcd_spi_async_callback = p_callback;
    g_lcd_spi_async_context = p_context;

    g_lcd_spi_last_error = g_spi_lcd.p_api->write(g_spi_lcd.p_ctrl,
                                                   data,
                                                   length,
                                                   SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != g_lcd_spi_last_error)
    {
        g_lcd_spi_wait_task = NULL;
        g_lcd_spi_transfer_pending = false;
        g_lcd_spi_release_cs_on_complete = false;
        g_lcd_spi_async_callback = NULL;
        g_lcd_spi_async_context = NULL;
        return false;
    }

    return true;
}

bool lcd_spi_write(const uint8_t * data, uint32_t length)
{
    return lcd_spi_start_write(data, length, false, NULL, NULL) &&
           lcd_spi_wait_for_transfer();
}

void spi_write_byte(uint8_t data)
{
    (void) lcd_spi_write(&data, 1U);
}

static void lcd_convert_rgb565_to_rgb666(const uint16_t * pixels, uint32_t pixel_count)
{
    for (uint32_t i = 0U; i < pixel_count; i++)
    {
        uint16_t color = pixels[i];
        uint8_t r5 = (uint8_t) ((color >> 11U) & 0x1FU);
        uint8_t g6 = (uint8_t) ((color >> 5U)  & 0x3FU);
        uint8_t b5 = (uint8_t) (color & 0x1FU);

        g_lcd_spi_tx_buffer[(i * 3U) + 0U] = (uint8_t) ((r5 << 3U) | (r5 >> 2U));
        g_lcd_spi_tx_buffer[(i * 3U) + 1U] = (uint8_t) ((g6 << 2U) | (g6 >> 4U));
        g_lcd_spi_tx_buffer[(i * 3U) + 2U] = (uint8_t) ((b5 << 3U) | (b5 >> 2U));
    }
}

bool lcd_write_pixels_rgb565_async(const uint16_t         * pixels,
                                   uint32_t                 pixel_count,
                                   lcd_spi_async_callback_t p_callback,
                                   void                     * p_context)
{
    if ((NULL == pixels) || (0U == pixel_count) ||
        (pixel_count > LCD_SPI_MAX_PIXELS_PER_TRANSFER))
    {
        return false;
    }

    if (!lcd_spi_wait_for_transfer())
    {
        return false;
    }

    lcd_convert_rgb565_to_rgb666(pixels, pixel_count);

    dc_high();
    cs_low();

    if (!lcd_spi_start_write(g_lcd_spi_tx_buffer,
                             pixel_count * LCD_SPI_BYTES_PER_PIXEL,
                             true,
                             p_callback,
                             p_context))
    {
        cs_high();
        return false;
    }

    return true;
}

bool lcd_spi_wait_for_async(void)
{
    return lcd_spi_wait_for_transfer();
}

bool lcd_write_pixels_rgb565(const uint16_t * pixels, uint32_t pixel_count)
{
    if (NULL == pixels)
    {
        return false;
    }

    while (pixel_count > 0U)
    {
        uint32_t chunk_pixels = pixel_count;
        if (chunk_pixels > LCD_SPI_MAX_PIXELS_PER_TRANSFER)
        {
            chunk_pixels = LCD_SPI_MAX_PIXELS_PER_TRANSFER;
        }

        if (!lcd_write_pixels_rgb565_async(pixels,
                                           chunk_pixels,
                                           NULL,
                                           NULL) ||
            !lcd_spi_wait_for_transfer())
        {
            return false;
        }

        pixels += chunk_pixels;
        pixel_count -= chunk_pixels;
    }

    return true;
}

/* ===== LCD command / data / window ===== */
void lcd_cmd(uint8_t cmd)  { cs_low(); dc_low();  spi_write_byte(cmd); cs_high(); }
void lcd_data(uint8_t d)   { cs_low(); dc_high(); spi_write_byte(d);   cs_high(); }

void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    lcd_cmd(0x2A);
    lcd_data((uint8_t)(x1 >> 8));
    lcd_data((uint8_t)(x1 & 0xFF));
    lcd_data((uint8_t)(x2 >> 8));
    lcd_data((uint8_t)(x2 & 0xFF));

    lcd_cmd(0x2B);
    lcd_data((uint8_t)(y1 >> 8));
    lcd_data((uint8_t)(y1 & 0xFF));
    lcd_data((uint8_t)(y2 >> 8));
    lcd_data((uint8_t)(y2 & 0xFF));

    lcd_cmd(0x2C);
}

void lcd_fill_screen(uint16_t color)
{
    lcd_fill_rect(0, 0, 479, 319, color);
}

void lcd_fill_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint32_t pixel_count = (uint32_t)(x2 - x1 + 1U) * (uint32_t)(y2 - y1 + 1U);
    uint8_t r5 = (uint8_t) ((color >> 11U) & 0x1FU);
    uint8_t g6 = (uint8_t) ((color >> 5U)  & 0x3FU);
    uint8_t b5 = (uint8_t) (color & 0x1FU);
    uint8_t rgb[3] =
    {
        (uint8_t) ((r5 << 3U) | (r5 >> 2U)),
        (uint8_t) ((g6 << 2U) | (g6 >> 4U)),
        (uint8_t) ((b5 << 3U) | (b5 >> 2U))
    };

    lcd_set_window(x1, y1, x2, y2);

    dc_high();
    cs_low();
    while (pixel_count > 0U)
    {
        uint32_t chunk_pixels = pixel_count;
        if (chunk_pixels > LCD_SPI_MAX_PIXELS_PER_TRANSFER)
        {
            chunk_pixels = LCD_SPI_MAX_PIXELS_PER_TRANSFER;
        }

        for (uint32_t i = 0U; i < chunk_pixels; i++)
        {
            g_lcd_spi_tx_buffer[(i * 3U) + 0U] = rgb[0];
            g_lcd_spi_tx_buffer[(i * 3U) + 1U] = rgb[1];
            g_lcd_spi_tx_buffer[(i * 3U) + 2U] = rgb[2];
        }

        if (!lcd_spi_write(g_lcd_spi_tx_buffer,
                           chunk_pixels * LCD_SPI_BYTES_PER_PIXEL))
        {
            break;
        }

        pixel_count -= chunk_pixels;
    }
    cs_high();
}

/* ===== ILI9488 init (STM32 4SPI reference, 18-bit RGB666) ===== */
void lcd_init(void)
{
    R_IOPORT_PinCfg(&g_ioport_ctrl, PIN_CS,   IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_DRIVE_HIGH | IOPORT_CFG_PORT_OUTPUT_HIGH);
    R_IOPORT_PinCfg(&g_ioport_ctrl, PIN_RST,  IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_DRIVE_HIGH | IOPORT_CFG_PORT_OUTPUT_HIGH);
    R_IOPORT_PinCfg(&g_ioport_ctrl, PIN_DC,   IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_DRIVE_HIGH | IOPORT_CFG_PORT_OUTPUT_LOW);
    R_IOPORT_PinCfg(&g_ioport_ctrl, PIN_LED,  IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_DRIVE_HIGH | IOPORT_CFG_PORT_OUTPUT_LOW);

    cs_high();

    g_lcd_spi_last_error = g_spi_lcd.p_api->open(g_spi_lcd.p_ctrl,
                                                  g_spi_lcd.p_cfg);
    if (FSP_SUCCESS != g_lcd_spi_last_error)
    {
        return;
    }
    g_lcd_spi_is_open = true;

    R_IOPORT_PinWrite(&g_ioport_ctrl, PIN_RST, BSP_IO_LEVEL_HIGH);
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
    R_IOPORT_PinWrite(&g_ioport_ctrl, PIN_RST, BSP_IO_LEVEL_LOW);
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    R_IOPORT_PinWrite(&g_ioport_ctrl, PIN_RST, BSP_IO_LEVEL_HIGH);
    R_BSP_SoftwareDelay(150, BSP_DELAY_UNITS_MILLISECONDS);
    led_write(BSP_IO_LEVEL_HIGH);

    lcd_cmd(0xF7); lcd_data(0xA9); lcd_data(0x51); lcd_data(0x2C); lcd_data(0x82);
    lcd_cmd(0xEC); lcd_data(0x00); lcd_data(0x02); lcd_data(0x03); lcd_data(0x7A);
    lcd_cmd(0xC0); lcd_data(0x13); lcd_data(0x13);
    lcd_cmd(0xC1); lcd_data(0x41);
    lcd_cmd(0xC5); lcd_data(0x00); lcd_data(0x28); lcd_data(0x80);
    lcd_cmd(0xB0); lcd_data(0x00);
    lcd_cmd(0xB1); lcd_data(0xB0); lcd_data(0x11);
    lcd_cmd(0xB4); lcd_data(0x02);
    lcd_cmd(0xB6); lcd_data(0x02); lcd_data(0x22);
    lcd_cmd(0xB7); lcd_data(0xC6);
    lcd_cmd(0xBE); lcd_data(0x00); lcd_data(0x04);
    lcd_cmd(0xE9); lcd_data(0x00);
    lcd_cmd(0xF4); lcd_data(0x00); lcd_data(0x00); lcd_data(0x0F);

    lcd_cmd(0xE0);
    { const uint8_t g[] = {0x00,0x04,0x0E,0x08,0x17,0x0A,0x40,0x79,0x4D,0x07,0x0E,0x0A,0x1A,0x1D,0x0F};
      dc_high(); cs_low(); for (int i=0;i<15;i++) spi_write_byte(g[i]); cs_high(); }

    lcd_cmd(0xE1);
    { const uint8_t g[] = {0x00,0x1B,0x1F,0x02,0x10,0x05,0x32,0x34,0x43,0x02,0x0A,0x09,0x33,0x37,0x0F};
      dc_high(); cs_low(); for (int i=0;i<15;i++) spi_write_byte(g[i]); cs_high(); }

    lcd_cmd(0xF4); lcd_data(0x00); lcd_data(0x00); lcd_data(0x0F);
    lcd_cmd(0x36); lcd_data(LCD_MADCTL_LANDSCAPE_180);
    lcd_cmd(0x3A); lcd_data(0x66);
    lcd_cmd(0x21);
    lcd_cmd(0x11); R_BSP_SoftwareDelay(120, BSP_DELAY_UNITS_MILLISECONDS);
    lcd_cmd(0x29); R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);
}
