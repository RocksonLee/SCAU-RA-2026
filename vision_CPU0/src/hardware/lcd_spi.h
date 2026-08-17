#ifndef LCD_SPI_H
#define LCD_SPI_H

#include "hal_data.h"
#include <stdbool.h>

/* ===== Pin definitions ===== */
#define PIN_CS      BSP_IO_PORT_05_PIN_15  // P515
#define PIN_RST     BSP_IO_PORT_06_PIN_00  // P600
#define PIN_DC      BSP_IO_PORT_01_PIN_02  // P102
#define PIN_LED     BSP_IO_PORT_01_PIN_06  // P106

#ifdef __cplusplus
extern "C" {
#endif

/* ===== GPIO helpers ===== */
static inline void cs_low(void)  { R_IOPORT_PinWrite(&g_ioport_ctrl, PIN_CS, BSP_IO_LEVEL_LOW); }
static inline void cs_high(void) { R_IOPORT_PinWrite(&g_ioport_ctrl, PIN_CS, BSP_IO_LEVEL_HIGH); }
static inline void dc_low(void)  { R_IOPORT_PinWrite(&g_ioport_ctrl, PIN_DC, BSP_IO_LEVEL_LOW); }
static inline void dc_high(void) { R_IOPORT_PinWrite(&g_ioport_ctrl, PIN_DC, BSP_IO_LEVEL_HIGH); }
static inline void led_write(bsp_io_level_t v)  { R_IOPORT_PinWrite(&g_ioport_ctrl, PIN_LED, v); }

/* ===== Hardware SPI ===== */
typedef void (* lcd_spi_async_callback_t)(void * p_context);

bool lcd_spi_write(const uint8_t * data, uint32_t length);
void spi_write_byte(uint8_t data);
bool lcd_write_pixels_rgb565(const uint16_t * pixels, uint32_t pixel_count);
bool lcd_write_pixels_rgb565_async(const uint16_t       * pixels,
                                   uint32_t               pixel_count,
                                   lcd_spi_async_callback_t p_callback,
                                   void                   * p_context);
bool lcd_spi_wait_for_async(void);

/* Debug information: inspect these variables when an SPI transfer fails. */
extern volatile fsp_err_t   g_lcd_spi_last_error;
extern volatile spi_event_t g_lcd_spi_last_event;

/* ===== LCD command / data / window ===== */
void lcd_cmd(uint8_t cmd);
void lcd_data(uint8_t d);
void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void lcd_fill_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcd_fill_screen(uint16_t color);

/* ===== LCD initialization ===== */
void lcd_init(void);

#ifdef __cplusplus
}
#endif

#endif /* LCD_SPI_H */
