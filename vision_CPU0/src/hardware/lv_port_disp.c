#include "lv_port_disp.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "lvgl.h"
#pragma GCC diagnostic pop
#include "lcd_spi.h"

#define DISP_HOR_RES  480
#define DISP_VER_RES  320
#define BYTE_PER_PIXEL 2   /* RGB565 = 2 bytes */

/* Single partial buffer: 480 x 20 rows x 2 bytes = 19200 bytes */
#define BUF_ROWS  20
static LV_ATTRIBUTE_MEM_ALIGN uint8_t buf_1[DISP_HOR_RES * BUF_ROWS * BYTE_PER_PIXEL];

static void disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
    uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);

    lcd_set_window((uint16_t)area->x1, (uint16_t)area->y1,
                   (uint16_t)area->x2, (uint16_t)area->y2);

    (void) lcd_write_pixels_rgb565((const uint16_t *) px_map, w * h);

    lv_display_flush_ready(disp);
}

void lv_port_disp_init(void)
{
    lv_display_t * disp = lv_display_create(DISP_HOR_RES, DISP_VER_RES);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp, disp_flush);
    lv_display_set_buffers(disp, buf_1, NULL, sizeof(buf_1), LV_DISPLAY_RENDER_MODE_PARTIAL);
}
