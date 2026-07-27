#include "ft6336.h"
#include "hal_data.h"

#include <stdbool.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#define PIN_CTP_RST BSP_IO_PORT_08_PIN_01

#define FT6336_REG_GESTURE_ID   0x01U
#define FT6336_REG_TD_STATUS    0x02U
#define FT6336_REG_TOUCH1_XH    0x03U
#define FT6336_REG_THRESHOLD    0x80U
#define FT6336_REG_CHIP_ID      0xA3U
#define FT6336_REG_VENDOR_ID    0xA8U
#define FT6336_REG_FOCALTECH_ID 0xACU
#define FT6336_TOUCH_FRAME_LEN  16U
#define FT6336_TOUCH_MAX        2U
#define FT6336_TOUCH_EVENT_MASK 0xC0U
#define FT6336_TOUCH_HIGH_MASK  0x0FU
#define FT6336_TOUCH_ID_MASK    0xF0U
#define FT6336_TOUCH_THRESHOLD  32U
#define FT6336_I2C_TIMEOUT      pdMS_TO_TICKS(50U)

volatile uint8_t g_ft6336_last_frame[FT6336_TOUCH_FRAME_LEN];
volatile uint8_t g_ft6336_last_read_ok;
volatile uint8_t g_ft6336_last_touches;
volatile uint8_t g_ft6336_last_chip_id;
volatile uint8_t g_ft6336_last_vendor_id;
volatile uint8_t g_ft6336_last_focaltech_id;

static volatile bool g_i2c_done;
static volatile bool g_i2c_error;
static bool g_i2c_open;

void i2c_touch_callback(i2c_master_callback_args_t * p_args)
{
    if (p_args == NULL) {
        return;
    }

    g_i2c_error = (p_args->event == I2C_MASTER_EVENT_ABORTED);
    if ((p_args->event == I2C_MASTER_EVENT_TX_COMPLETE) ||
        (p_args->event == I2C_MASTER_EVENT_RX_COMPLETE) ||
        (p_args->event == I2C_MASTER_EVENT_ABORTED)) {
        g_i2c_done = true;
    }
}

static bool i2c_wait_done(void)
{
    TickType_t const start = xTaskGetTickCount();

    while (!g_i2c_done) {
        if ((xTaskGetTickCount() - start) > FT6336_I2C_TIMEOUT) {
            (void) g_i2c_touch.p_api->abort(g_i2c_touch.p_ctrl);
            return false;
        }
        vTaskDelay(1);
    }

    return !g_i2c_error;
}

static uint8_t ft6336_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};

    if (!g_i2c_open) {
        return 1U;
    }

    g_i2c_done = false;
    g_i2c_error = false;
    if (g_i2c_touch.p_api->write(g_i2c_touch.p_ctrl, tx, sizeof(tx), false) != FSP_SUCCESS) {
        return 1U;
    }

    return i2c_wait_done() ? 0U : 1U;
}

static uint8_t ft6336_read_block(uint8_t reg, uint8_t * buf, uint8_t len)
{
    if ((!g_i2c_open) || (buf == NULL) || (len == 0U)) {
        return 1U;
    }

    g_i2c_done = false;
    g_i2c_error = false;
    if (g_i2c_touch.p_api->write(g_i2c_touch.p_ctrl, &reg, 1U, true) != FSP_SUCCESS) {
        return 1U;
    }
    if (!i2c_wait_done()) {
        return 1U;
    }

    g_i2c_done = false;
    g_i2c_error = false;
    if (g_i2c_touch.p_api->read(g_i2c_touch.p_ctrl, buf, len, false) != FSP_SUCCESS) {
        return 1U;
    }

    return i2c_wait_done() ? 0U : 1U;
}

static uint8_t ft6336_read_reg(uint8_t reg, uint8_t * value)
{
    if (value == NULL) {
        return 1U;
    }

    return ft6336_read_block(reg, value, 1U);
}

void ft6336_init(void)
{
    R_IOPORT_PinCfg(&g_ioport_ctrl, PIN_CTP_RST, IOPORT_CFG_PORT_DIRECTION_OUTPUT);

    R_IOPORT_PinWrite(&g_ioport_ctrl, PIN_CTP_RST, BSP_IO_LEVEL_LOW);
    R_BSP_SoftwareDelay(20, BSP_DELAY_UNITS_MILLISECONDS);
    R_IOPORT_PinWrite(&g_ioport_ctrl, PIN_CTP_RST, BSP_IO_LEVEL_HIGH);
    R_BSP_SoftwareDelay(300, BSP_DELAY_UNITS_MILLISECONDS);

    g_i2c_open = (g_i2c_touch.p_api->open(g_i2c_touch.p_ctrl, g_i2c_touch.p_cfg) == FSP_SUCCESS);
    (void) ft6336_write_reg(FT6336_REG_THRESHOLD, FT6336_TOUCH_THRESHOLD);
    g_ft6336_last_read_ok = 0U;
    g_ft6336_last_touches = 0U;
    for (uint8_t i = 0U; i < FT6336_TOUCH_FRAME_LEN; i++) {
        g_ft6336_last_frame[i] = 0U;
    }
    (void) ft6336_read_reg(FT6336_REG_CHIP_ID, (uint8_t *) &g_ft6336_last_chip_id);
    (void) ft6336_read_reg(FT6336_REG_VENDOR_ID, (uint8_t *) &g_ft6336_last_vendor_id);
    (void) ft6336_read_reg(FT6336_REG_FOCALTECH_ID, (uint8_t *) &g_ft6336_last_focaltech_id);
}

uint8_t ft6336_read_point(ft6336_point_t * point)
{
    uint8_t buf[4];
    uint8_t touches;

    if (point == NULL) {
        return 0U;
    }

    point->touches = 0U;
    point->event = 0U;
    point->track_id = 0U;
    point->x = 0U;
    point->y = 0U;

    for (uint8_t i = 0U; i < FT6336_TOUCH_FRAME_LEN; i++) {
        g_ft6336_last_frame[i] = 0U;
    }
    g_ft6336_last_read_ok = 0U;
    if (ft6336_read_reg(FT6336_REG_TD_STATUS, &touches) != 0U) {
        return 0U;
    }
    g_ft6336_last_frame[FT6336_REG_TD_STATUS] = touches;
    g_ft6336_last_read_ok = 1U;
    touches = (uint8_t) (touches & 0x0FU);
    g_ft6336_last_touches = touches;
    if ((touches == 0U) || (touches > FT6336_TOUCH_MAX)) {
        return 0U;
    }

    if (ft6336_read_block(FT6336_REG_TOUCH1_XH, buf, (uint8_t) sizeof(buf)) != 0U) {
        return 0U;
    }
    for (uint8_t i = 0U; i < (uint8_t) sizeof(buf); i++) {
        g_ft6336_last_frame[FT6336_REG_TOUCH1_XH + i] = buf[i];
    }

    point->touches = touches;
    point->event = (uint8_t) ((buf[0] & FT6336_TOUCH_EVENT_MASK) >> 6);
    point->x = (uint16_t) ((((uint16_t) buf[0] & FT6336_TOUCH_HIGH_MASK) << 8) |
                           (uint16_t) buf[1]);
    point->track_id = (uint8_t) ((buf[2] & FT6336_TOUCH_ID_MASK) >> 4);
    point->y = (uint16_t) ((((uint16_t) buf[2] & FT6336_TOUCH_HIGH_MASK) << 8) |
                           (uint16_t) buf[3]);
    return 1U;
}

uint8_t ft6336_scan(uint16_t * x, uint16_t * y)
{
    ft6336_point_t point;

    if (ft6336_read_point(&point) == 0U) {
        return 0U;
    }

    if (x != NULL) {
        *x = point.x;
    }
    if (y != NULL) {
        *y = point.y;
    }
    return 1U;
}
