#include "camera_ov5640.h"

#include <stdbool.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#define OV5640_I2C_TIMEOUT_TICKS    (pdMS_TO_TICKS(500U))
#define OV5640_CAPTURE_TIMEOUT_MS   (1000U)
#define OV5640_CHIP_ID              (0x5640U)
#define OV5640_REG_END              (0xFFFFU)
#define OV5640_REG_DELAY            (0xFFFEU)
#define OV5640_RESET_RETRIES        (3U)

#define OV5640_PIN_RESET            BSP_IO_PORT_07_PIN_09
#define OV5640_PIN_PWDN             BSP_IO_PORT_07_PIN_10
#define OV5640_ENABLE_COLOR_BAR     (0)
#define OV5640_CEU_FATAL_EVENTS     (CEU_EVENT_CRAM_OVERFLOW | CEU_EVENT_FIREWALL)

#define OV5640_STEP_NONE            (0U)
#define OV5640_STEP_I2C_OPEN        (1U)
#define OV5640_STEP_SOFT_RESET      (2U)
#define OV5640_STEP_CHIP_ID         (3U)
#define OV5640_STEP_INIT_TABLE      (4U)
#define OV5640_STEP_QVGA_TABLE      (5U)
#define OV5640_STEP_AWB_TABLE       (6U)
#define OV5640_STEP_COLOR_BAR       (7U)
#define OV5640_STEP_CEU_OPEN        (8U)

typedef struct st_ov5640_reg
{
    uint16_t reg;
    uint8_t  val;
} ov5640_reg_t;

static volatile bool g_i2c_done;
static volatile bool g_i2c_error;
static volatile bool g_frame_done;
static volatile uint32_t g_ceu_events;
static uint32_t g_last_error_step;
static camera_ov5640_ceu_debug_t g_ceu_debug;

static const ov5640_reg_t g_ov5640_init_regs[] =
{
    {0x3008, 0x42}, {0x3103, 0x03}, {0x3017, 0xFF}, {0x3018, 0xFF},
    {0x3034, 0x1A}, {0x3037, 0x13}, {0x3108, 0x01}, {0x3630, 0x36},
    {0x3631, 0x0E}, {0x3632, 0xE2}, {0x3633, 0x12}, {0x3621, 0xE0},
    {0x3704, 0xA0}, {0x3703, 0x5A}, {0x3715, 0x78}, {0x3717, 0x01},
    {0x370B, 0x60}, {0x3705, 0x1A}, {0x3905, 0x02}, {0x3906, 0x10},
    {0x3901, 0x0A}, {0x3731, 0x12}, {0x3600, 0x08}, {0x3601, 0x33},
    {0x302D, 0x60}, {0x3620, 0x52}, {0x371B, 0x20}, {0x471C, 0x50},
    {0x3A13, 0x43}, {0x3A18, 0x00}, {0x3A19, 0xF8}, {0x3635, 0x13},
    {0x3636, 0x03}, {0x3634, 0x40}, {0x3622, 0x01}, {0x3C01, 0x34},
    {0x3C04, 0x28}, {0x3C05, 0x98}, {0x3C06, 0x00}, {0x3C07, 0x08},
    {0x3C08, 0x00}, {0x3C09, 0x1C}, {0x3C0A, 0x9C}, {0x3C0B, 0x40},
    {0x3810, 0x00}, {0x3811, 0x10}, {0x3812, 0x00}, {0x3708, 0x64},
    {0x4001, 0x02}, {0x4005, 0x1A}, {0x3000, 0x00}, {0x3004, 0xFF},
    {0x300E, 0x58}, {0x302E, 0x00}, {0x4300, 0x30}, {0x501F, 0x00},
    {0x440E, 0x00}, {0x5000, 0xA7}, {0x3A0F, 0x30}, {0x3A10, 0x28},
    {0x3A1B, 0x30}, {0x3A1E, 0x26}, {0x3A11, 0x60}, {0x3A1F, 0x14},
    {0x5180, 0xFF}, {0x5181, 0xF2}, {0x5182, 0x00}, {0x5183, 0x14},
    {0x5184, 0x25}, {0x5185, 0x24}, {0x5186, 0x09}, {0x5187, 0x09},
    {0x5188, 0x09}, {0x5189, 0x75}, {0x518A, 0x54}, {0x518B, 0xE0},
    {0x518C, 0xB2}, {0x518D, 0x42}, {0x518E, 0x3D}, {0x518F, 0x56},
    {0x5190, 0x46}, {0x5191, 0xF8}, {0x5192, 0x04}, {0x5193, 0x70},
    {0x5194, 0xF0}, {0x5195, 0xF0}, {0x5196, 0x03}, {0x5197, 0x01},
    {0x5198, 0x04}, {0x5199, 0x12}, {0x519A, 0x04}, {0x519B, 0x00},
    {0x519C, 0x06}, {0x519D, 0x82}, {0x519E, 0x38}, {0x5480, 0x01},
    {0x5481, 0x08}, {0x5482, 0x14}, {0x5483, 0x28}, {0x5484, 0x51},
    {0x5485, 0x65}, {0x5486, 0x71}, {0x5487, 0x7D}, {0x5488, 0x87},
    {0x5489, 0x91}, {0x548A, 0x9A}, {0x548B, 0xAA}, {0x548C, 0xB8},
    {0x548D, 0xCD}, {0x548E, 0xDD}, {0x548F, 0xEA}, {0x5490, 0x1D},
    {0x5381, 0x1E}, {0x5382, 0x5B}, {0x5383, 0x08}, {0x5384, 0x0A},
    {0x5385, 0x7E}, {0x5386, 0x88}, {0x5387, 0x7C}, {0x5388, 0x6C},
    {0x5389, 0x10}, {0x538A, 0x01}, {0x538B, 0x98}, {0x5580, 0x06},
    {0x5583, 0x40}, {0x5584, 0x10}, {0x5589, 0x10}, {0x558A, 0x00},
    {0x558B, 0xF8}, {0x501D, 0x40}, {0x5300, 0x08}, {0x5301, 0x30},
    {0x5302, 0x10}, {0x5303, 0x00}, {0x5304, 0x08}, {0x5305, 0x30},
    {0x5306, 0x08}, {0x5307, 0x16}, {0x5309, 0x08}, {0x530A, 0x30},
    {0x530B, 0x04}, {0x530C, 0x06}, {0x5025, 0x00}, {0x3008, 0x02},
    {0x4740, 0x21},
    {OV5640_REG_END, 0x00},
};

static const ov5640_reg_t g_ov5640_qvga_rgb565_regs[] =
{
    {0x4300, 0x6F},
    {0x501F, 0x01},
    {0x3035, 0x41}, {0x3036, 0x69}, {0x3C07, 0x07},
    {0x3820, 0x46}, {0x3821, 0x00}, {0x3814, 0x31}, {0x3815, 0x31},
    {0x3800, 0x00}, {0x3801, 0x00}, {0x3802, 0x00}, {0x3803, 0x00},
    {0x3804, 0x0A}, {0x3805, 0x3F}, {0x3806, 0x06}, {0x3807, 0xA9},
    {0x3808, 0x01}, {0x3809, 0x40}, {0x380A, 0x00}, {0x380B, 0xF0},
    {0x380C, 0x05}, {0x380D, 0xF8}, {0x380E, 0x03}, {0x380F, 0x84},
    {0x3813, 0x04}, {0x3618, 0x00}, {0x3612, 0x29}, {0x3709, 0x52},
    {0x370C, 0x03}, {0x3A02, 0x02}, {0x3A03, 0xE0}, {0x3A14, 0x02},
    {0x3A15, 0xE0}, {0x4004, 0x02}, {0x3002, 0x1C}, {0x3006, 0xC3},
    {0x4713, 0x03}, {0x4407, 0x04}, {0x460B, 0x37}, {0x460C, 0x20},
    {0x4837, 0x16}, {0x3824, 0x04}, {0x5001, 0xA3}, {0x3503, 0x00},
    {OV5640_REG_END, 0x00},
};

static const ov5640_reg_t g_ov5640_advanced_awb_regs[] =
{
    {0x3406, 0x00},
    {0x5192, 0x04}, {0x5191, 0xF8}, {0x5193, 0x70}, {0x5194, 0xF0},
    {0x5195, 0xF0}, {0x518D, 0x3D}, {0x518F, 0x54}, {0x518E, 0x3D},
    {0x5190, 0x54}, {0x518B, 0xA8}, {0x518C, 0xA8}, {0x5187, 0x18},
    {0x5188, 0x18}, {0x5189, 0x6E}, {0x518A, 0x68}, {0x5186, 0x1C},
    {0x5181, 0x50}, {0x5184, 0x25}, {0x5182, 0x11}, {0x5183, 0x14},
    {0x5184, 0x25}, {0x5185, 0x24},
    {OV5640_REG_END, 0x00},
};

void i2c_camera_callback(i2c_master_callback_args_t * p_args)
{
    if (NULL == p_args)
    {
        return;
    }

    g_i2c_error = (I2C_MASTER_EVENT_ABORTED == p_args->event);
    if ((I2C_MASTER_EVENT_TX_COMPLETE == p_args->event) ||
        (I2C_MASTER_EVENT_RX_COMPLETE == p_args->event) ||
        (I2C_MASTER_EVENT_ABORTED == p_args->event))
    {
        g_i2c_done = true;
    }
}

void g_ceu0_user_callback(capture_callback_args_t * p_args)
{
    if (NULL == p_args)
    {
        return;
    }

    g_ceu_events |= p_args->event;
    if (0U != (p_args->event & (CEU_EVENT_FRAME_END | OV5640_CEU_FATAL_EVENTS)))
    {
        g_frame_done = true;
    }
}

static bool wait_i2c_done(void)
{
    TickType_t const start = xTaskGetTickCount();

    while (!g_i2c_done)
    {
        if ((xTaskGetTickCount() - start) > OV5640_I2C_TIMEOUT_TICKS)
        {
            (void) g_i2c_camera.p_api->abort(g_i2c_camera.p_ctrl);
            return false;
        }

        vTaskDelay(1);
    }

    return !g_i2c_error;
}

static bool ov5640_write_reg(uint16_t reg, uint8_t val)
{
    uint8_t tx[3];

    tx[0] = (uint8_t) (reg >> 8);
    tx[1] = (uint8_t) reg;
    tx[2] = val;

    g_i2c_done = false;
    g_i2c_error = false;

    if (FSP_SUCCESS != g_i2c_camera.p_api->write(g_i2c_camera.p_ctrl, tx, sizeof(tx), false))
    {
        return false;
    }

    return wait_i2c_done();
}

static bool ov5640_read_reg(uint16_t reg, uint8_t * p_val)
{
    uint8_t tx[2];

    tx[0] = (uint8_t) (reg >> 8);
    tx[1] = (uint8_t) reg;

    g_i2c_done = false;
    g_i2c_error = false;
    if (FSP_SUCCESS != g_i2c_camera.p_api->write(g_i2c_camera.p_ctrl, tx, sizeof(tx), true))
    {
        return false;
    }

    if (!wait_i2c_done())
    {
        return false;
    }

    g_i2c_done = false;
    g_i2c_error = false;
    if (FSP_SUCCESS != g_i2c_camera.p_api->read(g_i2c_camera.p_ctrl, p_val, 1U, false))
    {
        return false;
    }

    return wait_i2c_done();
}

static bool ov5640_write_table(ov5640_reg_t const * p_table)
{
    for (uint32_t i = 0; p_table[i].reg != OV5640_REG_END; i++)
    {
        if (p_table[i].reg == OV5640_REG_DELAY)
        {
            vTaskDelay(pdMS_TO_TICKS(p_table[i].val));
        }
        else if (!ov5640_write_reg(p_table[i].reg, p_table[i].val))
        {
            return false;
        }
    }

    return true;
}

static void ov5640_configure_parallel_pins(void)
{
    uint32_t const ceu_input =
        (uint32_t) IOPORT_CFG_DRIVE_MID |
        (uint32_t) IOPORT_CFG_PERIPHERAL_PIN |
        (uint32_t) IOPORT_PERIPHERAL_CEU;
    uint32_t const ceu_input_high_drive =
        (uint32_t) IOPORT_CFG_DRIVE_HIGH |
        (uint32_t) IOPORT_CFG_NMOS_ENABLE |
        (uint32_t) IOPORT_CFG_PERIPHERAL_PIN |
        (uint32_t) IOPORT_PERIPHERAL_CEU;

    (void) g_ioport.p_api->pinCfg(g_ioport.p_ctrl, BSP_IO_PORT_04_PIN_00, ceu_input_high_drive);
    (void) g_ioport.p_api->pinCfg(g_ioport.p_ctrl, BSP_IO_PORT_04_PIN_01, ceu_input_high_drive);
    (void) g_ioport.p_api->pinCfg(g_ioport.p_ctrl, BSP_IO_PORT_04_PIN_05, ceu_input);
    (void) g_ioport.p_api->pinCfg(g_ioport.p_ctrl, BSP_IO_PORT_04_PIN_06, ceu_input);
    (void) g_ioport.p_api->pinCfg(g_ioport.p_ctrl, BSP_IO_PORT_04_PIN_14, ceu_input);
    (void) g_ioport.p_api->pinCfg(g_ioport.p_ctrl, BSP_IO_PORT_04_PIN_15, ceu_input);
    (void) g_ioport.p_api->pinCfg(g_ioport.p_ctrl, BSP_IO_PORT_07_PIN_00, ceu_input);
    (void) g_ioport.p_api->pinCfg(g_ioport.p_ctrl, BSP_IO_PORT_07_PIN_01, ceu_input);
    (void) g_ioport.p_api->pinCfg(g_ioport.p_ctrl, BSP_IO_PORT_07_PIN_02, ceu_input);
    (void) g_ioport.p_api->pinCfg(g_ioport.p_ctrl, BSP_IO_PORT_07_PIN_03, ceu_input);
    (void) g_ioport.p_api->pinCfg(g_ioport.p_ctrl, BSP_IO_PORT_07_PIN_08, ceu_input);
}

static void ov5640_save_ceu_debug(fsp_err_t capture_start_error)
{
    g_ceu_debug.capture_start_error = (uint32_t) capture_start_error;
    g_ceu_debug.caps                = R_CEU->CAPSR;
    g_ceu_debug.status              = R_CEU->CSTSR;
    g_ceu_debug.events              = R_CEU->CETCR;
    g_ceu_debug.data_size           = R_CEU->CDSSR;
    g_ceu_debug.interface_control   = R_CEU->CAMCR;
}

static void ov5640_pin_reset(void)
{
    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, OV5640_PIN_RESET, BSP_IO_LEVEL_LOW);
    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, OV5640_PIN_PWDN, BSP_IO_LEVEL_HIGH);
    vTaskDelay(pdMS_TO_TICKS(50U));
    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, OV5640_PIN_PWDN, BSP_IO_LEVEL_LOW);
    vTaskDelay(pdMS_TO_TICKS(50U));
    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, OV5640_PIN_RESET, BSP_IO_LEVEL_HIGH);
    vTaskDelay(pdMS_TO_TICKS(200U));
}

static bool ov5640_soft_reset(void)
{
    for (uint32_t attempt = 0U; attempt < OV5640_RESET_RETRIES; attempt++)
    {
        if (ov5640_write_reg(0x3103, 0x03) && ov5640_write_reg(0x3008, 0x82))
        {
            vTaskDelay(pdMS_TO_TICKS(100U));
            return true;
        }

        (void) g_i2c_camera.p_api->abort(g_i2c_camera.p_ctrl);
        ov5640_pin_reset();
    }

    return false;
}

static bool ov5640_recover_ceu(void)
{
    fsp_err_t err;

    err = g_ceu0.p_api->close(g_ceu0.p_ctrl);
    if ((FSP_SUCCESS != err) && (FSP_ERR_NOT_OPEN != err))
    {
        return false;
    }

    vTaskDelay(1);

    err = g_ceu0.p_api->open(g_ceu0.p_ctrl, g_ceu0.p_cfg);
    return (FSP_SUCCESS == err) || (FSP_ERR_ALREADY_OPEN == err);
}

uint16_t camera_ov5640_chip_id(void)
{
    uint8_t idh = 0U;
    uint8_t idl = 0U;

    if (!ov5640_read_reg(0x300A, &idh) || !ov5640_read_reg(0x300B, &idl))
    {
        return 0U;
    }

    return (uint16_t) (((uint16_t) idh << 8) | idl);
}

bool camera_ov5640_read_reg(uint16_t reg, uint8_t * p_val)
{
    if (NULL == p_val)
    {
        return false;
    }

    return ov5640_read_reg(reg, p_val);
}

uint32_t camera_ov5640_last_ceu_events(void)
{
    return g_ceu_events;
}

uint32_t camera_ov5640_last_error_step(void)
{
    return g_last_error_step;
}

void camera_ov5640_get_ceu_debug(camera_ov5640_ceu_debug_t * p_debug)
{
    if (NULL != p_debug)
    {
        *p_debug = g_ceu_debug;
    }
}

camera_ov5640_result_t camera_ov5640_init(void)
{
    fsp_err_t err;

    g_last_error_step = OV5640_STEP_NONE;
    ov5640_configure_parallel_pins();
    ov5640_pin_reset();

    err = g_i2c_camera.p_api->open(g_i2c_camera.p_ctrl, g_i2c_camera.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        g_last_error_step = OV5640_STEP_I2C_OPEN;
        return CAMERA_OV5640_ERR_I2C;
    }

    if (!ov5640_soft_reset())
    {
        g_last_error_step = OV5640_STEP_SOFT_RESET;
        return CAMERA_OV5640_ERR_I2C;
    }

    if (OV5640_CHIP_ID != camera_ov5640_chip_id())
    {
        g_last_error_step = OV5640_STEP_CHIP_ID;
        return CAMERA_OV5640_ERR_CHIP_ID;
    }

    if (!ov5640_write_table(g_ov5640_init_regs))
    {
        g_last_error_step = OV5640_STEP_INIT_TABLE;
        return CAMERA_OV5640_ERR_I2C;
    }

    if (!ov5640_write_table(g_ov5640_qvga_rgb565_regs))
    {
        g_last_error_step = OV5640_STEP_QVGA_TABLE;
        return CAMERA_OV5640_ERR_I2C;
    }

    if (!ov5640_write_table(g_ov5640_advanced_awb_regs))
    {
        g_last_error_step = OV5640_STEP_AWB_TABLE;
        return CAMERA_OV5640_ERR_I2C;
    }

#if OV5640_ENABLE_COLOR_BAR
    if (!ov5640_write_reg(0x503D, 0x80) || !ov5640_write_reg(0x4741, 0x00))
    {
        g_last_error_step = OV5640_STEP_COLOR_BAR;
        return CAMERA_OV5640_ERR_I2C;
    }
#endif

    err = g_ceu0.p_api->open(g_ceu0.p_ctrl, g_ceu0.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        g_last_error_step = OV5640_STEP_CEU_OPEN;
        return CAMERA_OV5640_ERR_CAPTURE;
    }

    return CAMERA_OV5640_OK;
}

camera_ov5640_result_t camera_ov5640_capture_frame(uint8_t * p_frame)
{
    TickType_t const start = xTaskGetTickCount();
    fsp_err_t capture_start_error;

    if (NULL == p_frame)
    {
        return CAMERA_OV5640_ERR_CAPTURE;
    }

    ov5640_configure_parallel_pins();
    g_frame_done = false;
    g_ceu_events = 0U;
    g_ceu_debug.capture_start_error = 0U;
    g_ceu_debug.caps                = 0U;
    g_ceu_debug.status              = 0U;
    g_ceu_debug.events              = 0U;
    g_ceu_debug.data_size           = 0U;
    g_ceu_debug.interface_control   = 0U;

    capture_start_error = g_ceu0.p_api->captureStart(g_ceu0.p_ctrl, p_frame);
    if (FSP_SUCCESS != capture_start_error)
    {
        ov5640_save_ceu_debug(capture_start_error);
        (void) ov5640_recover_ceu();
        return CAMERA_OV5640_ERR_CAPTURE;
    }

    while (!g_frame_done)
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(OV5640_CAPTURE_TIMEOUT_MS))
        {
            ov5640_save_ceu_debug(FSP_SUCCESS);
            (void) ov5640_recover_ceu();
            return CAMERA_OV5640_ERR_CAPTURE;
        }

        if (0U != (g_ceu_events & OV5640_CEU_FATAL_EVENTS))
        {
            ov5640_save_ceu_debug(FSP_SUCCESS);
            (void) ov5640_recover_ceu();
            return CAMERA_OV5640_ERR_CAPTURE;
        }

        vTaskDelay(1);
    }

    if (0U != (g_ceu_events & OV5640_CEU_FATAL_EVENTS))
    {
        ov5640_save_ceu_debug(FSP_SUCCESS);
        (void) ov5640_recover_ceu();
        return CAMERA_OV5640_ERR_CAPTURE;
    }

    return CAMERA_OV5640_OK;
}
