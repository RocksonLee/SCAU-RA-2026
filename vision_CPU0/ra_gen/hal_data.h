/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_ospi_b.h"
#include "r_spi_flash_api.h"
#include "r_sci_b_i2c.h"
#include "r_i2c_master_api.h"
#include "r_dmac.h"
#include "r_transfer_api.h"
#include "r_sci_b_spi.h"
#include "r_spi_api.h"
#include "r_ipc.h"
#include "r_sci_b_uart.h"
#include "r_uart_api.h"
#include "r_capture_api.h"
#include "r_ceu.h"
#include "r_iic_master.h"
#include "r_i2c_master_api.h"
FSP_HEADER
#if OSPI_B_CFG_DMAC_SUPPORT_ENABLE
#include "r_dmac.h"
#endif
#if OSPI_CFG_DOTF_SUPPORT_ENABLE
#include "r_sce_if.h"
#endif

extern const spi_flash_instance_t g_ospi0;
extern ospi_b_instance_ctrl_t g_ospi0_ctrl;
extern const spi_flash_cfg_t g_ospi0_cfg;
extern const i2c_master_cfg_t g_i2c_touch_cfg;
/* I2C on SCI Instance. */
extern const i2c_master_instance_t g_i2c_touch;
#ifndef i2c_touch_callback
void i2c_touch_callback(i2c_master_callback_args_t *p_args);
#endif

extern const sci_b_i2c_extended_cfg_t g_i2c_touch_cfg_extend;
extern sci_b_i2c_instance_ctrl_t g_i2c_touch_ctrl;
/* Transfer on DMAC Instance. */
extern const transfer_instance_t g_spi_lcd_rx_transfer;

/** Access the DMAC instance using these structures when calling API functions directly (::p_api is not used). */
extern dmac_instance_ctrl_t g_spi_lcd_rx_transfer_ctrl;
extern const transfer_cfg_t g_spi_lcd_rx_transfer_cfg;

#ifndef g_spi_lcd_rx_transfer_callback
void g_spi_lcd_rx_transfer_callback(transfer_callback_args_t *p_args);
#endif
/* Transfer on DMAC Instance. */
extern const transfer_instance_t g_spi_lcd_tx_transfer;

/** Access the DMAC instance using these structures when calling API functions directly (::p_api is not used). */
extern dmac_instance_ctrl_t g_spi_lcd_tx_transfer_ctrl;
extern const transfer_cfg_t g_spi_lcd_tx_transfer_cfg;

#ifndef g_spi_lcd_tx_transfer_callback
void g_spi_lcd_tx_transfer_callback(transfer_callback_args_t *p_args);
#endif
/** SPI on SCI Instance. */
extern const spi_instance_t g_spi_lcd;

/** Access the SCI_B_SPI instance using these structures when calling API functions directly (::p_api is not used). */
extern sci_b_spi_instance_ctrl_t g_spi_lcd_ctrl;
extern const spi_cfg_t g_spi_lcd_cfg;

/** Called by the driver when a transfer has completed or an error has occurred (Must be implemented by the user). */
#ifndef lcd_spi_callback
void lcd_spi_callback(spi_callback_args_t *p_args);
#endif
/** IPC Instance. */
extern const ipc_instance_t g_ipc0;

/** Access the IPC instance using these structures when calling API functions directly
 (::p_api is not used). */
extern ipc_instance_ctrl_t g_ipc0_ctrl;
extern const ipc_cfg_t g_ipc0_cfg;

#ifndef ipc0_callback
void ipc0_callback(ipc_callback_args_t *p_args);
#endif
/** UART on SCI Instance. */
extern const uart_instance_t g_uart9;

/** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
extern sci_b_uart_instance_ctrl_t g_uart9_ctrl;
extern const uart_cfg_t g_uart9_cfg;
extern const sci_b_uart_extended_cfg_t g_uart9_cfg_extend;

#ifndef NULL
void NULL(uart_callback_args_t *p_args);
#endif
/* CEU on CAPTURE instance */
extern const capture_instance_t g_ceu0;
/* Access the CEU instance using these structures when calling API functions directly (::p_api is not used). */
extern ceu_instance_ctrl_t g_ceu0_ctrl;
extern const capture_cfg_t g_ceu0_cfg;
#ifndef g_ceu0_user_callback
void g_ceu0_user_callback(capture_callback_args_t *p_args);
#endif
/* I2C Master on IIC Instance. */
extern const i2c_master_instance_t g_i2c_camera;

/** Access the I2C Master instance using these structures when calling API functions directly (::p_api is not used). */
extern iic_master_instance_ctrl_t g_i2c_camera_ctrl;
extern const i2c_master_cfg_t g_i2c_camera_cfg;

#ifndef i2c_camera_callback
void i2c_camera_callback(i2c_master_callback_args_t *p_args);
#endif
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
