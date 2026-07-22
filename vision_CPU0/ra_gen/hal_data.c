/* generated HAL source file - do not edit */
#include "hal_data.h"
ipc_instance_ctrl_t g_ipc0_ctrl;

/** IPC configuration */
const ipc_cfg_t g_ipc0_cfg = { .channel = 0, .p_callback = NULL,
#if defined(NULL)
                .p_context = NULL,
#else
		.p_context = (void*) &NULL,
#endif
		.ipl = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_IPC_IRQ0)
                .irq = VECTOR_NUMBER_IPC_IRQ0,
#else
		.irq = FSP_INVALID_VECTOR,
#endif
		};

/* Instance structure to use this module. */
const ipc_instance_t g_ipc0 = { .p_ctrl = &g_ipc0_ctrl, .p_cfg = &g_ipc0_cfg,
		.p_api = &g_ipc_on_ipc };
sci_b_uart_instance_ctrl_t g_uart9_ctrl;

sci_b_baud_setting_t g_uart9_baud_setting = {
/* Baud rate calculated with 0.160% error. */.baudrate_bits_b.abcse = 0,
		.baudrate_bits_b.abcs = 0, .baudrate_bits_b.bgdm = 1,
		.baudrate_bits_b.cks = 0, .baudrate_bits_b.brr = 64,
		.baudrate_bits_b.mddr = (uint8_t) 256, .baudrate_bits_b.brme = false };

/** UART extended configuration for UARTonSCI HAL driver */
const sci_b_uart_extended_cfg_t g_uart9_cfg_extend = { .clock =
		SCI_B_UART_CLOCK_INT,
		.rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE, .noise_cancel =
				SCI_B_UART_NOISE_CANCELLATION_DISABLE, .rx_fifo_trigger =
				SCI_B_UART_RX_FIFO_TRIGGER_MAX, .p_baud_setting =
				&g_uart9_baud_setting, .flow_control =
				SCI_B_UART_FLOW_CONTROL_RTS,
#if 0xFF != 0xFF
                .flow_control_pin       = BSP_IO_PORT_FF_PIN_0xFF,
                #else
		.flow_control_pin = (bsp_io_port_pin_t) UINT16_MAX,
#endif
		.rs485_setting = { .enable = SCI_B_UART_RS485_DISABLE, .polarity =
				SCI_B_UART_RS485_DE_POLARITY_HIGH, .assertion_time = 1,
				.negation_time = 1, }, .delay_cycles = 0, };

/** UART interface configuration */
const uart_cfg_t g_uart9_cfg = { .channel = 9, .data_bits = UART_DATA_BITS_8,
		.parity = UART_PARITY_OFF, .stop_bits = UART_STOP_BITS_1, .p_callback =
				NULL, .p_context = NULL, .p_extend = &g_uart9_cfg_extend,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
		.p_transfer_tx = NULL,
#else
                .p_transfer_tx       = &RA_NOT_DEFINED,
#endif
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
		.p_transfer_rx = NULL,
#else
                .p_transfer_rx       = &RA_NOT_DEFINED,
#endif
#undef RA_NOT_DEFINED
		.rxi_ipl = (12), .txi_ipl = (12), .tei_ipl = (12), .eri_ipl = (12),
#if defined(VECTOR_NUMBER_SCI9_RXI)
                .rxi_irq             = VECTOR_NUMBER_SCI9_RXI,
#else
		.rxi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI9_TXI)
                .txi_irq             = VECTOR_NUMBER_SCI9_TXI,
#else
		.txi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI9_TEI)
                .tei_irq             = VECTOR_NUMBER_SCI9_TEI,
#else
		.tei_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI9_ERI)
                .eri_irq             = VECTOR_NUMBER_SCI9_ERI,
#else
		.eri_irq = FSP_INVALID_VECTOR,
#endif
		};

/* Instance structure to use this module. */
const uart_instance_t g_uart9 = { .p_ctrl = &g_uart9_ctrl,
		.p_cfg = &g_uart9_cfg, .p_api = &g_uart_on_sci_b };
ceu_instance_ctrl_t g_ceu0_ctrl;
const ceu_extended_cfg_t g_ceu0_extended_cfg = { .capture_format =
		CEU_CAPTURE_FORMAT_DATA_SYNCHRONOUS, .input_order =
		CEU_INPUT_ORDER_CB0Y0CR0Y1, .output_format = CEU_OUTPUT_FORMAT_YCBCR422,
		.data_bus_width = CEU_DATA_BUS_SIZE_8_BIT, .edge_info.dsel = 0,
		.edge_info.hdsel = 0, .edge_info.vdsel = 0, .hsync_polarity =
				CEU_HSYNC_POLARITY_HIGH, .vsync_polarity =
				CEU_VSYNC_POLARITY_HIGH, .byte_swapping = { .swap_8bit_units =
				(0x1 | 0x2 | 0x4 | 0x0) >> 0x00 & 0x01, .swap_16bit_units = (0x1
				| 0x2 | 0x4 | 0x0) >> 0x01 & 0x01, .swap_32bit_units = (0x1
				| 0x2 | 0x4 | 0x0) >> 0x02 & 0x01, }, .burst_mode =
				CEU_BURST_TRANSFER_MODE_X8, .scale_down_factor = 0x0U,
		.h_output_size = 0, .v_output_size = 0,
		.image_area_size = 320 * 240 * 2, .interrupts_enabled = 0
				| R_CEU_CEIER_CPEIE_Msk | 0 | R_CEU_CEIER_VDIE_Msk
				| R_CEU_CEIER_CDTOFIE_Msk | 0 | 0 | R_CEU_CEIER_VBPIE_Msk
				| R_CEU_CEIER_NHDIE_Msk | R_CEU_CEIER_NVDIE_Msk,
		.ceu_ipl = (12), .ceu_irq = VECTOR_NUMBER_CEU_CEUI, };

const capture_cfg_t g_ceu0_cfg = { .x_capture_pixels = 320, .y_capture_pixels =
		240, .x_capture_start_pixel = 0, .y_capture_start_pixel = 0,
		.bytes_per_pixel = 2, .p_callback = g_ceu0_user_callback, .p_context =
				(void*) NULL, .p_extend = &g_ceu0_extended_cfg, };

const capture_instance_t g_ceu0 = { .p_ctrl = &g_ceu0_ctrl,
		.p_cfg = &g_ceu0_cfg, .p_api = &g_ceu_on_capture, };
iic_master_instance_ctrl_t g_i2c_camera_ctrl;
const iic_master_extended_cfg_t g_i2c_camera_extend =
		{ .timeout_mode = IIC_MASTER_TIMEOUT_MODE_SHORT, .timeout_scl_low =
				IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED, .smbus_operation = 0,
				/* Actual calculated bitrate: 97809. Actual calculated duty cycle: 49%. */.clock_settings.brl_value =
						17, .clock_settings.brh_value = 16,
				.clock_settings.cks_value = 4, .clock_settings.sddl_value = 0,
				.clock_settings.dlcs_value = 0, };
const i2c_master_cfg_t g_i2c_camera_cfg = { .channel = 0, .rate =
		I2C_MASTER_RATE_STANDARD, .slave = 0x3C, .addr_mode =
		I2C_MASTER_ADDR_MODE_7BIT,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
		.p_transfer_tx = NULL,
#else
                .p_transfer_tx       = &RA_NOT_DEFINED,
#endif
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
		.p_transfer_rx = NULL,
#else
                .p_transfer_rx       = &RA_NOT_DEFINED,
#endif
#undef RA_NOT_DEFINED
		.p_callback = i2c_camera_callback, .p_context = NULL,
#if defined(VECTOR_NUMBER_IIC0_RXI)
    .rxi_irq             = VECTOR_NUMBER_IIC0_RXI,
#else
		.rxi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_IIC0_TXI)
    .txi_irq             = VECTOR_NUMBER_IIC0_TXI,
#else
		.txi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_IIC0_TEI)
    .tei_irq             = VECTOR_NUMBER_IIC0_TEI,
#else
		.tei_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_IIC0_ERI)
    .eri_irq             = VECTOR_NUMBER_IIC0_ERI,
#else
		.eri_irq = FSP_INVALID_VECTOR,
#endif
		.ipl = (12), .p_extend = &g_i2c_camera_extend, };
/* Instance structure to use this module. */
const i2c_master_instance_t g_i2c_camera = { .p_ctrl = &g_i2c_camera_ctrl,
		.p_cfg = &g_i2c_camera_cfg, .p_api = &g_i2c_master_on_iic };
void g_hal_init(void) {
	g_common_init();
}
