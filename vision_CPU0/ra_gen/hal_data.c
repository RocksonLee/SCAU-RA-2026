/* generated HAL source file - do not edit */
#include "hal_data.h"
#include "r_sci_b_i2c_cfg.h"
sci_b_i2c_instance_ctrl_t g_i2c_touch_ctrl;
const sci_b_i2c_extended_cfg_t g_i2c_touch_cfg_extend =
		{
				/* Actual calculated bitrate: 99985. Actual SDA delay: 248.00 ns. */.clock_settings.clk_divisor_value =
						0, .clock_settings.brr_value = 37,
				.clock_settings.mddr_value = 249,
				.clock_settings.bitrate_modulation = true,
				.clock_settings.cycles_value = 31, .clock_settings.snfr_value =
						(1), .clock_settings.clock_source =
						SCI_B_I2C_CLOCK_SOURCE_PCLK };

const i2c_master_cfg_t g_i2c_touch_cfg = { .channel = 3, .rate =
		I2C_MASTER_RATE_STANDARD, .slave = 0x38, .addr_mode =
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
		.p_callback = i2c_touch_callback, .p_context = NULL,
#if defined(VECTOR_NUMBER_SCI3_RXI) && SCI_B_I2C_CFG_DTC_ENABLE
    .rxi_irq             = VECTOR_NUMBER_SCI3_RXI,
#else
		.rxi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI3_TXI)
    .txi_irq             = VECTOR_NUMBER_SCI3_TXI,
#else
		.txi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI3_TEI)
    .tei_irq             = VECTOR_NUMBER_SCI3_TEI,
#else
		.tei_irq = FSP_INVALID_VECTOR,
#endif
		.ipl = (12), /* (BSP_IRQ_DISABLED) is unused */
		.p_extend = &g_i2c_touch_cfg_extend, };
/* Instance structure to use this module. */
const i2c_master_instance_t g_i2c_touch = { .p_ctrl = &g_i2c_touch_ctrl,
		.p_cfg = &g_i2c_touch_cfg, .p_api = &g_i2c_master_on_sci_b };

dmac_instance_ctrl_t g_spi_lcd_rx_transfer_ctrl;
transfer_info_t g_spi_lcd_rx_transfer_info =
		{ .transfer_settings_word_b.dest_addr_mode =
				TRANSFER_ADDR_MODE_INCREMENTED,
				.transfer_settings_word_b.repeat_area =
						TRANSFER_REPEAT_AREA_DESTINATION,
				.transfer_settings_word_b.irq = TRANSFER_IRQ_END,
				.transfer_settings_word_b.chain_mode =
						TRANSFER_CHAIN_MODE_DISABLED,
				.transfer_settings_word_b.src_addr_mode =
						TRANSFER_ADDR_MODE_FIXED,
				.transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE,
				.transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL, .p_dest =
						(void*) NULL, .p_src = (void const*) NULL, .num_blocks =
						0, .length = 0, };
const dmac_extended_cfg_t g_spi_lcd_rx_transfer_extend = { .offset = 1,
		.src_buffer_size = 1,
#if defined(VECTOR_NUMBER_DMAC1_INT)
    .irq                 = VECTOR_NUMBER_DMAC1_INT,
#else
		.irq = FSP_INVALID_VECTOR,
#endif
		.ipl = (12), .channel = 1, .p_callback = g_spi_lcd_rx_transfer_callback,
		.p_context = NULL, .activation_source = ELC_EVENT_SCI4_RXI, };
const transfer_cfg_t g_spi_lcd_rx_transfer_cfg =
		{ .p_info = &g_spi_lcd_rx_transfer_info, .p_extend =
				&g_spi_lcd_rx_transfer_extend, };
/* Instance structure to use this module. */
const transfer_instance_t g_spi_lcd_rx_transfer = { .p_ctrl =
		&g_spi_lcd_rx_transfer_ctrl, .p_cfg = &g_spi_lcd_rx_transfer_cfg,
		.p_api = &g_transfer_on_dmac };

dmac_instance_ctrl_t g_spi_lcd_tx_transfer_ctrl;
transfer_info_t g_spi_lcd_tx_transfer_info =
		{ .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
				.transfer_settings_word_b.repeat_area =
						TRANSFER_REPEAT_AREA_SOURCE,
				.transfer_settings_word_b.irq = TRANSFER_IRQ_END,
				.transfer_settings_word_b.chain_mode =
						TRANSFER_CHAIN_MODE_DISABLED,
				.transfer_settings_word_b.src_addr_mode =
						TRANSFER_ADDR_MODE_INCREMENTED,
				.transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE,
				.transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL, .p_dest =
						(void*) NULL, .p_src = (void const*) NULL, .num_blocks =
						0, .length = 0, };
const dmac_extended_cfg_t g_spi_lcd_tx_transfer_extend = { .offset = 1,
		.src_buffer_size = 1,
#if defined(VECTOR_NUMBER_DMAC0_INT)
    .irq                 = VECTOR_NUMBER_DMAC0_INT,
#else
		.irq = FSP_INVALID_VECTOR,
#endif
		.ipl = (12), .channel = 0, .p_callback = g_spi_lcd_tx_transfer_callback,
		.p_context = NULL, .activation_source = ELC_EVENT_SCI4_TXI, };
const transfer_cfg_t g_spi_lcd_tx_transfer_cfg =
		{ .p_info = &g_spi_lcd_tx_transfer_info, .p_extend =
				&g_spi_lcd_tx_transfer_extend, };
/* Instance structure to use this module. */
const transfer_instance_t g_spi_lcd_tx_transfer = { .p_ctrl =
		&g_spi_lcd_tx_transfer_ctrl, .p_cfg = &g_spi_lcd_tx_transfer_cfg,
		.p_api = &g_transfer_on_dmac };
#define RA_NOT_DEFINED (UINT32_MAX)
#if (RA_NOT_DEFINED) != (1)

/* If the TX transfer module is DMAC, define a DMAC TX transfer callback. */
#include "r_dmac.h"
extern void sci_b_spi_tx_dmac_callback(
		sci_b_spi_instance_ctrl_t const *const p_ctrl);

void g_spi_lcd_tx_transfer_callback(dmac_callback_args_t *p_args) {
	FSP_PARAMETER_NOT_USED(p_args);
	sci_b_spi_tx_dmac_callback(&g_spi_lcd_ctrl);
}
#endif

#if (RA_NOT_DEFINED) != (1)

/* If the RX transfer module is DMAC, define a DMAC RX transfer callback. */
#include "r_dmac.h"
extern void sci_b_spi_rx_dmac_callback(
		sci_b_spi_instance_ctrl_t const *const p_ctrl);

void g_spi_lcd_rx_transfer_callback(dmac_callback_args_t *p_args) {
	FSP_PARAMETER_NOT_USED(p_args);
	sci_b_spi_rx_dmac_callback(&g_spi_lcd_ctrl);
}
#endif
#undef RA_NOT_DEFINED

sci_b_spi_instance_ctrl_t g_spi_lcd_ctrl;

/** SPI extended configuration */
const sci_b_spi_extended_cfg_t g_spi_lcd_cfg_extend = { .clk_div = {
/* Actual calculated bitrate: 7812500. */.cks = 0, .brr = 7, .bgdm = 1, },
		.clock_source = (sci_b_spi_clock_source_t) 1, .rx_sampling_delay =
				SCI_B_SPI_RX_SAMPLING_DELAY_CYCLES_0, .tx_fifo_trigger =
				SCI_B_SPI_TX_FIFO_TRIGGER_DISABLED, };

const spi_cfg_t g_spi_lcd_cfg = { .channel = 4, .operating_mode =
		SPI_MODE_MASTER, .clk_phase = SPI_CLK_PHASE_EDGE_ODD, .clk_polarity =
		SPI_CLK_POLARITY_LOW, .mode_fault = SPI_MODE_FAULT_ERROR_DISABLE,
		.bit_order = SPI_BIT_ORDER_MSB_FIRST,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == g_spi_lcd_tx_transfer)
    .p_transfer_tx   = NULL,
#else
		.p_transfer_tx = &g_spi_lcd_tx_transfer,
#endif
#if (RA_NOT_DEFINED == g_spi_lcd_rx_transfer)
    .p_transfer_rx   = NULL,
#else
		.p_transfer_rx = &g_spi_lcd_rx_transfer,
#endif
#undef RA_NOT_DEFINED
		.p_callback = lcd_spi_callback, .p_context = NULL,
#if defined(VECTOR_NUMBER_SCI4_RXI)
    .rxi_irq         = VECTOR_NUMBER_SCI4_RXI,
#else
		.rxi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI4_TXI)
    .txi_irq         = VECTOR_NUMBER_SCI4_TXI,
#else
		.txi_irq = FSP_INVALID_VECTOR,
#endif
		.tei_irq = VECTOR_NUMBER_SCI4_TEI, .eri_irq = VECTOR_NUMBER_SCI4_ERI,
		.rxi_ipl = (BSP_IRQ_DISABLED), .txi_ipl = (BSP_IRQ_DISABLED), .tei_ipl =
				(12), .eri_ipl = (12), .p_extend = &g_spi_lcd_cfg_extend, };
/* Instance structure to use this module. */
const spi_instance_t g_spi_lcd = { .p_ctrl = &g_spi_lcd_ctrl, .p_cfg =
		&g_spi_lcd_cfg, .p_api = &g_spi_on_sci_b };
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
		.image_area_size = 640 * 480 * 2, .interrupts_enabled = 0
				| R_CEU_CEIER_CPEIE_Msk | 0 | R_CEU_CEIER_VDIE_Msk
				| R_CEU_CEIER_CDTOFIE_Msk | 0 | 0 | R_CEU_CEIER_VBPIE_Msk
				| R_CEU_CEIER_NHDIE_Msk | R_CEU_CEIER_NVDIE_Msk,
		.ceu_ipl = (12), .ceu_irq = VECTOR_NUMBER_CEU_CEUI, };

const capture_cfg_t g_ceu0_cfg = { .x_capture_pixels = 640, .y_capture_pixels =
		480, .x_capture_start_pixel = 0, .y_capture_start_pixel = 0,
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
