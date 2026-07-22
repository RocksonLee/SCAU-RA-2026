/* generated HAL source file - do not edit */
#include "hal_data.h"
ipc_instance_ctrl_t g_ipc0_ctrl;

/** IPC configuration */
const ipc_cfg_t g_ipc0_cfg = { .channel = 0, .p_callback = ipc0_callback,
#if defined(NULL)
                .p_context = NULL,
#else
		.p_context = (void*) &NULL,
#endif
		.ipl = (10),
#if defined(VECTOR_NUMBER_IPC_IRQ0)
                .irq = VECTOR_NUMBER_IPC_IRQ0,
#else
		.irq = FSP_INVALID_VECTOR,
#endif
		};

/* Instance structure to use this module. */
const ipc_instance_t g_ipc0 = { .p_ctrl = &g_ipc0_ctrl, .p_cfg = &g_ipc0_cfg,
		.p_api = &g_ipc_on_ipc };
/* Nominal and Data bit timing configuration */

can_bit_timing_cfg_t g_canfd0_bit_timing_cfg = {
/* Actual bitrate: 500000 Hz. Actual sample point: 75 %. */
.baud_rate_prescaler = 1, .time_segment_1 = 119, .time_segment_2 = 40,
		.synchronization_jump_width = 4 };

#if BSP_FEATURE_CANFD_FD_SUPPORT
can_bit_timing_cfg_t g_canfd0_data_timing_cfg =
{
    /* Actual bitrate: 2000000 Hz. Actual sample point: 75 %. */
    .baud_rate_prescaler = 1,
    .time_segment_1 = 29,
    .time_segment_2 = 10,
    .synchronization_jump_width = 4
};
#endif

extern const canfd_afl_entry_t p_canfd0_afl[CANFD_CFG_AFL_CH0_RULE_NUM];

#define CANFD_CFG_COMMONFIFO0 (((0) << R_CANFD_CFDCFCC_CFE_Pos) | \
                                        ((0) << R_CANFD_CFDCFCC_CFRXIE_Pos) | \
                                        ((0) << R_CANFD_CFDCFCC_CFTXIE_Pos) | \
                                        ((0) << R_CANFD_CFDCFCC_CFPLS_Pos) | \
                                        ((0) << R_CANFD_CFDCFCC_CFM_Pos) | \
                                        ((0) << R_CANFD_CFDCFCC_CFITSS_Pos) | \
                                        ((0) << R_CANFD_CFDCFCC_CFITR_Pos) | \
                                        ((0)  << R_CANFD_CFDCFCC_CFIM_Pos) | \
                                        ((3U) << R_CANFD_CFDCFCC_CFIGCV_Pos) | \
                                        ((0) << R_CANFD_CFDCFCC_CFTML_Pos) | \
                                        ((3) << R_CANFD_CFDCFCC_CFDC_Pos) | \
                                        (0 << R_CANFD_CFDCFCC_CFITT_Pos))

/* Buffer RAM used: 340 bytes */
canfd_global_cfg_t g_canfd0_global_cfg = { .global_interrupts =
		(R_CANFD_CFDGCTR_DEIE_Msk | R_CANFD_CFDGCTR_MEIE_Msk | 0x3),
		.global_config = ((R_CANFD_CFDGCFG_TPRI_Msk) | (R_CANFD_CFDGCFG_DCE_Msk)
				| (BSP_CFG_CANFDCLK_SOURCE == BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC ?
						R_CANFD_CFDGCFG_DCS_Msk : 0U) | (0)
				| ((0) << R_CANFD_CFDGCFG_ITRCP_Pos)), .rx_mb_config = (1
				| ((0) << R_CANFD_CFDRMNB_RMPLS_Pos)), .global_err_ipl =
				CANFD_CFG_GLOBAL_ERR_IPL, .rx_fifo_ipl = CANFD_CFG_RX_FIFO_IPL,
		.rx_fifo_config = { ((3U) << R_CANFD_CFDRFCC_RFIGCV_Pos)
				| ((3) << R_CANFD_CFDRFCC_RFDC_Pos)
				| ((0) << R_CANFD_CFDRFCC_RFPLS_Pos)
				| ((R_CANFD_CFDRFCC_RFIE_Msk | R_CANFD_CFDRFCC_RFIM_Msk))
				| ((1)), ((3U) << R_CANFD_CFDRFCC_RFIGCV_Pos)
				| ((3) << R_CANFD_CFDRFCC_RFDC_Pos)
				| ((0) << R_CANFD_CFDRFCC_RFPLS_Pos)
				| ((R_CANFD_CFDRFCC_RFIE_Msk | R_CANFD_CFDRFCC_RFIM_Msk))
				| ((0)) }, .common_fifo_config = {
		CANFD_CFG_COMMONFIFO0 } };

canfd_extended_cfg_t g_canfd0_extended_cfg = { .p_afl = p_canfd0_afl,
		.txmb_txi_enable = ((1ULL << 0) | 0ULL), .error_interrupts =
				(R_CANFD_CFDC_CTR_EWIE_Msk | R_CANFD_CFDC_CTR_EPIE_Msk
						| R_CANFD_CFDC_CTR_BOEIE_Msk
						| R_CANFD_CFDC_CTR_BORIE_Msk | R_CANFD_CFDC_CTR_OLIE_Msk
						| 0U),
#if BSP_FEATURE_CANFD_FD_SUPPORT
    .p_data_timing      = &g_canfd0_data_timing_cfg,
#else
		.p_data_timing = NULL,
#endif
		.delay_compensation = (1), .p_global_cfg = &g_canfd0_global_cfg, };

canfd_instance_ctrl_t g_canfd0_ctrl;
const can_cfg_t g_canfd0_cfg = { .channel = 0, .p_bit_timing =
		&g_canfd0_bit_timing_cfg, .p_callback = canfd0_callback, .p_extend =
		&g_canfd0_extended_cfg, .p_context = NULL, .ipl = (10),
#if defined(VECTOR_NUMBER_CAN0_COMFRX)
    .rx_irq             = VECTOR_NUMBER_CAN0_COMFRX,
#else
		.rx_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_CAN0_TX)
    .tx_irq             = VECTOR_NUMBER_CAN0_TX,
#else
		.tx_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_CAN0_CHERR)
    .error_irq             = VECTOR_NUMBER_CAN0_CHERR,
#else
		.error_irq = FSP_INVALID_VECTOR,
#endif
		};
/* Instance structure to use this module. */
const can_instance_t g_canfd0 = { .p_ctrl = &g_canfd0_ctrl, .p_cfg =
		&g_canfd0_cfg, .p_api = &g_canfd_on_canfd };
void g_hal_init(void) {
	g_common_init();
}
