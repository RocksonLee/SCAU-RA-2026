/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = iic_master_rxi_isr, /* IIC0 RXI (Receive data full) */
            [1] = iic_master_txi_isr, /* IIC0 TXI (Transmit data empty) */
            [2] = iic_master_tei_isr, /* IIC0 TEI (Transmit end) */
            [3] = iic_master_eri_isr, /* IIC0 ERI (Transfer error) */
            [4] = ceu_isr, /* CEU CEUI (CEU interrupt) */
            [5] = sci_b_uart_rxi_isr, /* SCI9 RXI (Receive data full) */
            [6] = sci_b_uart_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [7] = sci_b_uart_tei_isr, /* SCI9 TEI (Transmit end) */
            [8] = sci_b_uart_eri_isr, /* SCI9 ERI (Receive error) */
            [9] = rm_ethosu_isr, /* NPU IRQ (NPU IRQ) */
            [10] = sci_b_spi_tei_isr, /* SCI4 TEI (Transmit end) */
            [11] = sci_b_spi_eri_isr, /* SCI4 ERI (Receive error) */
            [12] = dmac_int_isr, /* DMAC0 INT (DMAC0 transfer end) */
            [13] = dmac_int_isr, /* DMAC1 INT (DMAC1 transfer end) */
            [14] = sci_b_i2c_txi_isr, /* SCI3 TXI (Transmit data empty) */
            [15] = sci_b_i2c_tei_isr, /* SCI3 TEI (Transmit end) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_IIC0_RXI,GROUP0), /* IIC0 RXI (Receive data full) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_IIC0_TXI,GROUP1), /* IIC0 TXI (Transmit data empty) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_IIC0_TEI,GROUP2), /* IIC0 TEI (Transmit end) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_IIC0_ERI,GROUP3), /* IIC0 ERI (Transfer error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_CEU_CEUI,GROUP4), /* CEU CEUI (CEU interrupt) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_SCI9_RXI,GROUP5), /* SCI9 RXI (Receive data full) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TXI,GROUP6), /* SCI9 TXI (Transmit data empty) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TEI,GROUP7), /* SCI9 TEI (Transmit end) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_SCI9_ERI,GROUP0), /* SCI9 ERI (Receive error) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_NPU_IRQ,GROUP1), /* NPU IRQ (NPU IRQ) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_SCI4_TEI,GROUP2), /* SCI4 TEI (Transmit end) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_SCI4_ERI,GROUP3), /* SCI4 ERI (Receive error) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_DMAC0_INT,GROUP4), /* DMAC0 INT (DMAC0 transfer end) */
            [13] = BSP_PRV_VECT_ENUM(EVENT_DMAC1_INT,GROUP5), /* DMAC1 INT (DMAC1 transfer end) */
            [14] = BSP_PRV_VECT_ENUM(EVENT_SCI3_TXI,GROUP6), /* SCI3 TXI (Transmit data empty) */
            [15] = BSP_PRV_VECT_ENUM(EVENT_SCI3_TEI,GROUP7), /* SCI3 TEI (Transmit end) */
        };
        #endif
        #endif
