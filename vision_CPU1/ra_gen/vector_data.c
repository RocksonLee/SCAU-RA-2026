/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = canfd_error_isr, /* CAN0 CHERR (Channel  error) */
            [1] = canfd_channel_tx_isr, /* CAN0 TX (Transmit interrupt) */
            [2] = canfd_common_fifo_rx_isr, /* CAN0 COMFRX (Common FIFO receive interrupt) */
            [3] = canfd_error_isr, /* CAN GLERR (Global error) */
            [4] = canfd_rx_fifo_isr, /* CAN RXF (Global receive FIFO interrupt) */
            [5] = ipc_isr, /* IPC IRQ0 (CPU Mutual Interrupt 0) */
            [6] = sci_b_uart_rxi_isr, /* SCI0 RXI (Receive data full) */
            [7] = sci_b_uart_txi_isr, /* SCI0 TXI (Transmit data empty) */
            [8] = sci_b_uart_tei_isr, /* SCI0 TEI (Transmit end) */
            [9] = sci_b_uart_eri_isr, /* SCI0 ERI (Receive error) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_CAN0_CHERR,GROUP0), /* CAN0 CHERR (Channel  error) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_CAN0_TX,GROUP1), /* CAN0 TX (Transmit interrupt) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_CAN0_COMFRX,GROUP2), /* CAN0 COMFRX (Common FIFO receive interrupt) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_CAN_GLERR,GROUP3), /* CAN GLERR (Global error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_CAN_RXF,GROUP4), /* CAN RXF (Global receive FIFO interrupt) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_IPC_IRQ0,GROUP5), /* IPC IRQ0 (CPU Mutual Interrupt 0) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI0_RXI,GROUP6), /* SCI0 RXI (Receive data full) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SCI0_TXI,GROUP7), /* SCI0 TXI (Transmit data empty) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_SCI0_TEI,GROUP0), /* SCI0 TEI (Transmit end) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_SCI0_ERI,GROUP1), /* SCI0 ERI (Receive error) */
        };
        #endif
        #endif
