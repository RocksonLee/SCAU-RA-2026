#include "Uart9_thread.h"
#include "hardware/drv_uart.h"

#define IPC_COORDINATE_HEADER    (0x434F4F52U)

/* Uart9_thread entry function */
/* pvParameters contains TaskHandle_t */
void Uart9_thread_entry(void *pvParameters) {
	FSP_PARAMETER_NOT_USED(pvParameters);

	R_BSP_SecondaryCoreStart();

	if (FSP_SUCCESS != g_ipc0.p_api->open(g_ipc0.p_ctrl, g_ipc0.p_cfg))
	{
		vTaskDelete(NULL);
	}

	if (FSP_SUCCESS != drv_uart_init())
	{
		vTaskDelete(NULL);
	}

	TickType_t last_uart9_test = xTaskGetTickCount();
	static uint8_t uart9_test_message[] = "UART9 alive\r\n";

	while (1)
	{
		TickType_t const now = xTaskGetTickCount();
		if ((now - last_uart9_test) >= pdMS_TO_TICKS(1000))
		{
			(void) drv_uart9_send(uart9_test_message, sizeof(uart9_test_message) - 1U);
			last_uart9_test = now;
		}

		if (g_k230_data_ready)
		{
			union
			{
				float    value;
				uint32_t bits;
			} x, y;

			taskENTER_CRITICAL();
			x.value = g_k230_pos_x;
			y.value = g_k230_pos_y;
			g_k230_data_ready = false;
			taskEXIT_CRITICAL();

			uint32_t const messages[] = {IPC_COORDINATE_HEADER, x.bits, y.bits};
			for (uint32_t i = 0; i < (sizeof(messages) / sizeof(messages[0])); i++)
			{
				fsp_err_t err;
				do
				{
					err = g_ipc0.p_api->messageSend(g_ipc0.p_ctrl, messages[i]);
					if (FSP_ERR_OVERFLOW == err)
					{
						vTaskDelay(pdMS_TO_TICKS(1));
					}
				} while (FSP_ERR_OVERFLOW == err);

				if (FSP_SUCCESS != err)
				{
					__BKPT();
				}
			}
		}

		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
