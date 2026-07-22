#include "Uart9_thread.h"
#include "hardware/drv_uart.h"
/* Uart9_thread entry function */
extern void send_result_to_can(float result);
/* pvParameters contains TaskHandle_t */
void Uart9_thread_entry(void *pvParameters) {
	FSP_PARAMETER_NOT_USED(pvParameters);

	if (FSP_SUCCESS != drv_uart_init())
	{
		vTaskDelete(NULL);
	}

	while (1)
	{
		static uint8_t uart9_test_message[] = "UART9 alive\r\n";
		(void) drv_uart9_send(uart9_test_message, sizeof(uart9_test_message) - 1U);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
