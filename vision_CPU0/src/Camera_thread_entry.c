#include <Camera_thread.h>
#include "hardware/camera_stream.h"

#define CAMERA_FEATURE_ENABLED    (0)

/* Camera_thread entry function */
/* pvParameters contains TaskHandle_t */
void Camera_thread_entry(void *pvParameters)
{
	FSP_PARAMETER_NOT_USED(pvParameters);

#if CAMERA_FEATURE_ENABLED
	R_BSP_SecondaryCoreStart();

	if (FSP_SUCCESS != g_ipc0.p_api->open(g_ipc0.p_ctrl, g_ipc0.p_cfg))
	{
		vTaskDelete(NULL);
	}

	camera_stream_task();
#else
	/* Keep UART9 (J-Link VCOM) available exclusively for Weight_Thread.
	 * Camera, NPU, IPC and all camera debug output are temporarily disabled. */
	(void) camera_debug_uart_init();

	while (1)
	{
		vTaskDelay(portMAX_DELAY);
	}
#endif
}
