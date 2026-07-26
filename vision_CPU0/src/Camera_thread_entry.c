#include <Camera_thread.h>
#include "hardware/camera_stream.h"

/* Camera_thread entry function */
/* pvParameters contains TaskHandle_t */
void Camera_thread_entry(void *pvParameters)
{
	FSP_PARAMETER_NOT_USED(pvParameters);
	// vTaskDelay(pdMS_TO_TICKS(3000));
	R_BSP_SecondaryCoreStart();

	if (FSP_SUCCESS != g_ipc0.p_api->open(g_ipc0.p_ctrl, g_ipc0.p_cfg))
	{
		vTaskDelete(NULL);
	}

	camera_stream_task();
}
