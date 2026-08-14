#include <Camera_thread.h>
#include "hardware/camera_stream.h"

/* Camera_thread entry function */
/* pvParameters contains TaskHandle_t */
void Camera_thread_entry(void *pvParameters)
{
	FSP_PARAMETER_NOT_USED(pvParameters);

	R_BSP_SecondaryCoreStart();

	if (FSP_SUCCESS != g_ipc0.p_api->open(g_ipc0.p_ctrl, g_ipc0.p_cfg))
	{
		vTaskDelete(NULL);
	}

	camera_stream_task();
}
