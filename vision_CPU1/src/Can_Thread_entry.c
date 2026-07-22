#include <Can_Thread.h>
#include "hardware/canfd0.h"
/* New Thread entry function */
/* pvParameters contains TaskHandle_t */
void Can_Thread_entry(void *pvParameters) {
	FSP_PARAMETER_NOT_USED(pvParameters);
	extern volatile float g_k230_pos_x;
	extern volatile float g_k230_pos_y;
	extern volatile bool  g_k230_data_ready;
	extern int inverse_kinematics_5dof(double x, double y, double z, int elbow_dir,
	                                  double *q1, double *q2, double *q3, double *q5);

	CANFD0_Init();
	while (1)
	{
		if (g_k230_data_ready)
		{
			float x;
			float y;
			double q1_angle;
			double q2_angle;
			double q3_angle;
			double q5_angle;

			taskENTER_CRITICAL();
			x = g_k230_pos_x;
			y = g_k230_pos_y;
			g_k230_data_ready = false;
			taskEXIT_CRITICAL();

			if (inverse_kinematics_5dof(x, y, 320.0, 1,
			                                &q1_angle, &q2_angle,
			                                &q3_angle, &q5_angle))
			{
				CANFD0_Operation_1((int32_t) q1_angle);
				CANFD0_Operation_2((int32_t) q2_angle);
				CANFD0_Operation_3((int32_t) q3_angle);
				CANFD0_Operation_5((int32_t) q5_angle);
				run();
			}
		}

		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
