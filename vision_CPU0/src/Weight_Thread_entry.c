#include "Weight_Thread.h"

#include "hardware/fruit_ui.h"
#include "hardware/hx711.h"

#define HX711_TARE_RETRY_MS      (1000U)

/* Weight_Thread entry function */
/* pvParameters contains TaskHandle_t */
void Weight_Thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    int32_t offset = 0;
    hx711_prepare_status_t prepare_status = HX711_PREPARE_NOT_READY;

    fruit_ui_set_weight(0, false);

    while (HX711_PREPARE_OK != prepare_status)
    {
        prepare_status = hx711_prepare(&offset);

        if (HX711_PREPARE_OK != prepare_status)
        {
            vTaskDelay(pdMS_TO_TICKS(HX711_TARE_RETRY_MS));
        }
    }

    while (1)
    {
        int32_t raw = 0;

        if (hx711_read_filtered(&raw))
        {
            int32_t const net = raw - offset;
            int32_t const weight_0p1g = hx711_net_to_weight_0p1g(net);
            fruit_ui_set_weight(weight_0p1g, true);
        }
        else
        {
            fruit_ui_set_weight(0, false);
        }
    }
}
