#include "Weight_Thread.h"

#include <stdbool.h>
#include <stdio.h>

#include "hardware/camera_stream.h"
#include "hardware/hx711.h"

#define HX711_REPORT_PERIOD_MS   (200U)

/* Weight_Thread entry function */
/* pvParameters contains TaskHandle_t */
void Weight_Thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    char line[96];
    int32_t offset = 0;

    if (!hx711_prepare(&offset))
    {
        (void) camera_debug_send_text("HX711_ERR not_ready\r\n");
    }
    else
    {
        int const count = snprintf(line, sizeof(line),
                                   "HX711_TARE offset=%ld\r\n", (long) offset);
        if ((count > 0) && ((size_t) count < sizeof(line)))
        {
            (void) camera_debug_send_text(line);
        }
    }


    while (1)
    {
        int32_t raw = 0;

        if (hx711_read_filtered(&raw))
        {
            int32_t const net = raw - offset;
            int32_t const weight_0p1g = hx711_net_to_weight_0p1g(net);
            int32_t const weight_abs = (weight_0p1g < 0) ? -weight_0p1g : weight_0p1g;
            char const * const sign = (weight_0p1g < 0) ? "-" : "";
            int const count = snprintf(line, sizeof(line),
                                       "HX711 raw=%ld net=%ld weight=%s%ld.%ldg\r\n",
                                       (long) raw,
                                       (long) net,
                                       sign,
                                       (long) (weight_abs / 10),
                                       (long) (weight_abs % 10));
            if ((count > 0) && ((size_t) count < sizeof(line)))
            {
                (void) camera_debug_send_text(line);
            }

            vTaskDelay(pdMS_TO_TICKS(HX711_REPORT_PERIOD_MS));
        }
        else
        {
            (void) camera_debug_send_text("HX711_ERR filter_timeout\r\n");
        }
    }
}
