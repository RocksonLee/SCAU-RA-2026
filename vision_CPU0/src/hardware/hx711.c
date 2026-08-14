#include "hx711.h"

#include "hal_data.h"
#include "bsp_pin_cfg.h"
#include "FreeRTOS.h"
#include "task.h"

#define HX711_BITS             (24U)
#define HX711_SIGN_BIT         (0x00800000UL)
#define HX711_SIGN_EXTENSION   (0xFF000000UL)
#define HX711_WARMUP_MS        (750U)
#define HX711_TARE_WINDOW_SAMPLES (16U)
#define HX711_TARE_TIMEOUT_MS  (5000U)
#define HX711_TARE_MAX_SPAN    (600L) /* About 0.83 g peak-to-peak at the current calibration. */
#define HX711_FILTER_SAMPLES   (16U)
#define HX711_READY_POLL_MS    (5U)
#define HX711_FILTER_TIMEOUT_MS (5000U)
#define HX711_COUNTS_PER_10G   (7192L) /* 719.2 counts/g: net ~= 359600 at 500 g. */
#define HX711_ZERO_DEADBAND    (40L)

static void hx711_delay_1us(void)
{
    R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MICROSECONDS);
}

void hx711_init(void)
{
    /* PD_SCK must remain low while HX711 is converting. */
    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, HX711_SCK, BSP_IO_LEVEL_LOW);
}

bool hx711_is_ready(void)
{
    bsp_io_level_t level = BSP_IO_LEVEL_HIGH;

    if (FSP_SUCCESS != g_ioport.p_api->pinRead(g_ioport.p_ctrl, HX711_DT, &level))
    {
        return false;
    }

    return (BSP_IO_LEVEL_LOW == level);
}

bool hx711_read(int32_t * p_raw)
{
    if ((NULL == p_raw) || !hx711_is_ready())
    {
        return false;
    }

    uint32_t value = 0U;

    /* Keep the 25 clock pulses contiguous.  In particular, PD_SCK must not
     * remain high for 60 us or longer, which would power down the HX711. */
    taskENTER_CRITICAL();

    for (uint32_t bit = 0U; bit < HX711_BITS; bit++)
    {
        bsp_io_level_t data = BSP_IO_LEVEL_LOW;

        (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, HX711_SCK, BSP_IO_LEVEL_HIGH);
        hx711_delay_1us();
        (void) g_ioport.p_api->pinRead(g_ioport.p_ctrl, HX711_DT, &data);

        value = (value << 1U) | ((BSP_IO_LEVEL_HIGH == data) ? 1U : 0U);

        (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, HX711_SCK, BSP_IO_LEVEL_LOW);
        hx711_delay_1us();
    }

    /* Pulse 25 selects channel A with gain 128 for the next conversion. */
    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, HX711_SCK, BSP_IO_LEVEL_HIGH);
    hx711_delay_1us();
    (void) g_ioport.p_api->pinWrite(g_ioport.p_ctrl, HX711_SCK, BSP_IO_LEVEL_LOW);

    taskEXIT_CRITICAL();

    if (0U != (value & HX711_SIGN_BIT))
    {
        value |= HX711_SIGN_EXTENSION;
    }

    *p_raw = (int32_t) value;
    return true;
}

static bool hx711_average(uint32_t sample_count, uint32_t timeout_ms, int32_t * p_average)
{
    int64_t sum = 0;
    uint32_t samples = 0U;
    TickType_t const deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    if ((0U == sample_count) || (NULL == p_average))
    {
        return false;
    }

    while ((samples < sample_count) &&
           ((int32_t) (deadline - xTaskGetTickCount()) > 0))
    {
        int32_t raw = 0;

        if (hx711_read(&raw))
        {
            sum += raw;
            samples++;
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(HX711_READY_POLL_MS));
        }
    }

    if (samples != sample_count)
    {
        return false;
    }

    *p_average = (int32_t) (sum / (int64_t) sample_count);
    return true;
}

hx711_prepare_status_t hx711_prepare(int32_t * p_offset)
{
    int32_t window[HX711_TARE_WINDOW_SAMPLES] = {0};
    uint32_t sample_count = 0U;
    uint32_t next_sample = 0U;
    bool window_was_filled = false;

    if (NULL == p_offset)
    {
        return HX711_PREPARE_NOT_READY;
    }

    hx711_init();
    vTaskDelay(pdMS_TO_TICKS(HX711_WARMUP_MS));

    TickType_t const deadline = xTaskGetTickCount() + pdMS_TO_TICKS(HX711_TARE_TIMEOUT_MS);

    while ((int32_t) (deadline - xTaskGetTickCount()) > 0)
    {
        int32_t raw = 0;

        if (!hx711_read(&raw))
        {
            vTaskDelay(pdMS_TO_TICKS(HX711_READY_POLL_MS));
            continue;
        }

        window[next_sample] = raw;
        next_sample = (next_sample + 1U) % HX711_TARE_WINDOW_SAMPLES;

        if (sample_count < HX711_TARE_WINDOW_SAMPLES)
        {
            sample_count++;
        }

        if (sample_count == HX711_TARE_WINDOW_SAMPLES)
        {
            int32_t minimum = window[0];
            int32_t maximum = window[0];
            int64_t sum = 0;

            window_was_filled = true;

            for (uint32_t i = 0U; i < HX711_TARE_WINDOW_SAMPLES; i++)
            {
                int32_t const sample = window[i];

                sum += sample;
                minimum = (sample < minimum) ? sample : minimum;
                maximum = (sample > maximum) ? sample : maximum;
            }

            if ((maximum - minimum) <= HX711_TARE_MAX_SPAN)
            {
                *p_offset = (int32_t) (sum / (int64_t) HX711_TARE_WINDOW_SAMPLES);
                return HX711_PREPARE_OK;
            }
        }
    }

    return window_was_filled ? HX711_PREPARE_ZERO_UNSTABLE : HX711_PREPARE_NOT_READY;
}

bool hx711_read_filtered(int32_t * p_raw)
{
    return hx711_average(HX711_FILTER_SAMPLES, HX711_FILTER_TIMEOUT_MS, p_raw);
}

int32_t hx711_net_to_weight_0p1g(int32_t net)
{
    int64_t scaled;

    if ((net >= -HX711_ZERO_DEADBAND) && (net <= HX711_ZERO_DEADBAND))
    {
        return 0;
    }

    /* Return weight in 0.1 g units, rounded to the nearest 0.1 g. */
    scaled = (int64_t) net * 100;
    scaled += (scaled >= 0) ? (HX711_COUNTS_PER_10G / 2) : -(HX711_COUNTS_PER_10G / 2);

    return (int32_t) (scaled / HX711_COUNTS_PER_10G);
}
