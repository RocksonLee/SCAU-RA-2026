#include "lcd_test.h"

#include "hal_data.h"
#include "lcd_spi.h"

void lcd_rgb_cycle_test(void)
{
    while (1)
    {
        lcd_fill_screen(0xF800U);
        R_BSP_SoftwareDelay(1000U, BSP_DELAY_UNITS_MILLISECONDS);

        lcd_fill_screen(0x07E0U);
        R_BSP_SoftwareDelay(1000U, BSP_DELAY_UNITS_MILLISECONDS);

        lcd_fill_screen(0x001FU);
        R_BSP_SoftwareDelay(1000U, BSP_DELAY_UNITS_MILLISECONDS);
    }
}
