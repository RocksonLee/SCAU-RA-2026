#ifndef CAMERA_OV5640_H
#define CAMERA_OV5640_H

#include <stdbool.h>
#include <stdint.h>
#include "hal_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_OV5640_WIDTH          (640U)
#define CAMERA_OV5640_HEIGHT         (480U)
#define CAMERA_OV5640_BYTES_PER_PIXEL (2U)
#define CAMERA_OV5640_FRAME_BYTES    (CAMERA_OV5640_WIDTH * CAMERA_OV5640_HEIGHT * CAMERA_OV5640_BYTES_PER_PIXEL)

typedef enum e_camera_ov5640_result
{
    CAMERA_OV5640_OK = 0,
    CAMERA_OV5640_ERR_I2C,
    CAMERA_OV5640_ERR_CHIP_ID,
    CAMERA_OV5640_ERR_CAPTURE,
} camera_ov5640_result_t;

typedef struct st_camera_ov5640_ceu_debug
{
    uint32_t capture_start_error;
    uint32_t caps;
    uint32_t status;
    uint32_t events;
    uint32_t data_size;
    uint32_t interface_control;
    uint32_t interface_cycle;
    uint32_t written_bytes;
    uint32_t written_rows;
} camera_ov5640_ceu_debug_t;

camera_ov5640_result_t camera_ov5640_init(void);
camera_ov5640_result_t camera_ov5640_init_warm(void);
camera_ov5640_result_t camera_ov5640_pause(void);
camera_ov5640_result_t camera_ov5640_resume(void);
camera_ov5640_result_t camera_ov5640_stop(void);
camera_ov5640_result_t camera_ov5640_capture_frame(uint8_t * p_frame);
uint16_t camera_ov5640_chip_id(void);
bool camera_ov5640_read_reg(uint16_t reg, uint8_t * p_val);
bool camera_ov5640_set_strobe_led(bool enabled);
uint32_t camera_ov5640_last_ceu_events(void);
uint32_t camera_ov5640_last_error_step(void);
uint16_t camera_ov5640_last_failed_reg(void);
void camera_ov5640_get_ceu_debug(camera_ov5640_ceu_debug_t * p_debug);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_OV5640_H */
