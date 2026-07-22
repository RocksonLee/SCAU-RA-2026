#ifndef CAMERA_OV5640_H
#define CAMERA_OV5640_H

#include <stdbool.h>
#include <stdint.h>
#include "hal_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_OV5640_WIDTH          (320U)
#define CAMERA_OV5640_HEIGHT         (240U)
#define CAMERA_OV5640_BYTES_PER_PIXEL (2U)
#define CAMERA_OV5640_FRAME_BYTES    (CAMERA_OV5640_WIDTH * CAMERA_OV5640_HEIGHT * CAMERA_OV5640_BYTES_PER_PIXEL)

typedef enum e_camera_ov5640_result
{
    CAMERA_OV5640_OK = 0,
    CAMERA_OV5640_ERR_I2C,
    CAMERA_OV5640_ERR_CHIP_ID,
    CAMERA_OV5640_ERR_CAPTURE,
} camera_ov5640_result_t;

camera_ov5640_result_t camera_ov5640_init(void);
camera_ov5640_result_t camera_ov5640_capture_frame(uint8_t * p_frame);
uint16_t camera_ov5640_chip_id(void);
bool camera_ov5640_read_reg(uint16_t reg, uint8_t * p_val);
uint32_t camera_ov5640_last_ceu_events(void);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_OV5640_H */
