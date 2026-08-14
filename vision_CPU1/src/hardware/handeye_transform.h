#ifndef HANDEYE_TRANSFORM_H
#define HANDEYE_TRANSFORM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct st_handeye_arm_point
{
    double x_mm;
    double y_mm;
    double z_mm;
} handeye_arm_point_t;

/**
 * Convert a 640x480 camera pixel coordinate to the robot coordinate system.
 *
 * The homography output and the fixed height are expressed in millimetres.
 * The fixed height is 28 cm (280 mm).
 *
 * @param[in]  camera_x_px Camera horizontal pixel coordinate [0, 639].
 * @param[in]  camera_y_px Camera vertical pixel coordinate [0, 479].
 * @param[out] p_arm_point Converted robot coordinate.
 *
 * @return true when conversion succeeds; false for invalid input or a
 *         near-zero homography denominator.
 */
bool handeye_pixel_to_arm(int32_t camera_x_px,
                          int32_t camera_y_px,
                          handeye_arm_point_t * p_arm_point);

/**
 * Convert the side-camera horizontal pixel coordinate to robot Z height.
 *
 * The conversion is valid over the calibrated side-camera range only.
 *
 * @param[in]  side_x_px Side-camera horizontal pixel coordinate.
 * @param[out] p_z_mm    Converted robot Z coordinate in millimetres.
 *
 * @return true when the pixel is inside the calibrated range; otherwise false.
 */
bool handeye_side_pixel_to_arm_z(int32_t side_x_px, double * p_z_mm);

/**
 * Fuse the top-camera XY conversion with the side-camera Z conversion.
 */
bool handeye_pixels_to_arm_3d(int32_t top_x_px,
                              int32_t top_y_px,
                              int32_t side_x_px,
                              handeye_arm_point_t * p_arm_point);

#ifdef __cplusplus
}
#endif

#endif /* HANDEYE_TRANSFORM_H */
