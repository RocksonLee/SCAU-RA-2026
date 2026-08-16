#include "handeye_transform.h"

#include <stddef.h>

#define HANDEYE_CAMERA_WIDTH_PX       (640)
#define HANDEYE_CAMERA_HEIGHT_PX      (480)
#define HANDEYE_FIXED_HEIGHT_MM       (100.0)
#define HANDEYE_DENOMINATOR_EPSILON   (1.0e-9)
#define HANDEYE_SIDE_X_MIN_PX         (104)
#define HANDEYE_SIDE_X_MAX_PX         (435)
#define HANDEYE_SIDE_Z_SLOPE_MM_PX    (0.39634450194777904)
#define HANDEYE_SIDE_Z_OFFSET_MM      (270.4756856225454)

/*
 * Homography returned by:
 *     cv2.findHomography(pts_camera, pts_arm)
 *
 * pts_arm uses millimetres, matching the CPU1 inverse-kinematics module.
 */
static double const g_camera_to_arm_homography[3][3] =
{
    {0.031114,  0.800983,  -64.142100},
    {0.954234, -0.013392, -101.298640},
    {0.000247, -0.000043,    1.000000},
};

bool handeye_pixel_to_arm(int32_t camera_x_px,
                          int32_t camera_y_px,
                          handeye_arm_point_t * p_arm_point)
{
    if ((NULL == p_arm_point) ||
        (camera_x_px < 0) || (camera_x_px >= HANDEYE_CAMERA_WIDTH_PX) ||
        (camera_y_px < 0) || (camera_y_px >= HANDEYE_CAMERA_HEIGHT_PX))
    {
        return false;
    }

    double const u = (double) camera_x_px;
    double const v = (double) camera_y_px;
    double const denominator = (g_camera_to_arm_homography[2][0] * u) +
                               (g_camera_to_arm_homography[2][1] * v) +
                                g_camera_to_arm_homography[2][2];

    if ((denominator > -HANDEYE_DENOMINATOR_EPSILON) &&
        (denominator < HANDEYE_DENOMINATOR_EPSILON))
    {
        return false;
    }

    p_arm_point->x_mm = ((g_camera_to_arm_homography[0][0] * u) +
                         (g_camera_to_arm_homography[0][1] * v) +
                          g_camera_to_arm_homography[0][2]) / denominator;
    p_arm_point->y_mm = ((g_camera_to_arm_homography[1][0] * u) +
                         (g_camera_to_arm_homography[1][1] * v) +
                          g_camera_to_arm_homography[1][2]) / denominator;
    p_arm_point->z_mm = HANDEYE_FIXED_HEIGHT_MM;

    return true;
}

bool handeye_side_pixel_to_arm_z(int32_t side_x_px, double * p_z_mm)
{
    if ((NULL == p_z_mm) ||
        (side_x_px < HANDEYE_SIDE_X_MIN_PX) ||
        (side_x_px > HANDEYE_SIDE_X_MAX_PX))
    {
        return false;
    }

    *p_z_mm = (HANDEYE_SIDE_Z_SLOPE_MM_PX * (double) side_x_px) +
               HANDEYE_SIDE_Z_OFFSET_MM;

    return true;
}

bool handeye_pixels_to_arm_3d(int32_t top_x_px,
                              int32_t top_y_px,
                              int32_t side_x_px,
                              handeye_arm_point_t * p_arm_point)
{
    double z_mm;

    if ((NULL == p_arm_point) ||
        !handeye_pixel_to_arm(top_x_px, top_y_px, p_arm_point) ||
        !handeye_side_pixel_to_arm_z(side_x_px, &z_mm))
    {
        return false;
    }

    p_arm_point->z_mm = z_mm;

    return true;
}
