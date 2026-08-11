#include "handeye_transform.h"

#include <stddef.h>

#define HANDEYE_CAMERA_WIDTH_PX       (640)
#define HANDEYE_CAMERA_HEIGHT_PX      (480)
#define HANDEYE_FIXED_HEIGHT_MM       (280.0)
#define HANDEYE_DENOMINATOR_EPSILON   (1.0e-9)

/*
 * Homography returned by:
 *     cv2.findHomography(pts_camera, pts_arm)
 *
 * pts_arm uses millimetres, matching the CPU1 inverse-kinematics module.
 */
static double const g_camera_to_arm_homography[3][3] =
{
    {0.038658,  0.386610, 54.505698},
    {0.548603, -0.045075, 62.271670},
    {0.000383, -0.000106,  1.000000},
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
