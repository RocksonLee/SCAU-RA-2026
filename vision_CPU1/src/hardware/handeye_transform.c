#include "handeye_transform.h"

#include <stddef.h>

#define HANDEYE_CAMERA_WIDTH_PX       (640)
#define HANDEYE_CAMERA_HEIGHT_PX      (480)
#define HANDEYE_FIXED_HEIGHT_MM       (100.0)
#define HANDEYE_TOP_Z_LOW_MM          (275.0)
#define HANDEYE_TOP_Z_HIGH_MM         (425.0)
#define HANDEYE_DENOMINATOR_EPSILON   (1.0e-9)
#define HANDEYE_SIDE_X_MIN_PX         (104)
#define HANDEYE_SIDE_X_MAX_PX         (435)
#define HANDEYE_SIDE_Z_SLOPE_MM_PX    (0.39634450194777904)
#define HANDEYE_SIDE_Z_OFFSET_MM      (270.4756856225454)

/* Z=0 top-camera homography used by the top-only task. */
static double const g_camera_to_arm_homography_z0[3][3] =
{
    {0.031114,  0.800983,  -64.142100},
    {0.954234, -0.013392, -101.298640},
    {0.000247, -0.000043,    1.000000},
};

/* Height-aware top-camera homographies calibrated at 27.5 cm and 42.5 cm. */
static double const g_camera_to_arm_homography_z275[3][3] =
{
    {0.166329,  0.311028, 10.802838},
    {0.796301, -0.114012, 26.787988},
    {0.001480, -0.000806,  1.000000},
};

static double const g_camera_to_arm_homography_z425[3][3] =
{
    {0.127322, -0.270970,  96.431837},
    {0.309648, -0.456334, 148.252655},
    {0.001204, -0.002781,   1.000000},
};

static bool handeye_project_top(double const homography[3][3],
                                double       u,
                                double       v,
                                double     * p_x_mm,
                                double     * p_y_mm)
{
    double const denominator = (homography[2][0] * u) +
                               (homography[2][1] * v) +
                                homography[2][2];

    if ((denominator > -HANDEYE_DENOMINATOR_EPSILON) &&
        (denominator < HANDEYE_DENOMINATOR_EPSILON))
    {
        return false;
    }

    *p_x_mm = ((homography[0][0] * u) +
               (homography[0][1] * v) +
                homography[0][2]) / denominator;
    *p_y_mm = ((homography[1][0] * u) +
               (homography[1][1] * v) +
                homography[1][2]) / denominator;
    return true;
}

static bool handeye_top_pixel_to_arm_at_z(int32_t               camera_x_px,
                                           int32_t               camera_y_px,
                                           double                z_mm,
                                           handeye_arm_point_t * p_arm_point)
{
    if ((NULL == p_arm_point) ||
        (camera_x_px < 0) || (camera_x_px >= HANDEYE_CAMERA_WIDTH_PX) ||
        (camera_y_px < 0) || (camera_y_px >= HANDEYE_CAMERA_HEIGHT_PX) ||
        (z_mm < HANDEYE_TOP_Z_LOW_MM) || (z_mm > HANDEYE_TOP_Z_HIGH_MM))
    {
        return false;
    }

    double const u = (double) camera_x_px;
    double const v = (double) camera_y_px;
    double x_low_mm;
    double y_low_mm;
    double x_high_mm;
    double y_high_mm;

    if (!handeye_project_top(g_camera_to_arm_homography_z275, u, v, &x_low_mm, &y_low_mm) ||
        !handeye_project_top(g_camera_to_arm_homography_z425, u, v, &x_high_mm, &y_high_mm))
    {
        return false;
    }

    double const ratio = (z_mm - HANDEYE_TOP_Z_LOW_MM) /
                         (HANDEYE_TOP_Z_HIGH_MM - HANDEYE_TOP_Z_LOW_MM);
    p_arm_point->x_mm = x_low_mm + (ratio * (x_high_mm - x_low_mm));
    p_arm_point->y_mm = y_low_mm + (ratio * (y_high_mm - y_low_mm));
    p_arm_point->z_mm = z_mm;

    return true;
}

bool handeye_pixel_to_arm(int32_t camera_x_px,
                          int32_t camera_y_px,
                          handeye_arm_point_t * p_arm_point)
{
    if ((NULL == p_arm_point) ||
        (camera_x_px < 0) || (camera_x_px >= HANDEYE_CAMERA_WIDTH_PX) ||
        (camera_y_px < 0) || (camera_y_px >= HANDEYE_CAMERA_HEIGHT_PX) ||
        !handeye_project_top(g_camera_to_arm_homography_z0,
                             (double) camera_x_px,
                             (double) camera_y_px,
                             &p_arm_point->x_mm,
                             &p_arm_point->y_mm))
    {
        return false;
    }

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
        !handeye_side_pixel_to_arm_z(side_x_px, &z_mm) ||
        !handeye_top_pixel_to_arm_at_z(top_x_px, top_y_px, z_mm, p_arm_point))
    {
        return false;
    }

    return true;
}
