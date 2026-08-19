#ifndef __JIESUAN_H__
#define __JIESUAN_H__
typedef struct {
    double x;
    double y;
    double z;
    double R[3][3]; // 最终水平姿态旋转矩阵
} Pose;
void forward_kinematics_5dof(double q1, double q2, double q3, Pose* out);
int inverse_kinematics_5dof(double x, double y, double z, int elbow_dir, double* q1, double* q2, double* q3, double* q5);
#endif
// 高度395mm
// [86.6, 238.1], [540, 167]
// [87.3, 227.4], [520, 163]
// [52.8, 197.2], [348, 27]
// [55.5, 193.5], [345, 35]
// [125, 216.5], [400, 343]
// [121.6, 202.3], [402, 343]

// 高度350mm
// [81, 235.4], [517, 166]
// [89.8, 233.8], [493, 171]
// [82.9, 227.7], [490, 155]
// [137.3, 211.5], [382, 383]
// [132.2, 211.5], [372, 350]
// [119.1, 206.3], [402, 307]