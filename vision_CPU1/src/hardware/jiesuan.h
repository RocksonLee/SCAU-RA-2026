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