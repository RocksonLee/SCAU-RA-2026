#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>
#include "jiesuan.h"
#define M_PI 3.14159
// ==========================================================
// 5轴机械臂正运动学（4轴锁死 | 5轴自动补偿保持夹爪水平）
// ==========================================================
void forward_kinematics_5dof(double q1, double q2, double q3, Pose* out) {
    const double a1 = 37.50;
    const double a2 = 160.00;
    const double a3 = 15.00;
    const double d1 = 170.00;
    const double d4 = 142.30;
    const double L_gripper = 80.00; // 夹爪水平延伸量

    // 【硬件映射层：电机角度 -> 数学模型角度】
    // 说明：1轴和3轴的电机物理方向与数学正方向相反，此处转回数学模型
    double math_q1 = -q1; 
    double math_q2 =  q2;
    double math_q3 = -q3;

    double q23 = math_q2 + math_q3;
    
    // 三角函数计算 (全部基于内部数学角度)
    double c1 = cos(math_q1), s1 = sin(math_q1);
    double c2 = cos(math_q2), s2 = sin(math_q2);
    double c23 = cos(q23), s23 = sin(q23);

    // 位置计算：由于 q5 补偿让夹爪永远水平，50mm 纯粹加在水平投影半径上
    double r_arm = a1 + a2 * s2 + a3 * s23 + d4 * c23 + L_gripper;
    
    out->x = c1 * r_arm;
    out->y = s1 * r_arm;
    out->z = d1 + a2 * c2 + a3 * c23 - d4 * s23; 

    // 姿态矩阵：永远与地面水平
    out->R[0][0] = c1;  out->R[0][1] = s1;  out->R[0][2] = 0.0;
    out->R[1][0] = -s1; out->R[1][1] = c1;  out->R[1][2] = 0.0;
    out->R[2][0] = 0.0; out->R[2][1] = 0.0; out->R[2][2] = 1.0;
}

// ==========================================================
// 5轴机械臂逆运动学（纯几何解法，零矩阵运算）
// ==========================================================
/**
 * @param x, y, z      目标坐标 (夹爪中心)
 * @param elbow_dir    手肘姿态选择：1 表示解法一(通常是手肘向外)，-1 表示解法二(手肘向内)
 * @param q1,q2,q3,q5  输出的关节角度(弧度)
 * @return int         1 表示解算成功，0 表示目标点超出机械臂可达范围
 */
int inverse_kinematics_5dof(double x, double y, double z, int elbow_dir, 
                            double* q1, double* q2, double* q3, double* q5) {
    // 连杆参数
    const double a1 = 37.50;
    const double a2 = 160.00;
    const double a3 = 15.00;
    const double d1 = 170.00;
    const double d4 = 142.30;
    const double L_gripper = 80.00; 

    // 内部数学模型角度变量
    double math_q1, math_q2, math_q3, math_q5;

    // 1. 计算 q1 (数学角度)
    math_q1 = atan2(y, x);
    double r_t = sqrt(x * x + y * y); 

    // 2. 剥离基座参数和夹爪延伸量，计算手腕相对于第二轴的 R-Z 平面偏移
    double dr = r_t - L_gripper - a1; 
    double dz = z - d1;
    
    // 3. 计算手腕到第二轴的直线距离的平方 (D^2)
    double D2 = dr * dr + dz * dz;
    double D = sqrt(D2);

    // 合成虚拟连杆 L3 的参数 
    double L2 = a2;
    double L3 = sqrt(a3 * a3 + d4 * d4); 
    double phi = atan2(d4, a3); 

    // 4. 判断是否超出工作空间
    if (D > L2 + L3 || D < fabs(L2 - L3)) {
        return 0; // 超出范围
    }

    // 5. 使用余弦定理求 q3 的相关角 (beta)
    double cos_beta = (D2 - L2 * L2 - L3 * L3) / (2 * L2 * L3);
    if(cos_beta > 1.0) cos_beta = 1.0;
    if(cos_beta < -1.0) cos_beta = -1.0;
    
    double beta = elbow_dir * acos(cos_beta); 
    math_q3 = beta - phi; // 数学 q3

    // 6. 使用余弦定理求 q2 的相关角 (alpha 和 gamma)
    double alpha = atan2(dr, dz); 
    double cos_gamma = (L2 * L2 + D2 - L3 * L3) / (2 * L2 * D);
    if(cos_gamma > 1.0) cos_gamma = 1.0;
    if(cos_gamma < -1.0) cos_gamma = -1.0;
    
    double gamma = acos(cos_gamma);
    math_q2 = alpha - elbow_dir * gamma; // 数学 q2

    // 7. 第5轴自动补偿，让夹爪水平 (基于数学模型计算)
    math_q5 = -(math_q2 + math_q3);

    // ==================================================
    // 【硬件映射层：数学模型角度 -> 电机实际角度】
    // 说明：按需求对 1轴、2轴进行取反输出，3轴、5轴保持数学模型方向
    // ==================================================
    *q1 = round(-math_q1 * 180.0 / M_PI);
    *q2 = round(-math_q2 * 180.0 / M_PI);
    *q3 = round( math_q3 * 180.0 / M_PI);
    *q5 = round( math_q5 * 180.0 / M_PI);

    return 1; // 解算成功
}
