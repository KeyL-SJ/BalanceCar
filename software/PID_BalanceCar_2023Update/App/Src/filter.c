#include "filter.h"
/**************************************************************************
Function: Simple Kalman filter
Input   : acceleration、angular velocity
Output  : none
函数功能：获取x轴角度简易卡尔曼滤波
入口参数：加速度获取的角度、角速度
返回  值：x轴角速度
**************************************************************************/
float dt = 0.005; // 每5ms进行一次滤波
float Kalman_Filter(float Accel, float Gyro)
{
    static float angle_dot;
    static float angle;
    float Q_angle = 0.001; // 过程噪声的协方差
    float Q_gyro = 0.003;  // 0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
    float R_angle = 0.5;   // 测量噪声的协方差 既测量偏差
    char C_0 = 1;
    static float Q_bias, Angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] = {0, 0, 0, 0};
    static float PP[2][2] = {{1, 0}, {0, 1}};
    angle += (Gyro - Q_bias) * dt;           // 先验估计
    Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1] = -PP[1][1];
    Pdot[2] = -PP[1][1];
    Pdot[3] = Q_gyro;
    PP[0][0] += Pdot[0] * dt; // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt; // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - angle; // zk-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0; // 后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    angle += K_0 * Angle_err;  // 后验估计
    Q_bias += K_1 * Angle_err; // 后验估计
    angle_dot = Gyro - Q_bias; // 输出值(后验估计)的微分=角速度
    return angle;
}
