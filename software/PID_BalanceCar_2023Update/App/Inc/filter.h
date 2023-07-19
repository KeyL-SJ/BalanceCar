#ifndef __FILTER_H
#define __FILTER_H

typedef struct{
    float angle_x;
    float Q_angle_x; // 过程噪声的协方差
    float Q_gyro_x;  // 0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
    float R_angle_x;   // 测量噪声的协方差 既测量偏差
    char C_0_x;
    float Q_bias_x, Angle_err_x;
    float PCt_0_x, PCt_1_x, E_x;
    float K_0_x, K_1_x, t_0_x, t_1_x;
    float Pdot_x[4];
    float PP_x[2][2];
    float dt_x;
    
    float angle_y;
    float Q_angle_y; // 过程噪声的协方差
    float Q_gyro_y;  // 0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
    float R_angle_y;   // 测量噪声的协方差 既测量偏差
    char C_0_y;
    float Q_bias_y, Angle_err_y;
    float PCt_0_y, PCt_1_y, E_y;
    float K_0_y, K_1_y, t_0_y, t_1_y;
    float Pdot_y[4];
    float PP_y[2][2];
    float dt_y;
}KalmaTypeDef;

extern KalmaTypeDef Kalman;

void kalman_filter_init(void);
float Kalman_Filter_x(float Accel,float Gyro);
float Kalman_Filter_y(float Accel,float Gyro);
#endif

