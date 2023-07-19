#include "filter.h"
/**************************************************************************
Function: Simple Kalman filter
Input   : acceleration、angular velocity
Output  : none
函数功能：获取x轴角度简易卡尔曼滤波
入口参数：加速度获取的角度、角速度
返回  值：x轴角速度
**************************************************************************/
KalmaTypeDef Kalman;

void kalman_filter_init(void)
{
    Kalman.Q_angle_x = 0.001;
    Kalman.Q_gyro_x = 0.003;
    Kalman.R_angle_x = 0.5;
    Kalman.C_0_x = 1;
    Kalman.Pdot_x[0] = 0;
    Kalman.Pdot_x[1] = 0;
    Kalman.Pdot_x[2] = 0;
    Kalman.Pdot_x[3] = 0;
    Kalman.PP_x[0][0] = 1;
    Kalman.PP_x[0][1] = 0;
    Kalman.PP_x[1][0] = 0;
    Kalman.PP_x[1][1] = 1;
    Kalman.dt_x= 0.005;
    
    Kalman.Q_angle_y = 0.001;
    Kalman.Q_gyro_y = 0.003;
    Kalman.R_angle_y = 0.5;
    Kalman.C_0_y = 1;
    Kalman.Pdot_y[0] = 0;
    Kalman.Pdot_y[1] = 0;
    Kalman.Pdot_y[2] = 0;
    Kalman.Pdot_y[3] = 0;
    Kalman.PP_y[0][0] = 1;
    Kalman.PP_y[0][1] = 0;
    Kalman.PP_y[1][0] = 0;
    Kalman.PP_y[1][1] = 1;
    Kalman.dt_y = 0.005;
}

float Kalman_Filter_x(float Accel, float Gyro)
{
    Kalman.angle_x += (Gyro - Kalman.Q_bias_x) * Kalman.dt_x;           // 先验估计
    Kalman.Pdot_x[0] = Kalman.Q_angle_x - Kalman.PP_x[0][1] - Kalman.PP_x[1][0]; // Pk-先验估计误差协方差的微分

    Kalman.Pdot_x[1] = -Kalman.PP_x[1][1];
    Kalman.Pdot_x[2] = -Kalman.PP_x[1][1];
    Kalman.Pdot_x[3] = Kalman.Q_gyro_x;
    Kalman.PP_x[0][0] += Kalman.Pdot_x[0] * Kalman.dt_x; // Pk-先验估计误差协方差微分的积分
    Kalman.PP_x[0][1] += Kalman.Pdot_x[1] * Kalman.dt_x; // =先验估计误差协方差
    Kalman.PP_x[1][0] += Kalman.Pdot_x[2] * Kalman.dt_x;
    Kalman.PP_x[1][1] += Kalman.Pdot_x[3] * Kalman.dt_x;

    Kalman.Angle_err_x = Accel - Kalman.angle_x; // zk-先验估计

    Kalman.PCt_0_x = Kalman.C_0_x * Kalman.PP_x[0][0];
    Kalman.PCt_1_x = Kalman.C_0_x * Kalman.PP_x[1][0];

    Kalman.E_x = Kalman.R_angle_x + Kalman.C_0_x * Kalman.PCt_0_x;

    Kalman.K_0_x = Kalman.PCt_0_x / Kalman.E_x;
    Kalman.K_1_x = Kalman.PCt_1_x / Kalman.E_x;

    Kalman.t_0_x = Kalman.PCt_0_x;
    Kalman.t_1_x = Kalman.C_0_x * Kalman.PP_x[0][1];

    Kalman.PP_x[0][0] -= Kalman.K_0_x * Kalman.t_0_x; // 后验估计误差协方差
    Kalman.PP_x[0][1] -= Kalman.K_0_x * Kalman.t_1_x;
    Kalman.PP_x[1][0] -= Kalman.K_1_x * Kalman.t_0_x;
    Kalman.PP_x[1][1] -= Kalman.K_1_x * Kalman.t_1_x;

    Kalman.angle_x += Kalman.K_0_x * Kalman.Angle_err_x;  // 后验估计
    Kalman.Q_bias_x += Kalman.K_1_x * Kalman.Angle_err_x; // 后验估计
    return Kalman.angle_x;
}

float Kalman_Filter_y(float Accel,float Gyro)		
{
	Kalman.angle_y += (Gyro - Kalman.Q_bias_y) * Kalman.dt_y; //先验估计
	Kalman.Pdot_y[0] = Kalman.Q_angle_y - Kalman.PP_y[0][1] - Kalman.PP_y[1][0]; // Pk-先验估计误差协方差的微分
	Kalman.Pdot_y[1] = -Kalman.PP_y[1][1];
	Kalman.Pdot_y[2] = -Kalman.PP_y[1][1];
	Kalman.Pdot_y[3] = Kalman.Q_gyro_y;

	Kalman.PP_y[0][0] += Kalman.Pdot_y[0] * Kalman.dt_y;   // Pk-先验估计误差协方差微分的积分
	Kalman.PP_y[0][1] += Kalman.Pdot_y[1] * Kalman.dt_y;   // =先验估计误差协方差
	Kalman.PP_y[1][0] += Kalman.Pdot_y[2] * Kalman.dt_y;
	Kalman.PP_y[1][1] += Kalman.Pdot_y[3] * Kalman.dt_y;

	Kalman.Angle_err_y = Accel - Kalman.angle_y;	//zk-先验估计
	
	Kalman.PCt_0_y = Kalman.C_0_y * Kalman.PP_y[0][0];
	Kalman.PCt_1_y = Kalman.C_0_y * Kalman.PP_y[1][0];
	
	Kalman.E_y = Kalman.R_angle_y + Kalman.C_0_y * Kalman.PCt_0_y;
	
	Kalman.K_0_y = Kalman.PCt_0_y / Kalman.E_y;
	Kalman.K_1_y = Kalman.PCt_1_y / Kalman.E_y;
	
	Kalman.t_0_y = Kalman.PCt_0_y;
	Kalman.t_1_y = Kalman.C_0_y * Kalman.PP_y[0][1];

	Kalman.PP_y[0][0] -= Kalman.K_0_y * Kalman.t_0_y;		 //后验估计误差协方差
	Kalman.PP_y[0][1] -= Kalman.K_0_y * Kalman.t_1_y;
	Kalman.PP_y[1][0] -= Kalman.K_1_y * Kalman.t_0_y;
	Kalman.PP_y[1][1] -= Kalman.K_1_y * Kalman.t_1_y;
		
	Kalman.angle_y += Kalman.K_0_y * Kalman.Angle_err_y;	   //后验估计
	Kalman.Q_bias_y += Kalman.K_1_y * Kalman.Angle_err_y;	 //后验估计
	return Kalman.angle_y;
}
