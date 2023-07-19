#ifndef __PID_H
#define __PID_H
#include "sys.h"

#define Middle_angle 0

typedef struct{
    float Balance_Kp;
    float Balance_Kd;
    
    float Velocity_Kp;
    float Velocity_Ki;
    
    float Turn_Kp;
    float Turn_Kd;
}PID_TypeDef;//PID参数（放大100倍）
extern PID_TypeDef PID;

void PID_Init(void);
int Balance(float Angle, float Gyro);
int Velocity(int encoder_left, int encoder_right);
int Turn(float gyro);
#endif
