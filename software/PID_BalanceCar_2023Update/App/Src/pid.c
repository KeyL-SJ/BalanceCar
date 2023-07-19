#include "pid.h"

PID_TypeDef PID;
uint8_t Auto_Turn_Flag = 0;
void PID_Init(void)
{
    PID.Balance_Kp = 22500;
    PID.Balance_Kd = 108;

    PID.Velocity_Kp = 16000;
    PID.Velocity_Ki = 80;

    PID.Turn_Kp = 4200;
    PID.Turn_Kd = 0;
}

/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle；Gyro：angular velocity
Output  : balance：Vertical control PWM
函数功能：直立PD控制
入口参数：Angle:角度；Gyro：角速度
返回  值：balance：直立控制PWM
**************************************************************************/
int Balance(float Angle, float Gyro)
{
    float Angle_bias, Gyro_bias;
    int balance;
    Angle_bias = Middle_angle - Angle; // 求出平衡的角度中值 和机械相关
    Gyro_bias = 0 - Gyro;
    balance = -PID.Balance_Kp / 100 * Angle_bias - Gyro_bias * PID.Balance_Kd / 100; // 计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
    return balance;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left：Left wheel encoder reading；encoder_right：Right wheel encoder reading
Output  : Speed control PWM
函数功能：速度控制PWM
入口参数：encoder_left：左轮编码器读数；encoder_right：右轮编码器读数
返回  值：速度控制PWM
**************************************************************************/
// 修改前进后退速度，请修改Target_Velocity，比如，改成60就比较慢了
int Velocity(int encoder_left, int encoder_right)
{
    static float velocity, Encoder_Least, Encoder_bias, Movement;
    static float Encoder_Integral, Target_Velocity;
    //================遥控前进后退部分====================//
    if (BalanceCar.Flag_follow == 1 || BalanceCar.Flag_avoid == 1)
        Target_Velocity = 30; // 如果进入跟随/避障模式,降低速度
    else
        Target_Velocity = 50;
    if (BalanceCar.Flag_front == 1)
        Movement = Target_Velocity / BalanceCar.Flag_velocity; // 收到前进信号
    else if (BalanceCar.Flag_back == 1)
        Movement = -Target_Velocity / BalanceCar.Flag_velocity; // 收到后退信号
    else
        Movement = 0;

    //=============超声波功能（跟随/避障）==================//
    if (BalanceCar.Flag_follow == 1 && (BalanceCar.Distance > 200 && BalanceCar.Distance < 500) && BalanceCar.Flag_Left != 1 && BalanceCar.Flag_Right != 1) // 跟随
        Movement = Target_Velocity / BalanceCar.Flag_velocity;
    if (BalanceCar.Flag_follow == 1 && BalanceCar.Distance < 200 && BalanceCar.Flag_Left != 1 && BalanceCar.Flag_Right != 1)
        Movement = -Target_Velocity / BalanceCar.Flag_velocity;

    if (BalanceCar.Flag_avoid == 1) // 超声波避障
    {
        Movement = Target_Velocity / BalanceCar.Flag_velocity;
        Auto_Turn_Flag = 0;
        if (BalanceCar.Distance < 450 && BalanceCar.Flag_Left != 1 && BalanceCar.Flag_Right != 1)
        {
            Auto_Turn_Flag = 1;
            Movement = -Target_Velocity / BalanceCar.Flag_velocity;
        }
    }

    //================速度PI控制器=====================//
    Encoder_Least = 0 - (encoder_left + encoder_right); // 获取最新速度偏差=目标速度（此处为零）-测量速度（左右编码器之和）
    Encoder_bias *= 0.86;                               // 一阶低通滤波器
    Encoder_bias += Encoder_Least * 0.14;               // 一阶低通滤波器，减缓速度变化
    Encoder_Integral += Encoder_bias;                   // 积分出位移 积分时间：10ms
    Encoder_Integral = Encoder_Integral + Movement;     // 接收遥控器数据，控制前进后退
    if (Encoder_Integral > 10000)
        Encoder_Integral = 10000; // 积分限幅
    if (Encoder_Integral < -10000)
        Encoder_Integral = -10000;                                                               // 积分限幅
    velocity = -Encoder_bias * PID.Velocity_Kp / 100 - Encoder_Integral * PID.Velocity_Ki / 100; // 速度控制
    if (Turn_Off(BalanceCar.Angle_Balance, BalanceCar.Voltage) == 1 || BalanceCar.Flag_Stop == 1)
        Encoder_Integral = 0; // 电机关闭后清除积分
    return velocity;
}

/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
函数功能：转向控制
入口参数：Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
int Turn(float gyro)
{
    static float Turn_Target, turn, Turn_Amplitude = 54;
    float Kp = PID.Turn_Kp, Kd; // 修改转向速度，请修改Turn_Amplitude即可

    if (Auto_Turn_Flag == 1) /*!< 超声波避障模式下默认右转 */
    {
        Turn_Target = Turn_Amplitude / BalanceCar.Flag_velocity;
    }
    else
    {
        //===================遥控左右旋转部分=================//
        if (BalanceCar.Flag_Left == 1)
            Turn_Target = -Turn_Amplitude / BalanceCar.Flag_velocity;
        else if (BalanceCar.Flag_Right == 1)
            Turn_Target = Turn_Amplitude / BalanceCar.Flag_velocity;
        else
            Turn_Target = 0;
        if (BalanceCar.Flag_front == 1 || BalanceCar.Flag_back == 1)
            Kd = PID.Turn_Kd;
        else
            Kd = 0; // 转向的时候取消陀螺仪的纠正 有点模糊PID的思想
    }
    //===================转向PD控制器=================//
    turn = Turn_Target * Kp / 100 + gyro * Kd / 100; // 结合Z轴陀螺仪进行PD控制
    return turn;                                     // 转向环PWM右转为正，左转为负
}
