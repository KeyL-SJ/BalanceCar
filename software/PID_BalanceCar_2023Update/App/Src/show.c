#include "show.h"
float Velocity_Left, Velocity_Right; //车轮速度(mm/s)
/**************************************************************************
Function: OLED display
Input   : none
Output  : none
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{
	//=============第一行显示小车模式=======================//
	OLED_ShowString(0, 0, "KeyLsj");

	if (Flag_follow == 1)
		OLED_ShowString(70, 0, "Follow ");
	else if (Flag_avoid == 1)
		OLED_ShowString(70, 0, "Avoid  ");
	else
		OLED_ShowString(70, 0, "Balance");
	//=============第二行显示角度=======================//
	OLED_ShowString(00, 10, "Angle");
	if (Angle_Balance < 0)
		OLED_ShowString(48, 10, "-");
	if (Angle_Balance >= 0)
		OLED_ShowString(48, 10, "+");
	OLED_ShowNumber(56, 10, myabs((int)Angle_Balance), 3, 12);
	//=============第三行显示角速度与距离===============//
	OLED_ShowString(0, 20, "Gyrox");
	if (Gyro_Balance < 0)
		OLED_ShowString(42, 20, "-");
	if (Gyro_Balance >= 0)
		OLED_ShowString(42, 20, "+");
	OLED_ShowNumber(50, 20, myabs((int)Gyro_Balance), 4, 12);

	OLED_ShowNumber(82, 20, (u16)Distance, 5, 12);
	OLED_ShowString(114, 20, "mm");

	//=============第四行显示左编码器PWM与读数=======================//
	OLED_ShowString(00, 30, "L");
	if (Motor_Left < 0)
		OLED_ShowString(16, 30, "-"),
			OLED_ShowNumber(26, 30, myabs((int)Motor_Left), 4, 12);
	if (Motor_Left >= 0)
		OLED_ShowString(16, 30, "+"),
			OLED_ShowNumber(26, 30, myabs((int)Motor_Left), 4, 12);

	if (Velocity_Left < 0)
		OLED_ShowString(60, 30, "-");
	if (Velocity_Left >= 0)
		OLED_ShowString(60, 30, "+");
	OLED_ShowNumber(68, 30, myabs((int)Velocity_Left), 4, 12);
	OLED_ShowString(96, 30, "mm/s");

	//=============第五行显示右编码器PWM与读数=======================//
	OLED_ShowString(00, 40, "R");
	if (Motor_Right < 0)
		OLED_ShowString(16, 40, "-"),
			OLED_ShowNumber(26, 40, myabs((int)Motor_Right), 4, 12);
	if (Motor_Right >= 0)
		OLED_ShowString(16, 40, "+"),
			OLED_ShowNumber(26, 40, myabs((int)Motor_Right), 4, 12);

	if (Velocity_Right < 0)
		OLED_ShowString(60, 40, "-");
	if (Velocity_Right >= 0)
		OLED_ShowString(60, 40, "+");
	OLED_ShowNumber(68, 40, myabs((int)Velocity_Right), 4, 12);
	OLED_ShowString(96, 40, "mm/s");

	//=============第六行显示电压与电机开关=======================//
	OLED_ShowString(0, 50, "V");
	OLED_ShowString(30, 50, ".");
	OLED_ShowString(54, 50, "V");
	OLED_ShowNumber(19, 50, Voltage / 100, 2, 12);
	OLED_ShowNumber(42, 50, Voltage % 100, 2, 12);
	if (Flag_Stop)
		OLED_ShowString(85, 50, "OFF");
	if (!Flag_Stop)
		OLED_ShowString(85, 50, "ON ");

	//=============刷新=======================//
	OLED_Refresh_Gram();
}
