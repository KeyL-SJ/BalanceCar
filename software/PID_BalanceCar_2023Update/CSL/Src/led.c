#include "led.h"
/**************************************************************************
Function: Led flashing
Input   : time：Flicker frequency
Output  : none
函数功能：LED闪烁
入口参数：闪烁频率 
返回  值：无
**************************************************************************/
void Led_Flash(u16 time)
{
	 static int temp;
	 if(0==time) LED=0;
	 else 	if(++temp==time)	LED=~LED,temp=0;
}

