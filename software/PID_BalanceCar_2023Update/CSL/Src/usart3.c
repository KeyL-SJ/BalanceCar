#include "usart3.h"

u8 Usart3_Receive_buf[1];                                    // 串口3接收中断数据存放的缓冲区
u8 Usart3_Receive;                                           // 从串口3读取的数据
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) // 接收回调函数
{
    if (UartHandle->Instance == USART3)
    {
        static int uart_receive = 0; // 蓝牙接收相关变量
        static u8 Flag_PID, i, j, Receive[50];
        static float Data;
        uart_receive = Usart3_Receive_buf[0];
        Usart3_Receive = uart_receive;
        if (uart_receive == 0x59)
            Flag_velocity = 2; // 低速挡（默认值）
        if (uart_receive == 0x58)
            Flag_velocity = 1; // 高速档

        if (uart_receive > 10) // 默认使用
        {
            if (uart_receive == 0x5A)
                Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 0; // 刹车
            else if (uart_receive == 0x41)
                Flag_front = 1, Flag_back = 0, Flag_Left = 0, Flag_Right = 0; // 前
            else if (uart_receive == 0x45)
                Flag_front = 0, Flag_back = 1, Flag_Left = 0, Flag_Right = 0; // 后
            else if (uart_receive == 0x42 || uart_receive == 0x43 || uart_receive == 0x44)
                Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 1; // 右
            else if (uart_receive == 0x46 || uart_receive == 0x47 || uart_receive == 0x48)
                Flag_front = 0, Flag_back = 0, Flag_Left = 1, Flag_Right = 0; // 左
            else
                Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 0; // 刹车
        }
        if (uart_receive < 10) // 备用app为：MiniBalanceV1.0  因为MiniBalanceV1.0的遥控指令为A~H 其HEX都小于10
        {
            Flag_velocity = 1; // 切换至高速档
            if (uart_receive == 0x00)
                Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 0; // 刹车
            else if (uart_receive == 0x01)
                Flag_front = 1, Flag_back = 0, Flag_Left = 0, Flag_Right = 0; // 前
            else if (uart_receive == 0x05)
                Flag_front = 0, Flag_back = 1, Flag_Left = 0, Flag_Right = 0; // 后
            else if (uart_receive == 0x02 || uart_receive == 0x03 || uart_receive == 0x04)
                Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 1;              // 左
            else if (uart_receive == 0x06 || uart_receive == 0x07 || uart_receive == 0x08) // 右
                Flag_front = 0, Flag_back = 0, Flag_Left = 1, Flag_Right = 0;
            else
                Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 0; // 刹车
        }

        if (Usart3_Receive == 0x7B)
            Flag_PID = 1; // APP参数指令起始位
        if (Usart3_Receive == 0x7D)
            Flag_PID = 2; // APP参数指令停止位

        if (Flag_PID == 1) // 采集数据
        {
            Receive[i] = Usart3_Receive;
            i++;
        }
        if (Flag_PID == 2) // 分析数据
        {
            if (Receive[3] == 0x50)
                PID_Send = 1;
            else if (Receive[1] != 0x23)
            {
                for (j = i; j >= 4; j--)
                {
                    Data += (Receive[j - 1] - 48) * pow(10, i - j);
                }
                switch (Receive[1])
                {
                case 0x30:
                    Balance_Kp = Data;
                    break;
                case 0x31:
                    Balance_Kd = Data;
                    break;
                case 0x32:
                    Velocity_Kp = Data;
                    break;
                case 0x33:
                    Velocity_Ki = Data;
                    break;
                case 0x34:
                    Turn_Kp = Data;
                    break;
                case 0x35:
                    Turn_Kd = Data;
                    break;
                case 0x36:
                    break; // 预留
                case 0x37:
                    break; // 预留
                case 0x38:
                    break; // 预留
                }
            }
            Flag_PID = 0;
            i = 0;
            j = 0;
            Data = 0;
            memset(Receive, 0, sizeof(u8) * 50); // 数组清零
        }

        HAL_UART_Receive_IT(&huart3, Usart3_Receive_buf, sizeof(Usart3_Receive_buf)); // 串口3回调函数执行完毕之后，需要再次开启接收中断等待下一次接收中断的发生
    }
}
