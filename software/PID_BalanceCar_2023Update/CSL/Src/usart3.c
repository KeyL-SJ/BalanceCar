#include "usart3.h"

u8 Usart3_Receive_buf[1];                                    // 串口3接收中断数据存放的缓冲区
u8 Usart3_Receive;                                           // 从串口3读取的数据
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) // 接收回调函数
{
    if (UartHandle->Instance == USART3)
    {
        static int uart_receive = 0; // 蓝牙接收相关变量

        uart_receive = Usart3_Receive_buf[0];
        Usart3_Receive = uart_receive;
        if (uart_receive == 0x59)
            BalanceCar.Flag_velocity = 2; // 低速挡（默认值）
        if (uart_receive == 0x58)
            BalanceCar.Flag_velocity = 1; // 高速档

        if (uart_receive > 10) // 默认使用
        {
            if (uart_receive == 0x5A)
                BalanceCar.Flag_front = 0, BalanceCar.Flag_back = 0, BalanceCar.Flag_Left = 0, BalanceCar.Flag_Right = 0; // 刹车
            else if (uart_receive == 0x41)
                BalanceCar.Flag_front = 1, BalanceCar.Flag_back = 0, BalanceCar.Flag_Left = 0, BalanceCar.Flag_Right = 0; // 前
            else if (uart_receive == 0x45)
                BalanceCar.Flag_front = 0, BalanceCar.Flag_back = 1, BalanceCar.Flag_Left = 0, BalanceCar.Flag_Right = 0; // 后
            else if (uart_receive == 0x42 || uart_receive == 0x43 || uart_receive == 0x44)
                BalanceCar.Flag_front = 0, BalanceCar.Flag_back = 0, BalanceCar.Flag_Left = 0, BalanceCar.Flag_Right = 1; // 右
            else if (uart_receive == 0x46 || uart_receive == 0x47 || uart_receive == 0x48)
                BalanceCar.Flag_front = 0, BalanceCar.Flag_back = 0, BalanceCar.Flag_Left = 1, BalanceCar.Flag_Right = 0; // 左
            else
                BalanceCar.Flag_front = 0, BalanceCar.Flag_back = 0, BalanceCar.Flag_Left = 0, BalanceCar.Flag_Right = 0; // 刹车
        }
        if (uart_receive < 10) // 备用app为：MiniBalanceV1.0  因为MiniBalanceV1.0的遥控指令为A~H 其HEX都小于10
        {
            BalanceCar.Flag_velocity = 1; // 切换至高速档
            if (uart_receive == 0x00)
                BalanceCar.Flag_front = 0, BalanceCar.Flag_back = 0, BalanceCar.Flag_Left = 0, BalanceCar.Flag_Right = 0; // 刹车
            else if (uart_receive == 0x01)
                BalanceCar.Flag_front = 1, BalanceCar.Flag_back = 0, BalanceCar.Flag_Left = 0, BalanceCar.Flag_Right = 0; // 前
            else if (uart_receive == 0x05)
                BalanceCar.Flag_front = 0, BalanceCar.Flag_back = 1, BalanceCar.Flag_Left = 0, BalanceCar.Flag_Right = 0; // 后
            else if (uart_receive == 0x02 || uart_receive == 0x03 || uart_receive == 0x04)
                BalanceCar.Flag_front = 0, BalanceCar.Flag_back = 0, BalanceCar.Flag_Left = 0, BalanceCar.Flag_Right = 1;              // 左
            else if (uart_receive == 0x06 || uart_receive == 0x07 || uart_receive == 0x08) // 右
                BalanceCar.Flag_front = 0, BalanceCar.Flag_back = 0, BalanceCar.Flag_Left = 1, BalanceCar.Flag_Right = 0;
            else
                BalanceCar.Flag_front = 0, BalanceCar.Flag_back = 0, BalanceCar.Flag_Left = 0, BalanceCar.Flag_Right = 0; // 刹车
        }
        HAL_UART_Receive_IT(&huart3, Usart3_Receive_buf, sizeof(Usart3_Receive_buf)); // 串口3回调函数执行完毕之后，需要再次开启接收中断等待下一次接收中断的发生
    }
}
