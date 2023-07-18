#include "MPU6050.h"
#include "IOI2C.h"
#include "usart.h"
#define PRINT_ACCEL (0x01)
#define PRINT_GYRO (0x02)
#define PRINT_QUAT (0x04)
#define ACCEL_ON (0x01)
#define GYRO_ON (0x02)
#define MOTION (0)
#define NO_MOTION (1)
#define DEFAULT_MPU_HZ (200)
#define FLASH_SIZE (512)
#define FLASH_MEM_START ((void *)0x1800)
#define q30 1073741824.0f
short gyro[3], accel[3], sensors;
float Roll, Pitch, Yaw;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static signed char gyro_orientation[9] = {-1, 0, 0,
                                          0, -1, 0,
                                          0, 0, 1};

static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7; // error
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7)
    {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        // printf("setting bias succesfully ......\r\n");
    }
}

uint8_t buffer[14];

int16_t MPU6050_FIFO[6][11];
int16_t Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;

/**************************************************************************
Function: The new ADC data is updated to FIFO array for filtering
Input   : ax，ay，az：x，y, z-axis acceleration data；gx，gy，gz：x. Y, z-axis angular acceleration data
Output  : none
函数功能：将新的ADC数据更新到 FIFO数组，进行滤波处理
入口参数：ax，ay，az：x，y，z轴加速度数据；gx，gy，gz：x，y，z轴角加速度数据
返回  值：无
**************************************************************************/
void MPU6050_newValues(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
    unsigned char i;
    int32_t sum = 0;
    for (i = 1; i < 10; i++)
    { // FIFO 操作
        MPU6050_FIFO[0][i - 1] = MPU6050_FIFO[0][i];
        MPU6050_FIFO[1][i - 1] = MPU6050_FIFO[1][i];
        MPU6050_FIFO[2][i - 1] = MPU6050_FIFO[2][i];
        MPU6050_FIFO[3][i - 1] = MPU6050_FIFO[3][i];
        MPU6050_FIFO[4][i - 1] = MPU6050_FIFO[4][i];
        MPU6050_FIFO[5][i - 1] = MPU6050_FIFO[5][i];
    }
    MPU6050_FIFO[0][9] = ax; // 将新的数据放置到 数据的最后面
    MPU6050_FIFO[1][9] = ay;
    MPU6050_FIFO[2][9] = az;
    MPU6050_FIFO[3][9] = gx;
    MPU6050_FIFO[4][9] = gy;
    MPU6050_FIFO[5][9] = gz;

    sum = 0;
    for (i = 0; i < 10; i++)
    { // 求当前数组的合，再取平均值
        sum += MPU6050_FIFO[0][i];
    }
    MPU6050_FIFO[0][10] = sum / 10;

    sum = 0;
    for (i = 0; i < 10; i++)
    {
        sum += MPU6050_FIFO[1][i];
    }
    MPU6050_FIFO[1][10] = sum / 10;

    sum = 0;
    for (i = 0; i < 10; i++)
    {
        sum += MPU6050_FIFO[2][i];
    }
    MPU6050_FIFO[2][10] = sum / 10;

    sum = 0;
    for (i = 0; i < 10; i++)
    {
        sum += MPU6050_FIFO[3][i];
    }
    MPU6050_FIFO[3][10] = sum / 10;

    sum = 0;
    for (i = 0; i < 10; i++)
    {
        sum += MPU6050_FIFO[4][i];
    }
    MPU6050_FIFO[4][10] = sum / 10;

    sum = 0;
    for (i = 0; i < 10; i++)
    {
        sum += MPU6050_FIFO[5][i];
    }
    MPU6050_FIFO[5][10] = sum / 10;
}

/**************************************************************************
Function: Setting the clock source of mpu6050
Input   : source：Clock source number
Output  : none
函数功能：设置  MPU6050 的时钟源
入口参数：source：时钟源编号
返回  值：无
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
**************************************************************************/
void MPU6050_setClockSource(uint8_t source)
{
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range)
{
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Setting the maximum range of mpu6050 accelerometer
Input   : range：Acceleration maximum range number
Output  : none
函数功能：设置 MPU6050 加速度计的最大量程
入口参数：range：加速度最大量程编号
返回  值：无
**************************************************************************/
// #define MPU6050_ACCEL_FS_2          0x00  		//===最大量程+-2G
// #define MPU6050_ACCEL_FS_4          0x01			//===最大量程+-4G
// #define MPU6050_ACCEL_FS_8          0x02			//===最大量程+-8G
// #define MPU6050_ACCEL_FS_16         0x03			//===最大量程+-16G
void MPU6050_setFullScaleAccelRange(uint8_t range)
{
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Set mpu6050 to sleep mode or not
Input   : enable：1，sleep；0，work；
Output  : none
函数功能：设置 MPU6050 是否进入睡眠模式
入口参数：enable：1，睡觉；0，工作；
返回  值：无
**************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled)
{
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************************************************************
Function: Read identity
Input   : none
Output  : 0x68
函数功能：读取  MPU6050 WHO_AM_I 标识
入口参数：无
返回  值：0x68
**************************************************************************/
uint8_t MPU6050_getDeviceID(void)
{

    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************************************************************
Function: Check whether mpu6050 is connected
Input   : none
Output  : 1：Connected；0：Not connected
函数功能：检测MPU6050 是否已经连接
入口参数：无
返回  值：1：已连接；0：未连接
**************************************************************************/
uint8_t MPU6050_testConnection(void)
{
    if (MPU6050_getDeviceID() == 0x68) // 0b01101000;
        return 1;
    else
        return 0;
}

/**************************************************************************
Function: Setting whether mpu6050 is the host of aux I2C cable
Input   : enable：1，yes；0;not
Output  : none
函数功能：设置 MPU6050 是否为AUX I2C线的主机
入口参数：enable：1，是；0：否
返回  值：无
**************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
{
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************************************************************
Function: Setting whether mpu6050 is the host of aux I2C cable
Input   : enable：1，yes；0;not
Output  : none
函数功能：设置 MPU6050 是否为AUX I2C线的主机
入口参数：enable：1，是；0：否
返回  值：无
**************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled)
{
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************************************************************
Function: initialization Mpu6050 to enter the available state
Input   : none
Output  : none
函数功能：初始化	MPU6050 以进入可用状态
入口参数：无
返回  值：无
**************************************************************************/
void MPU6050_initialize(void)
{
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO);     // 设置时钟
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000); // 陀螺仪量程设置
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // 加速度度最大量程 +-2G
    MPU6050_setSleepEnabled(0);                          // 进入工作状态
    MPU6050_setI2CMasterModeEnabled(0);                  // 不让MPU6050 控制AUXI2C
    MPU6050_setI2CBypassEnabled(0);                      // 主控制器的I2C与	MPU6050的AUXI2C	直通关闭
}

/**************************************************************************
Function: Initialization of DMP in mpu6050
Input   : none
Output  : none
函数功能：MPU6050内置DMP的初始化
入口参数：无
返回  值：无
**************************************************************************/
void DMP_Init(void)
{
    u8 temp[1] = {0};
    Flag_Show = 1;
    i2cRead(0x68, 0x75, 1, temp);
    printf("mpu_set_sensor complete ......\r\n");
    if (temp[0] != 0x68)
        NVIC_SystemReset();
    if (!mpu_init())
    {
        if (!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
            printf("mpu_set_sensor complete ......\r\n");
        if (!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
            printf("mpu_configure_fifo complete ......\r\n");
        if (!mpu_set_sample_rate(DEFAULT_MPU_HZ))
            printf("mpu_set_sample_rate complete ......\r\n");
        if (!dmp_load_motion_driver_firmware())
            printf("dmp_load_motion_driver_firmware complete ......\r\n");
        if (!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
            printf("dmp_set_orientation complete ......\r\n");
        if (!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                                DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                                DMP_FEATURE_GYRO_CAL))
            printf("dmp_enable_feature complete ......\r\n");
        if (!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
            printf("dmp_set_fifo_rate complete ......\r\n");
        run_self_test();
        if (!mpu_set_dmp_state(1))
            printf("mpu_set_dmp_state complete ......\r\n");
    }
    Flag_Show = 0;
}
/**************************************************************************
Function: Read the attitude information of DMP in mpu6050
Input   : none
Output  : none
函数功能：读取MPU6050内置DMP的姿态信息
入口参数：无
返回  值：无
**************************************************************************/
void Read_DMP(void)
{
    unsigned long sensor_timestamp;
    unsigned char more;
    long quat[4];

    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more); // 读取DMP数据
    if (sensors & INV_WXYZ_QUAT)
    {
        q0 = quat[0] / q30;
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;                                                                 // 四元数
        Roll = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                     // 计算出横滚角
        Pitch = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;    // 计算出俯仰角
        Yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; // 计算出偏航角
    }
}
/**************************************************************************
Function: Read mpu6050 built-in temperature sensor data
Input   : none
Output  : Centigrade temperature
函数功能：读取MPU6050内置温度传感器数据
入口参数：无
返回  值：摄氏温度
**************************************************************************/
int Read_Temperature(void)
{
    float Temp;
    Temp = (I2C_ReadOneByte(devAddr, MPU6050_RA_TEMP_OUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_TEMP_OUT_L);
    if (Temp > 32768)
        Temp -= 65536;                // 数据类型转换
    Temp = (36.53 + Temp / 340) * 10; // 温度放大十倍存放
    return (int)Temp;
}
//------------------End of File----------------------------
