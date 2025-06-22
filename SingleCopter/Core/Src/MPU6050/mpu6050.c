#include "MPU6050/inv_mpu.h"
#include "MPU6050/inv_mpu_dmp_motion_driver.h"
#include "MPU6050/I2C.h"
#include "MPU6050/mpu6050.h"
#include "string.h" //for reset buffer
#include <stdlib.h>

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define deg2rad(degrees) degrees * M_PI / 180.0
#define rad2deg(radians) radians * 180.0 / M_PI

short gyro[3], accel[3], sensors;
float phi, theta, psi;
float pitch, roll, yaw;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float dcm[3][3];
float gravity[3];
static signed char gyro_orientation[9] = { -1, 0, 0, 0, -1, 0, 0, 0, 1 };

static unsigned short inv_row_2_scale(const signed char *row) {
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
    b = 7;      // error
  return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx) {
  unsigned short scalar;
  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;

  return scalar;
}

static void run_self_test(void) {
  int result;
  long gyro[3], accel[3];

  result = mpu_run_self_test(gyro, accel);
  if (result == 0x7) {
    /* Test passed. We can trust the gyro data here, so let's push it down
     * to the DMP.
     */
    float sens;
    unsigned short accel_sens;
    mpu_get_gyro_sens(&sens);
    gyro[0] = (long) (gyro[0] * sens);
    gyro[1] = (long) (gyro[1] * sens);
    gyro[2] = (long) (gyro[2] * sens);
    dmp_set_gyro_bias(gyro);
    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    dmp_set_accel_bias(accel);
    log_i("setting bias succesfully ......\r\n");
  }
}

uint8_t buffer[14];

int16_t MPU6050_FIFO[6][11];
int16_t Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setClockSource(uint8_t source)
 *功　　能:	    设置  MPU6050 的时钟源
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
 *******************************************************************************/
void MPU6050_setClockSource(uint8_t source) {
  IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT,
  MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/**************************实现函数********************************************
 // *函数原型:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
 // *功　　能:	    将新的ADC数据更新到 FIFO数组，进行滤波处理
 // *******************************************************************************/
void MPU6050_newValues(int16_t ax, int16_t ay, int16_t az, int16_t gx,
    int16_t gy, int16_t gz) {
  unsigned char i;
  int32_t sum = 0;
  for (i = 1; i < 10; i++) {	//FIFO 操作
    MPU6050_FIFO[0][i - 1] = MPU6050_FIFO[0][i];
    MPU6050_FIFO[1][i - 1] = MPU6050_FIFO[1][i];
    MPU6050_FIFO[2][i - 1] = MPU6050_FIFO[2][i];
    MPU6050_FIFO[3][i - 1] = MPU6050_FIFO[3][i];
    MPU6050_FIFO[4][i - 1] = MPU6050_FIFO[4][i];
    MPU6050_FIFO[5][i - 1] = MPU6050_FIFO[5][i];
  }
  MPU6050_FIFO[0][9] = ax;	//将新的数据放置到 数据的最后面
  MPU6050_FIFO[1][9] = ay;
  MPU6050_FIFO[2][9] = az;
  MPU6050_FIFO[3][9] = gx;
  MPU6050_FIFO[4][9] = gy;
  MPU6050_FIFO[5][9] = gz;

  sum = 0;
  for (i = 0; i < 10; i++) {	//求当前数组的合，再取平均值
    sum += MPU6050_FIFO[0][i];
  }
  MPU6050_FIFO[0][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[1][i];
  }
  MPU6050_FIFO[1][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[2][i];
  }
  MPU6050_FIFO[2][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[3][i];
  }
  MPU6050_FIFO[3][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[4][i];
  }
  MPU6050_FIFO[4][10] = sum / 10;

  sum = 0;
  for (i = 0; i < 10; i++) {
    sum += MPU6050_FIFO[5][i];
  }
  MPU6050_FIFO[5][10] = sum / 10;
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
  IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT,
  MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
 *功　　能:	    设置  MPU6050 加速度计的最大量程
 *******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
  IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
  MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
 *功　　能:	    设置  MPU6050 是否进入睡眠模式
 enabled =1   睡觉
 enabled =0   工作
 *******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
  IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
 *函数原型:		uint8_t MPU6050_getDeviceID(void)
 *功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将返回 0x68
 *******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {
  memset(buffer,0,sizeof(buffer));
  i2c_read(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
  return buffer[0];
}

/**************************实现函数********************************************
 *函数原型:		uint8_t MPU6050_testConnection(void)
 *功　　能:	    检测MPU6050 是否已经连接
 *******************************************************************************/
uint8_t MPU6050_testConnection(void) {
  if (MPU6050_getDeviceID() == 0x68)  //0b01101000;
    return 1;
  else
    return 0;
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
 *功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
 *******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
  IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT,
      enabled);
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
 *功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
 *******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
  IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT,
      enabled);
}

/**************************实现函数********************************************
 *函数原型:		void MPU6050_initialize(void)
 *功　　能:	    初始化 	MPU6050 以进入可用状态。
 *******************************************************************************/
void MPU6050_initialize(void) {
  MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //设置时钟
  MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_500); // MPU6050_GYRO_FS_2000    陀螺仪最大量程 +-1000度每秒
  MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G
  MPU6050_setSleepEnabled(0); //进入工作状态
  MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
  MPU6050_setI2CBypassEnabled(0);	//主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
}

/**************************************************************************
 函数功能：MPU6050内置DMP的初始化
 入口参数：无
 返回  值：无
 作    者：平衡小车之家
 **************************************************************************/
void DMP_Init(void) {
  if (MPU6050_getDeviceID() != 0x68)
    NVIC_SystemReset();
  if (!mpu_init(NULL)) {
    if (!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
      log_i("mpu_set_sensor complete ......\r\n");
    if (!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
      log_i("mpu_configure_fifo complete ......\r\n");
    if (!mpu_set_sample_rate(DEFAULT_MPU_HZ))
      log_i("mpu_set_sample_rate complete ......\r\n");
    if (!dmp_load_motion_driver_firmware())
      log_i("dmp_load_motion_driver_firmware complete ......\r\n");
    if (!dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation)))
      log_i("dmp_set_orientation complete ......\r\n");
    if (!dmp_enable_feature(
        DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL
            | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL))
      log_i("dmp_enable_feature complete ......\r\n");
    if (!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
      log_i("dmp_set_fifo_rate complete ......\r\n");
    run_self_test();
    if (!mpu_set_dmp_state(1))
      log_i("mpu_set_dmp_state complete ......\r\n");
  }
}
/**************************************************************************
 函数功能：读取MPU6050内置DMP的姿态信息
 入口参数：无
 返回  值：无
 作    者：平衡小车之家
 **************************************************************************/
void Read_DMP(void)
{
  unsigned long sensor_timestamp;
  unsigned char more;
  long quat[4];

  dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);

  if (sensors & INV_WXYZ_QUAT)
  {
    q0 = quat[0] / q30;   // w
    q1 = quat[1] / q30;   // x
    q2 = quat[2] / q30;   // y
    q3 = quat[3] / q30;   // z

    float s = -1.0; // The minus sign is added to match the graph printed on the board.
    // Rotation X - Y - Z (R = Rz * Ry * Rx)
    // The quaternion is q = qz*qy*qx operator
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	phi   = s*atan2f(q0 * q1 +  q2 * q3, 0.5 - (q1 * q1 + q2 * q2));
	theta = s*asinf(2  * (q0 * q2 - q3 * q1 )); //
	psi   = s*atan2f(q0 * q3 + q1* q2, 0.5 - (q2 * q2 + q3 * q3)   );

    // angle in degrees:
    roll  = rad2deg(phi);
    pitch = rad2deg(theta);
    yaw   = rad2deg(psi);
  }
}

/**************************************************************************
 Function: Wait to active the DMP mode and set up.
 Entry parameters: none
 Return value: none
 **************************************************************************/
void Calubration_DMP(void)
{
  float avg_gyro_x, avg_gyro_y, avg_gyro_z;

  do
  {
	  avg_gyro_x=avg_gyro_y=avg_gyro_z=0;
	  for (uint32_t i =0; i<100 ;i++)
	  {
		  Read_DMP();
		  avg_gyro_x += abs((float)gyro[0]);
		  avg_gyro_y += abs((float)gyro[1]);
		  avg_gyro_z += abs((float)gyro[2]);
	  }
	  avg_gyro_x/=100;
	  avg_gyro_y/=100;
	  avg_gyro_z/=100;

  } while(abs(gyro[0])>1 || abs(gyro[1])>1 || abs(gyro[2])>1);
}


void DMP_get_gyro_offsets(float* gx_offset, float* gy_offset, float* gz_offset)
{
	  *gx_offset = 0.0;
	  *gy_offset = 0.0;
	  *gz_offset = 0.0;
	  for(uint32_t i=0 ; i<2000; i++)
	  {
		  Read_DMP();
		  *gx_offset = *gx_offset + (float)gyro[0];
		  *gy_offset = *gy_offset + (float)gyro[1];
		  *gz_offset = *gz_offset + (float)gyro[2];
	  }

	  *gx_offset=*gx_offset/2000;
	  *gy_offset=*gy_offset/2000;
	  *gz_offset=*gz_offset/2000;

	  return;
}


void DMP_get_gravity(void){

	gravity[0] = 2*(q1*q3 - q0*q2);
	gravity[1] = 2*(q0*q1 + q2*q3);
	gravity[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	return;
}

/**************************************************************************
 Function: Read MPU6050 built-in temperature sensor data
 Entry parameters: none
 Return value: Celsius temperature
 **************************************************************************/
int Read_Temperature(void) {
  float Temp;
  uint8_t H, L;
  i2c_read(devAddr, MPU6050_RA_TEMP_OUT_H, 1, &H);
  i2c_read(devAddr, MPU6050_RA_TEMP_OUT_L, 1, &L);
  Temp = (H << 8) + L;
  if (Temp > 32768)
    Temp -= 65536;
  Temp = (36.53 + Temp / 340) * 10;
  return (int) Temp;
}




#define GYRO_ADDRESS_2 (0x69 << 1) // Shifted for HAL

int8_t init_gyro_registers(void)
{

	HAL_StatusTypeDef status;
    uint8_t reg, val;

    // PWR_MGMT_1 = 0x00 (wake up)
    uint8_t wakeup[] = {0x6B, 0x00};
    status = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDRESS_2, wakeup, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return -1;

   /* GYRO_CONFIG FS_SEL setting:
      0 -> ±250 °/s  (131 LSB/°/s)
      1 -> ±500 °/s   (65.5 LSB/°/s)
      2 -> ±1000 °/s  (32.8 LSB/°/s)
      3 -> ±2000 °/s  (16.4 LSB/°/s)
      Sensitivity (LSB/°/s) decreases as full-scale range increases.
      For best precision use FS_SEL = 0 (±250 °/s) if gyro rates are within that range. */

    uint8_t gyro_cfg[] = {0x1B, 0x00};
    status = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDRESS_2, gyro_cfg, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return -1;

    // Read back GYRO_CONFIG register (0x1B) to verify it's 0x08
    reg = 0x1B;
    status = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDRESS_2, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return -1;
    status = HAL_I2C_Master_Receive(&hi2c1, GYRO_ADDRESS_2, &val, 1, HAL_MAX_DELAY);
    if (status != HAL_OK || val != gyro_cfg[1]) return -1;

    // CONFIG = 0x01 (DLPF to ~188 Hz)
    uint8_t config[] = {0x1A, 0x01};
    status = HAL_I2C_Master_Transmit(&hi2c1, GYRO_ADDRESS_2, config, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return -1;

    return 0;
}


HAL_StatusTypeDef readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    HAL_StatusTypeDef status;

    // Step 1: Send the subAddress (register to start reading from)
    status = HAL_I2C_Master_Transmit(&hi2c1, address, &subAddress, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        // Handle or report transmission error
        return HAL_ERROR;
    }

    // Step 2: Read 'count' bytes into destination buffer
    status = HAL_I2C_Master_Receive(&hi2c1, address, dest, count, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        // Handle or report reception error
        return HAL_ERROR;
    }

    return status;
}

int8_t read_gyro_only(int16_t *gyro_axis)
{
	  uint8_t rawData[6];  // x/y/z gyro register data stored here
	  HAL_StatusTypeDef status;
	  status = readBytes(GYRO_ADDRESS_2, 0x43, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	  if (status != HAL_OK) return -1;
	  gyro_axis[0] = -(int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	  gyro_axis[1] = -(int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
	  gyro_axis[2] = -(int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	  return 0;

	return 0;
}





//------------------End of File----------------------------
