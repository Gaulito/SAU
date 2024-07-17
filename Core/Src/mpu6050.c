#include <mpu6050.h>
#include <kalman.h>
#include <main.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern double total_G, roll, pitch;

int16_t x_accRAW,y_accRAW,z_accRAW;
uint8_t connected=0;
uint8_t accel=0;
uint8_t pwrmgmt=0;
uint8_t	temp_data=0;
uint8_t data[6];
int16_t x_gyroRAW;
int16_t y_gyroRAW;
int16_t z_gyroRAW;
int16_t acc;
double rotation;
double gyro;
double x_gyro;
double y_gyro;
double z_gyro;
double x_gyro_c;
double y_gyro_c;
double z_gyro_c;
double x_G;
double y_G;
double z_G;
double roll0;
double pitch0;

void mpu6050_init() 	// Инициализация датчика
{
  	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, (DEVICE_ADRESS <<1) + 0, 1, 100);
    if (ret == HAL_OK)
    {
  	  connected=1;
    }
    else
    {
  	  connected=2;
    }

    temp_data = 0;
    ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADRESS <<1), REG_PWR_MGMT, 1, &temp_data, 1, 100);
    if (ret == HAL_OK)
    {
  	  pwrmgmt=1;
    }
    else
    {
  	  pwrmgmt=2;
    }

    temp_data = FS_GYRO_250;
    ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADRESS <<1), REG_CONFIG_GYRO, 1, &temp_data, 1, 100);
    if (ret == HAL_OK)
    {
  	  gyro=1;
    }
    else
    {
  	  gyro=2;
    }

    temp_data = FS_ACC_2G;
    ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADRESS <<1), REG_CONGIG_ACC, 1, &temp_data, 1, 100);
    if (ret == HAL_OK)
    {
  	  accel=1;
    }
    else
    {
  	  accel=2;
    }

    pitch = pitch0;
    roll = roll0;
}

void mpu6050_read() // Чтение сырых данных из датчика
{
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADRESS <<1) + 1, REG_ACC_DATA, 1, data, 6, 100);
	x_accRAW = (((int16_t)data[0] << 8) + data[1])-325;
	y_accRAW = (((int16_t)data[2] << 8) + data[3])-104;
	z_accRAW = (((int16_t)data[4] << 8) + data[5])+1881;
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADRESS <<1) + 1, REG_GYRO_DATA, 1, data, 6, 100);
	x_gyroRAW = (((int16_t)data[0] << 8) + data[1])-53;
	y_gyroRAW = (((int16_t)data[2] << 8) + data[3])+93;
	z_gyroRAW = (((int16_t)data[4] << 8) + data[5])+57;
	acc = sqrt(pow(x_accRAW,2)+pow(y_accRAW,2)+pow(z_accRAW,2));
	gyro = sqrt(pow(x_gyroRAW,2)+pow(y_gyroRAW,2)+pow(z_gyroRAW,2));
	x_G = x_accRAW/16384.0;
	y_G = y_accRAW/16384.0;
	z_G = z_accRAW/16384.0;
	total_G = acc/16384.0;
	x_gyro = x_gyroRAW/131.0 * M_PI/180;
	y_gyro = y_gyroRAW/131.0 * M_PI/180;
	z_gyro = z_gyroRAW/131.0 * M_PI/180;
	rotation = gyro/131.0;

	getKalmanA(); // Фильтрация полученных значений ускорений с помощью фильтра Калмана
	getKalmanOMEGA(); // Фильтрация полученных значений угловых скоростей с помощью фильтра Калмана
}

void mpu6050_getAngle() // Вычисление крена и тангажа
{

	if (fabs(total_G-9.8) <= 0.4)
	{
		roll = atan(y_accRAW/(sqrt(pow(x_accRAW,2)+pow(z_accRAW,2))));
		pitch = atan(-x_accRAW/(sqrt(pow(y_accRAW,2)+pow(z_accRAW,2))));
	}
	else
	{
		pitch0 = pitch;
		roll0 = roll;
		pitch = y_gyro * sin(roll0) + z_gyro * cos(roll0) + pitch;
		roll = x_gyro - tan(pitch0)*(y_gyro * cos(roll0) - z_gyro * sin(roll0)) + roll;
	}
}
