#include <hmc5883.h>
#include <main.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
double x_mag, y_mag, z_mag, total_mag;
int16_t x_magRAW, y_magRAW, z_magRAW;
uint8_t temp_data1=0;
uint8_t connected1=0;
uint8_t configA, configB, configData;
uint8_t data1[6];
int16_t mag;
void hmc5883_init() // Инициализация датчика
{
  	HAL_StatusTypeDef ret1 = HAL_I2C_IsDeviceReady(&hi2c1, (DEVICE_ADRESS1 <<1) + 0, 100, 100);
    if (ret1 == HAL_OK)
    {
  	  connected1=1;
    }
    else
    {
  	  connected1=2;
    }

    temp_data1 = HERNYA_9;
    ret1 = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADRESS1 <<1), HERNYA_Z9, 1, &temp_data1, 1, 100);
    if (ret1 == HAL_OK)
    {
  	  configA=1;
    }
    else
    {
  	  configA=2;
    }

    temp_data1 = HERNYA_10;
    ret1 = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADRESS1 <<1), HERNYA_Z10, 1, &temp_data1, 1, 100);
    if (ret1 == HAL_OK)
    {
  	  configB=1;
    }
    else
    {
  	  configB=2;
    }

    temp_data1 = HERNYA_11;
    ret1 = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADRESS1 <<1), HERNYA_Z11, 1, &temp_data1, 1, 100);
    if (ret1 == HAL_OK)
    {
  	  configData=1;
    }
    else
    {
  	  configData=2;
    }
}

void hmc5883_read() // Чтение данных с датчика
{
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADRESS1 <<1) + 1, HERNYA_CHITAT, 1, data1, 6, 100);
	x_magRAW = (((int16_t)data1[0] << 8) + data1[1]);
	y_magRAW = (((int16_t)data1[2] << 8) + data1[3]);
	z_magRAW = (((int16_t)data1[4] << 8) + data1[5]);

	mag = sqrt(pow(x_magRAW,2)+pow(y_magRAW,2)+pow(z_magRAW,2));
	total_mag = mag/1370.0;
	x_mag = x_magRAW/1370.0;
	y_mag = y_magRAW/1370.0;
	z_mag = z_magRAW/1370.0;
}

double hmc5883_getYaw(double Pitch, double Roll) // Получение рыскания
{
	VECTOR ye, ze;
	double Yaw = 0, sYaw, cYaw, zeM;

	hmc5883_read();

	ye.x = sin(Pitch);
	ye.y = cos(Pitch) * cos(Roll);
	ye.z = -cos(Pitch) * sin(Roll);

	ze.x = ye.z * y_mag - ye.y * z_mag;
	ze.y = -ye.z * x_mag + ye.x * z_mag;
	ze.z = ye.y * x_mag - ye.x * y_mag;

	zeM = sqrt(pow(ze.x, 2) + pow(ze.y, 2) + pow(ze.z, 2));
	ze.x = ye.x / zeM;
	ze.y = -ye.y / zeM;
	ze.z = ye.z / zeM;

	(Pitch != 0) ? (sYaw = -ze.x / cos(Pitch)) : (sYaw = 0);

	(sin(Roll) == 0) ? (cYaw = (ze.z + sin(Roll) * sin(Pitch) * sYaw) / cos(Roll)) : (cYaw = (ze.y - cos(Roll) * sin(Pitch) * sYaw) / sin(Roll));

	(sYaw >= 0) ? (Yaw = acos(cYaw)) : (Yaw = M_PI + acos(cYaw));

	return Yaw;
}
