#include <ms4525.h>
#include <main.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern float Vair;
float tempC, pressD;
int16_t p_raw, t_raw;
uint8_t data2[4];
int conms4525;
float diff_press_PSI, p_rawF, pressDnew, pressDiff, pressDiffstar, pressDifferential;

void ms4525_init() // Инициализация датчика
{
	HAL_StatusTypeDef ret1 = HAL_I2C_IsDeviceReady(&hi2c1, (DEVICE_ADRESS2 <<1) + 0, 100, 100);
	if (ret1 == HAL_OK)
	{
	conms4525=1;
	}
	else
	{
	conms4525=2;
	}
}

void ms4525_read() // Чтение данных с датчика и вычисление скорости набегающего потока
{
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADRESS2 <<1) + 1, READ_ADRESS2, 1, data2, 4, 100);
	p_raw = (((int16_t)data2[0] << 8) + data2[1]);
	p_raw = 0x3FFF & p_raw;
	p_rawF = p_raw + 6.71;
	t_raw = (((int16_t)data2[2] << 8) + data2[3]);
	t_raw = (0xFFE0 & t_raw) >> 5;

	tempC  = ((200.0f * t_raw) / 2047) - 50;

	float density = 1.16; // Плотность воздуха

	pressDiffstar = (8192-p_raw)*1.02095;
	pressDiff = -0.5383*pow(p_raw,2)-0.4737*(p_raw)+8183.6;
	pressDifferential = (-1.1087*p_raw) + 9070.5;

    if (pressDifferential > 0) // Вычисление скорости набегающего потока
    {
    	Vair = (sqrtf(2*pressDifferential/density));
    }
    else
    {
        Vair = 0;
    }
}
