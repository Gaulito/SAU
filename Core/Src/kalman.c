#include <main.h>
#include <kalman.h>
#include <math.h>
#include <mpu6050.h>
extern double x_G, y_G, z_G;
extern VECTOR accV, RV, VelV;

extern double x_gyro,y_gyro,z_gyro;
extern double x_gyro_c, y_gyro_c, z_gyro_c;

float kalman_gain, current_estimate, last_estimate, err_estimate;
float kalman_gain1, current_estimate1, last_estimate1, err_estimate1;
float kalman_gain2, current_estimate2, last_estimate2, err_estimate2;

float kalman_gain3, current_estimate3, last_estimate3, err_estimate3;
float kalman_gain4, current_estimate4, last_estimate4, err_estimate4;
float kalman_gain5, current_estimate5, last_estimate5, err_estimate5;

float kalman_gain6, current_estimate6, last_estimate6, err_estimate6;
float kalman_gain7, current_estimate7, last_estimate7, err_estimate7;
float kalman_gain8, current_estimate8, last_estimate8, err_estimate8;

float kalman_gain9, current_estimate9, last_estimate9, err_estimate9;

float kalman_gain10, current_estimate10, last_estimate10, err_estimate10;
float kalman_gain11, current_estimate11, last_estimate11, err_estimate11;
float kalman_gain12, current_estimate12, last_estimate12, err_estimate12;


	  float simpleKalmanX(float newVal) {
		  float err_measure = 346.0;  // примерный шум измерений
		  float q = 1.8;   // скорость изменения значений 0.001-1, варьировать самому
		  err_estimate = err_measure;
		  kalman_gain = (float)err_estimate / (err_estimate + err_measure);
	   	  current_estimate = (float)last_estimate + (float)kalman_gain * (newVal - (float)last_estimate);
	   	  err_estimate =  (1.0 - kalman_gain) * err_estimate + fabs(last_estimate - current_estimate) * q;
	   	  last_estimate = current_estimate;
	   	  return current_estimate;
	  }

	  float simpleKalmanY(float newVal1) {
		  float err_measure1 = 306.0;  // примерный шум измерений
		  float q1 = 1.0;   // скорость изменения значений 0.001-1, варьировать самому
		  err_estimate1 = err_measure1;
		  kalman_gain1 = (float)err_estimate1 / (err_estimate1 + err_measure1);
		  current_estimate1 = last_estimate1 + (float)kalman_gain1 * (newVal1 - (float)last_estimate1);
		  err_estimate1 =  (1.0 - kalman_gain1) * err_estimate1 + fabs(last_estimate1 - current_estimate1) * q1;
		  last_estimate1 = current_estimate1;
		  return current_estimate1;
	  }

	  float simpleKalmanZ(float newVal2) {
		  float err_measure2 = 430.0;  // примерный шум измерений
		  float q2 = 0.8;   // скорость изменения значений 0.001-1, варьировать самому
		  err_estimate2 = err_measure2;
		  kalman_gain2 = (float)err_estimate2 / (err_estimate2 + err_measure2);
		  current_estimate2 = last_estimate2 + (float)kalman_gain2 * (newVal2 - last_estimate2);
		  err_estimate2 =  (1.0 - kalman_gain2) * err_estimate2 + fabs(last_estimate2 - current_estimate2) * q2;
		  last_estimate2 = current_estimate2;
		  return current_estimate2;
	  }




	  float simpleKalmanXE(float newVal3) {
		  float err_measure3 = 346.0;  // примерный шум измерений
		  float q3 = 1.8;   // скорость изменения значений 0.001-1, варьировать самому
		  err_estimate3 = err_measure3;
		  kalman_gain3 = (float)err_estimate3 / (err_estimate3 + err_measure3);
	   	  current_estimate3 = (float)last_estimate3 + (float)kalman_gain3 * (newVal3 - (float)last_estimate3);
	   	  err_estimate3 =  (1.0 - kalman_gain3) * err_estimate3 + fabs(last_estimate3 - current_estimate3) * q3;
	   	  last_estimate3 = current_estimate3;
	   	  return current_estimate3;
	  }

	  float simpleKalmanYE(float newVal4) {
		  float err_measure4 = 306.0;  // примерный шум измерений
		  float q4 = 1.0;   // скорость изменения значений 0.001-1, варьировать самому
		  err_estimate4 = err_measure4;
		  kalman_gain4 = (float)err_estimate4 / (err_estimate4 + err_measure4);
		  current_estimate4 = last_estimate4 + (float)kalman_gain4 * (newVal4 - (float)last_estimate4);
		  err_estimate4 =  (1.0 - kalman_gain4) * err_estimate4 + fabs(last_estimate4 - current_estimate4) * q4;
		  last_estimate4 = current_estimate4;
		  return current_estimate4;
	  }

	  float simpleKalmanZE(float newVal5) {
		  float err_measure5 = 430.0;  // примерный шум измерений
		  float q5 = 0.8;   // скорость изменения значений 0.001-1, варьировать самому
		  err_estimate5 = err_measure5;
		  kalman_gain5 = (float)err_estimate5 / (err_estimate5 + err_measure5);
		  current_estimate5 = last_estimate5 + (float)kalman_gain5 * (newVal5 - last_estimate5);
		  err_estimate5 =  (1.0 - kalman_gain5) * err_estimate5 + fabs(last_estimate5 - current_estimate5) * q5;
		  last_estimate5 = current_estimate5;
		  return current_estimate5;
	  }




	  float simpleKalmanOMEGA_X(float newVal6) {
		  float err_measure6 = 346.0;  // примерный шум измерений
		  float q6 = 1.8;   // скорость изменения значений 0.001-1, варьировать самому
		  err_estimate6 = err_measure6;
		  kalman_gain6 = (float)err_estimate6 / (err_estimate6 + err_measure6);
	   	  current_estimate6 = (float)last_estimate6 + (float)kalman_gain6 * (newVal6 - (float)last_estimate6);
	   	  err_estimate6 =  (1.0 - kalman_gain6) * err_estimate6 + fabs(last_estimate6 - current_estimate6) * q6;
	   	  last_estimate6 = current_estimate6;
	   	  return current_estimate6;
	  }

	  float simpleKalmanOMEGA_Y(float newVal7) {
		  float err_measure7 = 306.0;  // примерный шум измерений
		  float q7 = 1.0;   // скорость изменения значений 0.001-1, варьировать самому
		  err_estimate7 = err_measure7;
		  kalman_gain7 = (float)err_estimate7 / (err_estimate7 + err_measure7);
		  current_estimate7 = last_estimate7 + (float)kalman_gain7 * (newVal7 - (float)last_estimate7);
		  err_estimate7 =  (1.0 - kalman_gain7) * err_estimate7 + fabs(last_estimate7 - current_estimate7) * q7;
		  last_estimate7 = current_estimate7;
		  return current_estimate7;
	  }

	  float simpleKalmanOMEGA_Z(float newVal8) {
		  float err_measure8 = 430.0;  // примерный шум измерений
		  float q8 = 0.8;   // скорость изменения значений 0.001-1, варьировать самому
		  err_estimate8 = err_measure8;
		  kalman_gain8 = (float)err_estimate8 / (err_estimate8 + err_measure8);
		  current_estimate8 = last_estimate8 + (float)kalman_gain8 * (newVal8 - last_estimate8);
		  err_estimate8 =  (1.0 - kalman_gain8) * err_estimate8 + fabs(last_estimate8 - current_estimate8) * q8;
		  last_estimate8 = current_estimate8;
		  return current_estimate8;
	  }

	  // Функция задействования фильтра для получения рыскания
	  double simpleKalmanYAW(double newVal9) {
		  double err_measure9 = 430.0;  // примерный шум измерений
		  double q9 = 0.8;   // скорость изменения значений 0.001-1, варьировать самому
		  err_estimate9 = err_measure9;
		  kalman_gain9 = (double)err_estimate9 / (err_estimate9 + err_measure9);
		  current_estimate9 = last_estimate9 + (double)kalman_gain9 * (newVal9 - last_estimate9);
		  err_estimate9 =  (1.0 - kalman_gain9) * err_estimate9 + fabs(last_estimate9 - current_estimate9) * q9;
		  last_estimate9 = current_estimate9;
		  return current_estimate9;
	  }

	  float simpleKalmanVxGPS(float newVal10) {
	        float err_measure10 = 430.0;  // примерный шум измерений
	        float q10 = 0.8;   // скорость изменения значений 0.001-1, варьировать самому
	        err_estimate10 = err_measure10;
	        kalman_gain10 = (float)err_estimate10 / (err_estimate10 + err_measure10);
	        current_estimate10 = last_estimate10 + (float)kalman_gain10 * (newVal10 - last_estimate10);
	        err_estimate10 =  (1.0 - kalman_gain10) * err_estimate10 + fabs(last_estimate10 - current_estimate10) * q10;
	        last_estimate10 = current_estimate10;
	        return current_estimate10;
	  }

	  float simpleKalmanVyGPS(float newVal11) {
	        float err_measure11 = 430.0;  // примерный шум измерений
	        float q11 = 0.8;   // скорость изменения значений 0.001-1, варьировать самому
	        err_estimate11 = err_measure11;
	        kalman_gain11 = (float)err_estimate11 / (err_estimate11 + err_measure11);
	        current_estimate11 = last_estimate11 + (float)kalman_gain11 * (newVal11 - last_estimate11);
	        err_estimate11 =  (1.0 - kalman_gain11) * err_estimate11 + fabs(last_estimate11 - current_estimate11) * q11;
	        last_estimate11 = current_estimate11;
	        return current_estimate11;
	  }

	  float simpleKalmanVzGPS(float newVal12) {
	        float err_measure12 = 430.0;  // примерный шум измерений
	        float q12 = 0.8;   // скорость изменения значений 0.001-1, варьировать самому
	        err_estimate12 = err_measure12;
	        kalman_gain12 = (float)err_estimate12 / (err_estimate12 + err_measure12);
	        current_estimate12 = last_estimate12 + (float)kalman_gain12 * (newVal12 - last_estimate12);
	        err_estimate12 =  (1.0 - kalman_gain12) * err_estimate12 + fabs(last_estimate12 - current_estimate12) * q12;
	        last_estimate12 = current_estimate12;
	        return current_estimate12;
	  }




void getKalmanA() // Процедура задействования фильтра для получения ускорения по 3-ем осям
{
	accV.x = simpleKalmanX(x_G);
	accV.y = simpleKalmanY(y_G);
	accV.z = simpleKalmanZ(z_G);
}

void getKalmanCSE() // Процедура задействования фильтра для получения проекций радиус-вектора в стартовой системе координат на 3 оси
{
	RV.x = simpleKalmanXE(RV.x);
	RV.y = simpleKalmanYE(RV.y);
	RV.z = simpleKalmanZE(RV.z);
}

void getKalmanOMEGA() // Процедура задействования фильтра для получения угловых скоростей по 3-ем осях
{
	x_gyro_c = simpleKalmanOMEGA_X(x_gyro);
	y_gyro_c = simpleKalmanOMEGA_Y(y_gyro);
	z_gyro_c = simpleKalmanOMEGA_Z(z_gyro);
}

void getKalmanVGPS() // Процедура задействования фильтра для получения GNSS скоростей по 3-ем осям
{
	VelV.x = simpleKalmanVxGPS(VelV.x);
	VelV.y = simpleKalmanVyGPS(VelV.y);
	VelV.z = simpleKalmanVzGPS(VelV.z);
}
