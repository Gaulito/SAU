/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "trajectory.h"
#include "mpu6050.h"
#include "hmc5883.h"
#include "ms4525.h"
#include "bmp180.h"
#include "kalman.h"
#include "UartRingbuffer.h"
#include "NMEA.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
int32_t i = 0;
double GPSLastCalcTick, yelast, Ltarget = 0, LdropG;

VECTOR accV, RV, RV0, VelV;
//char msg[50];
//double total, totalroll, totalpitch, summedG, controlG, totalGmax, totalGmin = 1;    int32_t xa,ya,za;
double total_G, roll = 0, pitch = 0, yaw, beta = 0, a_traj = 0, phi_traj = 0, AltBarr0;
float Vair;


const float Pe = 0.5, Ie = 0.01, De = 0.04;
const float Pr = 0.6, Ir = 0.01, Dr = 0.5;
const float Pp = 0.4, Ip = 0.02, Dp = 0.02;
const float Py = 0.5, Iy = 0.01, Dy = 0.04;
const float Vsv = 14, Vpolet = 20;

float Zt_past = 0, Zt_sum = 0, Yt_past = 0, Yt_sum = 0, beta_past = 0, beta_sum = 0, Vt_past = 0, Vt_sum = 0;


double local_to_global[3][3], global_to_local[3][3];

double wairl[3], wairg[3] = {0, 0, 0}, Va[3], Vam;

POINT_TR P_current, P_next;

char UpT = 1;
extern int traj1_Length, trajUP_Length;
extern double R, Hud, Hdrop;
int Npoint = 0;

double angle_rudder_up = 90, angle_rudder_course = 90, angle_aileron = 90, throttle = 0;

//double x0, z0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

GPSSTRUCT gpsData;

void Turn_Matrix(double Yaw, double Pitch, double Roll, double (*A)[3], double (*AT)[3])
{
	A[0][0] = AT[0][0] = cos(Pitch) * cos(Yaw);
	A[0][1] = AT[1][0] = - cos(Roll) * cos(Yaw) * sin(Pitch) + sin(Roll) * sin(Yaw);
	A[0][2] = AT[2][0] = cos(Yaw) * sin(Roll) * sin(Pitch) + cos(Roll) * sin(Yaw);
	A[1][0] = AT[0][1] = sin(Pitch);
	A[1][1] = AT[1][1] = cos(Roll) * cos(Pitch);
	A[1][2] = AT[2][1] = - cos(Pitch) * cos(Roll);
	A[2][0] = AT[2][0] = - cos(Pitch) * sin(Yaw);
	A[2][1] = AT[1][2] = cos(Roll) * sin(Yaw) * sin(Pitch) + sin(Roll) * cos(Yaw);
	A[2][2] = AT[2][2] = - sin(Yaw) * sin(Roll) * sin(Pitch) + cos(Roll) * cos(Yaw);
}

void Matrix_multiply(double A[3][3], double B[3], double *C)
{
	for(int i = 0; i < 3; i++) {
	   C[i] = 0;
	   for(int k = 0; k < 3; k++)
	   C[i] += A[i][k] * B[k];
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  mpu6050_init();
  hmc5883_init();
  ms4525_init();
  bmp180_init();

  HAL_Delay (500);

  AltBarr0 = bmp180_getalt(0);

  while (getGPS(&gpsData) != 0);
  RV0.y = gpsData.ggastruct.alt.altitude;

  HAL_Delay (500);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  int Set_Servo_RightWing(double Angle) // from 0 to 180 degrees
  {
    uint16_t Pulse_length = 500;
    if (Angle > 0  &&  Angle <= 180) {
        Pulse_length += (2700-500)/180 * (int)Angle;
    }

    TIM1->CCR1=Pulse_length;
    return 0;
  }

  int Set_Servo_LeftWing(double Angle) // from 0 to 180 degrees
  {
    uint16_t Pulse_length = 500;
    if (Angle > 0  &&  Angle <= 180) {
        Pulse_length += (2700-500)/180 * (int)Angle;
    }

    TIM1->CCR2=Pulse_length;
    return 0;
  }

  int Set_Servo_Elevator(double Angle) // from 0 to 180 degrees
  {
    uint16_t Pulse_length = 500;
    if (Angle > 0  &&  Angle <= 180) {
        Pulse_length += (2700-500)/180 * (int)Angle;
    }

    TIM1->CCR3=Pulse_length;
    return 0;
  }

  int Set_Servo_Rudder(double Angle) // from 0 to 180 degrees
  {
    uint16_t Pulse_length = 500;
    if (Angle > 0  &&  Angle <= 180) {
        Pulse_length += (2700-500)/180 * (int)Angle;
    }

    TIM1->CCR4=Pulse_length;
    return 0;
  }

  int Set_Servo_CargoHatch(double Angle) // from 0 to 180 degrees
  {
    uint16_t Pulse_length = 500;
    if (Angle > 0  &&  Angle <= 180) {
        Pulse_length += (2700-500)/180 * (int)Angle;
    }

    TIM2->CCR1=Pulse_length;
    return 0;
  }

  int Set_Engine_Speed(double Percentage) // from 0 to 100 percents
  {
    uint16_t Pulse_length = 500;
    if (Percentage > 0  &&  Percentage <= 180) {
        Pulse_length += (2700-500)/100 * (int)Percentage;
    }

    TIM2->CCR2=Pulse_length;
    return 0;
  }

  LdropG = Vpolet * sqrt(2 * Hud / 9.8);


  Set_Servo_RightWing(angle_aileron);
  Set_Servo_LeftWing(angle_aileron);
  Set_Servo_Elevator(angle_rudder_up);
  Set_Servo_Rudder(angle_rudder_course);
  Set_Servo_CargoHatch(0);
  Set_Engine_Speed(throttle);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  mpu6050_read();
	  hmc5883_read();

	  //TempBarr = bmp180_gettemp();
	  //PressBarr = bmp180_getpress(0);
	  //xa = x_accRAW + xa;
	  //ya = y_accRAW + ya;
	  //za = z_accRAW + za;
	  //totalroll = roll + totalroll;
	  //totalpitch = pitch + totalpitch;
	  //summedG = total_G + summedG;
	  //controlG = summedG/i;
	  //if (total_G > totalGmax)
	  //{
		//  totalGmax = total_G;
	  //}
	  //if (total_G < totalGmin)
	  //{
		//  totalGmin = total_G;
	  //}
	  i++;

	  //sprintf(msg, "%d %d %d %d %d %d\n", x_accRAWc, y_accRAWc, z_accRAWc, x_accRAW, y_accRAW, z_accRAW);
	  //HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFF);


	  if (getGPS(&gpsData) == 0)
	  {

		  if (gpsData.ggastruct.lcation.NS == 'N')
		  {
			  RV.x = 6378000 * (gpsData.ggastruct.lcation.latitude * M_PI/180 - RV0.x);
		  }
		  else
		  {
			  RV.x = -6378000 * (gpsData.ggastruct.lcation.latitude * M_PI/180 + RV0.x);
		  }

		  if (gpsData.ggastruct.lcation.EW == 'E')
		  {
			  RV.z = 6378000 * cos(gpsData.ggastruct.lcation.latitude * M_PI/180) * (gpsData.ggastruct.lcation.longitude * M_PI/180 - RV0.z);
		  }
		  else
		  {
			  RV.z = -6378000 * cos(gpsData.ggastruct.lcation.latitude * M_PI/180) * (gpsData.ggastruct.lcation.longitude * M_PI/180 + RV0.z);
		  }

		  RV.y = 0.5 * (bmp180_getalt(0) - AltBarr0) + 0.5 * (gpsData.ggastruct.alt.altitude - RV0.y);

	  }

	  getKalmanCSE();

	  yaw = hmc5883_getYaw(pitch, roll);
	  yaw = simpleKalmanYAW(yaw);


	  float SOG = (gpsData.rmcstruct.speed)*0.514;
	  float course = ((gpsData.rmcstruct.course)*M_PI/180) - (2*M_PI);
	  float deltaT = (HAL_GetTick()-GPSLastCalcTick)/1000;
	  VelV.x = SOG * cos(course);
	  VelV.y = SOG * sin(course);
	  VelV.z = (RV.y - yelast)/deltaT;
	  yelast = RV.y;
	  GPSLastCalcTick = HAL_GetTick();
	  getKalmanVGPS();


	  ms4525_read();
	  Turn_Matrix(yaw, pitch, roll, local_to_global, global_to_local);

	  double VGPS[3] = {VelV.x, VelV.y, VelV.z}, Vlocal[3];
	  Matrix_multiply(global_to_local, VGPS, Vlocal);
	  double wair = Vlocal[0] - Vair;

	  Matrix_multiply(global_to_local, wairg, wairl);
	  wairl[0] = wair;
	  Matrix_multiply(local_to_global, wairl, wairg);

	  //***********************
	  Va[0] = Vlocal[0] - wairl[0];
	  Va[1] = Vlocal[1] - wairl[1];
	  Va[2] = Vlocal[2] - wairl[2];
	  double VaM = sqrt(pow(Va[0], 2) + pow(Va[1], 2) + pow(Va[2], 2));
	  beta = VaM > 0 ?  asin(Va[2] / VaM) : 0;

	  double VelM = sqrt(pow(VelV.x, 2) + pow(VelV.y, 2) + pow(VelV.z, 2));
	  double VelP = sqrt(pow(VelV.x, 2) + pow(VelV.z, 2));
	  a_traj = VelM > 0 ? (VelV.y >= 0 ? asin(VelV.y/VelM) : 2*M_PI - asin(VelV.y/VelM)) : 0;
	  phi_traj = VelP > 0 ? (VelV.x >= 0 ? (VelV.z >= 0 ? 2*M_PI - asin(VelV.z/VelP) : asin(VelV.z/VelP)) : M_PI + asin(VelV.z/VelP)) : 0;


	  if (i == 1) Vslet_Trajectory(a_traj, phi_traj, RV.x, RV.y, RV.z);


	  if (UpT && Npoint == trajUP_Length - 1)
	  {
		  Npoint = 1;
		  UpT = 0;
		  Ltarget = 1;
	  }




	  P_current = Get_point_traj(Npoint, UpT);

	  if (Ltarget < 1 && traj1_Length >= Npoint + 2)
	  {
		  double phi_current, phi_next,a_current, a_next;
		  P_next = Get_point_traj(Npoint + 1, UpT);

		  if ((UpT && P_next.type != 13) || (!UpT && P_next.type < 14)) Npoint++;
		  else {
			  if (P_next.type == 14)
			  {
				  phi_current = fabs(P_current.phi - phi_traj <= -M_PI ? 2*M_PI + P_current.phi - phi_traj :
						  (P_current.phi - phi_traj > M_PI ? -2*M_PI + P_current.phi - phi_traj : P_current.phi - phi_traj));
				  phi_next = fabs(P_next.phi - phi_traj <= -M_PI ? 2*M_PI + P_next.phi - phi_traj :
						  (P_next.phi - phi_traj > M_PI ? -2*M_PI + P_next.phi - phi_traj : P_next.phi - phi_traj));
				  if (phi_current >= phi_next) Npoint++;
			  }
			  else
			  {
				  a_current = fabs(P_current.alpha - a_traj <= -M_PI ? 2*M_PI + P_current.alpha - a_traj :
						  (P_current.alpha - a_traj > M_PI ? -2*M_PI + P_current.alpha - a_traj : P_current.alpha - a_traj));
				  a_next = fabs(P_next.alpha - a_traj <= -M_PI ? 2*M_PI + P_next.alpha - a_traj :
						  (P_next.alpha - a_traj > M_PI ? -2*M_PI + P_next.alpha - a_traj : P_next.alpha - a_traj));
				  if (a_current >= a_next) Npoint++;
			  }
		  }
	  }



	  double Laim = 0, Laimxz = 0, a_dof = 0, phi_dof = 0, a_adof = 0, phi_adof = 0;

	  if (P_current.Ry > 1000)
	  {
		  a_dof = P_current.alpha - a_traj <= -M_PI ? 2*M_PI + P_current.alpha - a_traj : (P_current.alpha - a_traj > M_PI ? -2*M_PI + P_current.alpha - a_traj : P_current.alpha - a_traj);
	  }

	  if (P_current.Rxz > 1000)
	  {
		  phi_dof = P_current.phi - phi_traj <= -M_PI ? 2*M_PI + P_current.phi - phi_traj : (P_current.phi - phi_traj > M_PI ? -2*M_PI + P_current.phi - phi_traj : P_current.phi - phi_traj);
	  }

	  if ((UpT && P_next.type != 13) || (!UpT && P_next.type < 14))
	  {
		  double  a_aim, phi_aim;

		  Laim = sqrt(pow(P_current.x - RV.x, 2) + pow(P_current.y - RV.y, 2) + pow(P_current.z - RV.z, 2));
		  Laimxz = sqrt(pow(P_current.x - RV.x, 2) + pow(P_current.z - RV.z, 2));

		  a_aim = Laim > 0 ? (P_current.y >= RV.y ? asin((P_current.y - RV.y)/Laim) : 2*M_PI - asin((P_current.y - RV.y)/Laim)) : 0;
		  phi_aim = Laimxz > 0 ? (P_current.x - RV.x >= 0 ? (P_current.z - RV.z >= 0 ? 2*M_PI - asin((P_current.z - RV.z)/Laimxz) : asin((P_current.z - RV.z)/Laimxz)) : M_PI + asin((P_current.z - RV.z)/Laimxz)) : 0;

		  a_adof = P_current.alpha - a_aim <= -M_PI ? 2*M_PI + P_current.alpha - a_aim : (P_current.alpha - a_aim > M_PI ? -2*M_PI + P_current.alpha - a_aim : P_current.alpha - a_aim);
		  phi_adof = P_current.phi - phi_aim <= -M_PI ? 2*M_PI + P_current.phi - phi_aim : (P_current.phi - phi_aim > M_PI ? -2*M_PI + P_current.phi - phi_aim : P_current.phi - phi_aim);
	  }



	  Ltarget = ((UpT && P_next.type != 13) || (!UpT && P_next.type < 14)) ? Laimxz * cos(phi_adof) : 0;

	  double Rxz, Ry, buf;

	  if (P_current.Rxz > 1000)
	  {
		  buf = phi_dof - Laimxz * sin(phi_adof) / 20;
		  buf = buf > 1 ? 1 : (buf < -1 ? -1 : buf);
		  Rxz = buf == 0 ? 9999 : R/buf;
	  }
	  else Rxz = P_current.Rxz;

	  if (P_current.Ry > 1000)
	  {
		  buf = a_dof - Laim * sin(a_adof) / 30;
		  buf = buf > 1 ? 1 : (buf < -1 ? -1 : buf);
		  Ry = buf == 0 ? 9999 : R/buf;
	  }
	  else Ry = P_current.Ry;




	  double target_accelerateG[3], target_accelerateL[3];

	  if (P_current.type == 16)
	  {
		  target_accelerateG[0] = - pow(VelM,2) / Ry * sin(a_traj) * cos(phi_traj) * (fabs(roll) > M_PI /2 ? -1 : 1);
		  target_accelerateG[1] = 9.8 + pow(VelM,2) / Ry * cos(a_traj) * (fabs(roll) > M_PI /2 ? -1 : 1);
		  target_accelerateG[2] = pow(VelM,2) / Ry * sin(a_traj) * sin(phi_traj) * (fabs(roll) > M_PI /2 ? -1 : 1);
	  }
	  else {
		  target_accelerateG[0] = pow(VelM,2) / Rxz * sin(phi_traj) - pow(VelM,2) / Ry * sin(a_traj) * cos(phi_traj);
		  target_accelerateG[1] = 9.8 + pow(VelM,2) / Ry * cos(a_traj);
		  target_accelerateG[2] = pow(VelM,2) / Rxz * cos(phi_traj) + pow(VelM,2) / Ry * sin(a_traj) * sin(phi_traj);
	  }
	  Matrix_multiply(global_to_local, target_accelerateG, target_accelerateL);


	  // PIDs **********************************
	  // пид регулятор двигателя
	  double Vt = P_current.type == 10 || P_current.type == 15 ? Vsv + (Vpolet - Vsv) - VelM * (RV.y / Hud) : Vpolet - VelM;

	  Vt_sum = Vt_sum + Vt;
	  throttle = P_current.type = 12 ? 0 : 100 *(Pe * Vt + Ie * Vt_sum + De * (Vt - Vt_past) / deltaT);
	  throttle = throttle < 0 ? 0 : (throttle > 100 ? 100 : throttle);
	  Vt_past = Vt;
	  Set_Engine_Speed(throttle);

	  // пид регулятор элеронов

	  Zt_sum = Zt_sum + target_accelerateL[2];
	  angle_aileron = 90 * (Pr * target_accelerateL[2] + Ir * Zt_sum + Dr * (target_accelerateL[2] - Zt_past) / deltaT);
	  angle_aileron = angle_aileron < 0 ? 0 : (angle_aileron > 180 ? 180 : angle_aileron);
	  Zt_past = target_accelerateL[2];
	  Set_Servo_RightWing(90 - angle_aileron);
	  Set_Servo_LeftWing(90 + angle_aileron);

	  // пид регулятор руля высоты

	  double Yt = target_accelerateL[1] - accV.y;
	  Yt_sum = Yt_sum + Yt;
	  angle_rudder_up = 90 + 90 * (Pp * Yt + Ip * Yt_sum + Dp * (Yt - Yt_past) / deltaT);
	  angle_rudder_up = angle_rudder_up < 0 ? 0 : (angle_rudder_up > 180 ? 180 : angle_rudder_up);
	  Yt_past = Yt;
	  Set_Servo_Elevator(angle_rudder_up);

	  // пид регулятор руля аправления

	  beta_sum = beta_sum + beta;
	  angle_rudder_course = 90 + 90 * (Py * beta + Iy * beta_sum + Dy * (Yt - beta_past) / deltaT);
	  angle_rudder_course = angle_rudder_course < 0 ? 0 : (angle_rudder_course > 180 ? 180 : angle_rudder_course);
	  beta_past = beta;
	  Set_Servo_Elevator(angle_rudder_course);

	  // сброс груза
	  if (P_current.type == 7 && Ltarget < LdropG + 1.5) Set_Servo_CargoHatch(180);
	  else Set_Servo_CargoHatch(0);

	  //***********************

	  HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 23;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 23;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
