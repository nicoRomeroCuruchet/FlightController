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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define sec2milliseconds(x) x*1000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LSB_Sensitivity 16.4

/* Controller parameters */

/************************** Roll **************************/
#define kp_roll  30.0f
#define ki_roll  0.0f
#define kd_roll  0.0f

#define PID_LIM_MIN_INT_ROLL -50.0f
#define PID_LIM_MAX_INT_ROLL +50.0f

#define PID_LIM_MIN_ROLL -200.0f
#define PID_LIM_MAX_ROLL +200.0f
/**********************************************************/

/************************** Pitch **************************/
#define kp_pitch kp_roll
#define ki_pitch ki_roll
#define kd_pitch kd_roll

#define PID_LIM_MIN_INT_PITCH PID_LIM_MIN_INT_ROLL
#define PID_LIM_MAX_INT_PITCH PID_LIM_MAX_INT_ROLL

#define PID_LIM_MIN_PITCH PID_LIM_MIN_ROLL
#define PID_LIM_MAX_PITCH PID_LIM_MAX_ROLL
/**********************************************************/

/************************** Yaw **************************/
#define kp_yaw  1.0f
#define ki_yaw  0.0f
#define kd_yaw  0.0f

#define PID_LIM_MIN_INT_YAW -100.0f
#define PID_LIM_MAX_INT_YAW +100.0f

#define PID_LIM_MIN_YAW -400.0f
#define PID_LIM_MAX_YAW +400.0f
/**********************************************************/

#define SAMPLE_TIME_S 0.005f // 200 Hz
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DUTY_CYCLE_MOTOR_MIN 1000
#define DUTY_CYCLE_MOTOR_MAX 2000
#define DUTY_CYCLE_SERVO_CENTER 1500
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
extern float q0, q1, q2, q3;
extern float pitch, roll, yaw;
extern short gyro[3];
float gx, gy, gz;
float gx_angle, gy_angle, gz_angle;
uint32_t loop_timer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /*Initialize PWM motors and servos */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  /* Motor CW    */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  /* Motor CCW   */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  /* Servo Roll  */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  /* Servo Pitch */
 /*turn off motors and center servos */
  htim3.Instance->CCR1 = (uint32_t)DUTY_CYCLE_MOTOR_MIN; 	 /*  Channel 1 motor CW    */
  htim3.Instance->CCR2 = (uint32_t)DUTY_CYCLE_MOTOR_MIN;	 /*  Channel 2 motor CCW   */
  htim3.Instance->CCR3 = (uint32_t)DUTY_CYCLE_SERVO_CENTER;	 /*  Channel 1 motor CW    */
  htim3.Instance->CCR4 = (uint32_t)DUTY_CYCLE_SERVO_CENTER;	 /*  Channel 2 motor CCW   */
  /* DMP MPU initialize */
  DMP_Init();
  // Wait 10s for calibration, don't move the IMU!
  Calubration_DMP();
  float gx_offset, gy_offset, gz_offset;
  DMP_get_gyro_offsets(&gx_offset, &gy_offset, &gz_offset);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  PIDController pid_roll, pid_pitch, pid_yaw;
  /* PID ROLL */
  initializePID(&pid_roll,
		  	  	  kp_roll,
				  ki_roll,
				  kd_roll,
				  SAMPLE_TIME_S,
				  PID_LIM_MIN_INT_ROLL,
				  PID_LIM_MAX_INT_ROLL,
				  PID_LIM_MIN_ROLL,
				  PID_LIM_MAX_ROLL);
  /* PID PITCH */
  initializePID(&pid_pitch,
  		  	  	  kp_pitch,
  				  ki_pitch,
  				  kd_pitch,
  				  SAMPLE_TIME_S,
  				  PID_LIM_MIN_INT_PITCH,
  				  PID_LIM_MAX_INT_PITCH,
  				  PID_LIM_MIN_PITCH,
  				  PID_LIM_MAX_PITCH);
  /* PID YAW */
  initializePID(&pid_yaw,
				  kp_yaw,
				  ki_yaw,
				  kd_yaw,
				  SAMPLE_TIME_S,
				  PID_LIM_MIN_INT_YAW,
				  PID_LIM_MAX_INT_YAW,
				  PID_LIM_MIN_YAW,
				  PID_LIM_MAX_YAW);

  float setpoint_roll   =0;
  float setpoint_pitch  =0;
  float setpoint_yaw    =0;

  float pid_roll_output  =0;
  float pid_pitch_output =0;
  float pid_yaw_output   =0;

  float rate_roll, rate_pitch, rate_yaw;
  float thrust_radio=1000;

  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	loop_timer = HAL_GetTick();
	Read_DMP();
	// get angle rates degrees / seconds
	rate_roll  = (((float)gyro[0]) - gx_offset) / LSB_Sensitivity;
	rate_pitch = (((float)gyro[1]) - gy_offset) / LSB_Sensitivity;
	rate_yaw   = (((float)gyro[2]) - gz_offset) / LSB_Sensitivity;

	pid_roll_output  = updatePID(&pid_roll, setpoint_roll, roll, rate_roll);
	pid_pitch_output = updatePID(&pid_pitch, setpoint_pitch, pitch, rate_pitch);
	pid_yaw_output   = updatePID(&pid_yaw, setpoint_yaw, roll, rate_yaw);
	/* set PWM Duty cycle */
	htim3.Instance->CCR1 = (uint32_t)CLIP(thrust_radio + pid_yaw_output, 1200, 2000);	/* Channel 1 motor CW    */
	htim3.Instance->CCR2 = (uint32_t)CLIP(thrust_radio - pid_yaw_output, 1200, 2000);	/* Channel 2 motor CCW   */
	htim3.Instance->CCR3 = (uint32_t)CLIP(1500.0  + pid_roll_output, 500, 2000);	  	/* Channel 3 serve roll  */
	htim3.Instance->CCR4 = (uint32_t)CLIP(1500.0  + pid_pitch_output,500, 2000);    	/* Channel 4 serve pitch */

	//integrate the pitch, roll, yaw angle every 5 milliseconds.
	while (HAL_GetTick() - loop_timer < sec2milliseconds(SAMPLE_TIME_S));
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3999 ;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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