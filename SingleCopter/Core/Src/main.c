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

#define LSB_Sensitivity 131.0f // 65.5f	// Gyro degrees conversions
#define ALPHA 1.0  				// Gyro low pass filter (1.0 no filter, raw data)
#define GYRO_SING -1.0
#define SAMPLE_TIME_S 0.001f 	// 1Khz!
/* Controller parameters */
/***********************************************************/
/************************** ROLL **************************/
/***********************************************************/
#define kp_roll  1.0f
#define ki_roll  0.0f
#define kd_roll  0.0f

#define PID_LIM_MIN_INT_ROLL -50.0f
#define PID_LIM_MAX_INT_ROLL +50.0f

#define PID_LIM_MIN_ROLL -400.0f
#define PID_LIM_MAX_ROLL +400.0f
/***********************************************************/
/************************** PITCH **************************/
/***********************************************************/
#define kp_pitch kp_roll
#define ki_pitch ki_roll
#define kd_pitch kd_roll

#define PID_LIM_MIN_INT_PITCH PID_LIM_MIN_INT_ROLL
#define PID_LIM_MAX_INT_PITCH PID_LIM_MAX_INT_ROLL

#define PID_LIM_MIN_PITCH PID_LIM_MIN_ROLL
#define PID_LIM_MAX_PITCH PID_LIM_MAX_ROLL
/***********************************************************/
/************************** YAW ****************************/
/***********************************************************/
#define kp_yaw  1.0f
#define ki_yaw  0.0f
#define kd_yaw  0.0f

#define PID_LIM_MIN_INT_YAW -100.0f
#define PID_LIM_MAX_INT_YAW +100.0f

#define PID_LIM_MIN_YAW -400.0f
#define PID_LIM_MAX_YAW +400.0f
/**********************************************************/
/**********************************************************/
#define MOTOR_TURN_OFF 1000
#define MOTOR_MAX_SPEED 2000
#define MOTOR_MIN_SPEED 1100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RADIO_MIDDLE_FREQ 1500.0f
#define RADIO_DEAD_BAND 20.0f

#define MAP(input, in_min, in_max, out_min, out_max) \
    (((input) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

#define M_PI 3.14159265358979323846
#define deg2rad(degrees) degrees * M_PI / 180.0
#define rad2deg(radians) radians * 180.0 / M_PI

#define TRANSMITED_BYTES 9*4

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t rx_buffer[TRANSMITED_BYTES];
float received_values[9]; // To store the result
int pid_update_ready=0;

float temperature, pressure, humidity, altitude;

extern float q0, q1, q2, q3;
extern float pitch, roll, yaw;
extern short gyro[3];
extern float gravity[3];
float gx, gy, gz;
float gx_angle, gy_angle, gz_angle;
float gyro_roll, gyro_pitch, gyro_yaw;
uint32_t loop_timer;
float yaw_offset, roll_offset, pitch_offset;
float yaw_w_offset, roll_w_offset, pitch_w_offset;
float gx_offset, gy_offset, gz_offset;


int16_t gyro_axis[3];
float gx_2, gy_2, gz_2;
float gx_2_offset, gy_2_offset, gz_2_offset;

/* Measure Width */
extern volatile uint32_t pulse_ch1;
extern volatile uint32_t pulse_ch2;
extern volatile uint32_t pulse_ch3;
extern volatile uint32_t pulse_ch4;


float setpoint_roll   = 0;
float setpoint_pitch  = 0;
float setpoint_yaw    = 0;
uint32_t measure_time = 0;

float pid_roll_output  = 0;
float pid_pitch_output = 0;
float pid_yaw_output   = 0;

float rate_roll = 0;
float rate_pitch = 0;
float rate_yaw = 0;

float throttle_radio = 0;


BMP280_HandleTypedef bmp280;
QMC_t qmc;

float pressure, temperature, humidity;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /*do{
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	  bmp280_init_default_params(&bmp280.params);
	  bmp280.addr = BMP280_I2C_ADDRESS_0;
	  bmp280.i2c = &hi2c2;
  }
  while((QMC_init(&qmc, &hi2c2 ,200)==-1) || !bmp280_init(&bmp280, &bmp280.params)) ;



  while(1){

	  loop_timer = HAL_GetTick();
	  bmp280_read_float(&bmp280, &temperature,&pressure, &humidity);
	  QMC_read(&qmc);
	  measure_time = HAL_GetTick() - loop_timer;
  }*/

  /*Initialize turn off motors */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  			/* Motor 1 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  			/* Motor 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); 		 	/* Motor 3 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  			/* Motor 4 */
  /*turn off motors and center servos */
  htim3.Instance->CCR1 = (uint32_t)MOTOR_TURN_OFF;      /*  Channel 1 Motor 1 */
  htim3.Instance->CCR2 = (uint32_t)MOTOR_TURN_OFF;      /*  Channel 2 Motor 2 */
  htim3.Instance->CCR3 = (uint32_t)MOTOR_TURN_OFF;      /*  Channel 3 Motor 3 */
  htim3.Instance->CCR4 = (uint32_t)MOTOR_TURN_OFF;      /*  Channel 4 Motor 4 */
  // Turn off IMU
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  HAL_UART_Receive_IT(&huart2, rx_buffer, TRANSMITED_BYTES);

  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK) // Throttle ---> radio ch1
  {
	 Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2) != HAL_OK) //Yaw ---> radio ch2
  {
     Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3) != HAL_OK) // Pitch --> radio ch3
  {
  	 Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK) // Roll ---> radio ch4
  {
  	 Error_Handler();
  }

  HAL_Delay(250);

  while(pulse_ch1 < 950 || pulse_ch2 < 950 || pulse_ch3 < 950 || pulse_ch4 < 950)
  {
	  //__HAL_TIM_SET_COUNTER(&htim1, 0);  // reset the counter
	  pulse_ch1=pulse_ch2=pulse_ch3=pulse_ch4=0;
	  HAL_Delay(250);
  }

  while(pulse_ch1>2000 || pulse_ch2>2000 || pulse_ch3>2000 || pulse_ch4>2000)
  {
	  __HAL_TIM_SET_COUNTER(&htim1, 0);  // reset the counter
	  pulse_ch1=pulse_ch2=pulse_ch3=pulse_ch4=0;
	  HAL_Delay(250);
  }

  /* DMP MPU initialize */
  HAL_Delay(150);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_Delay(150);

  do{
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  HAL_Delay(150);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	  HAL_Delay(150);

  } while(MPU6050_getDeviceID() != 0x68);

  DMP_Init();
  // Wait 10s for calibration, don't move the IMU!
  HAL_Delay(10000);
  Calubration_DMP();
  HAL_Delay(5000);
  DMP_get_gyro_offsets(&gx_offset, &gy_offset, &gz_offset);
  // Now set the gyro
  HAL_Delay(100);
  do{
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	  HAL_Delay(150);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
      HAL_Delay(150);
  }while(init_gyro_registers() == -1);

  HAL_Delay(1000);
  gx_2_offset=gy_2_offset=gz_2_offset=0;
  for (int var = 0; var < 2000; ++var) {
	  read_gyro_only(gyro_axis);
	  gx_2_offset += (float)gyro_axis[0];
	  gy_2_offset += (float)gyro_axis[1];
	  gz_2_offset += (float)gyro_axis[2];
  }
  gx_2_offset /=2000.0f;
  gy_2_offset /=2000.0f;
  gz_2_offset /=2000.0f;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  do {

	Read_DMP();
	yaw_offset   = yaw;
	roll_offset  = roll;
	pitch_offset = pitch;
	HAL_Delay(100);

  } while (pulse_ch2 > 1100);// throttle upper 1050 don't start

  PIDController pid_roll, pid_pitch, pid_yaw;
  initializePID(&pid_roll, 1.0, 0.1, 0.0, SAMPLE_TIME_S,
                PID_LIM_MIN_INT_ROLL, PID_LIM_MAX_INT_ROLL,
                PID_LIM_MIN_ROLL, PID_LIM_MAX_ROLL);

  initializePID(&pid_pitch, 1.0, 0.5, 0.15, SAMPLE_TIME_S,
                PID_LIM_MIN_INT_PITCH, PID_LIM_MAX_INT_PITCH,
                PID_LIM_MIN_PITCH, PID_LIM_MAX_PITCH);

  initializePID(&pid_yaw, 0.0, 1.0, 0.0, SAMPLE_TIME_S,
                PID_LIM_MIN_INT_YAW, PID_LIM_MAX_INT_YAW,
                PID_LIM_MIN_YAW, PID_LIM_MAX_YAW);

  gyro_roll=gyro_pitch=gyro_yaw=0.0f;
  rate_roll=rate_pitch=rate_yaw=0.0f;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	loop_timer = HAL_GetTick();
	Read_DMP();
	yaw_w_offset   = yaw - yaw_offset;  // TODO too much drift...
	roll_w_offset  = roll - roll_offset;
	pitch_w_offset = pitch - pitch_offset;

	read_gyro_only(gyro_axis);
	gx_2 = ((float)gyro_axis[0] - gx_2_offset)  / LSB_Sensitivity;
	gy_2 = ((float)gyro_axis[1] - gy_2_offset)  / LSB_Sensitivity;
	gz_2 = ((float)gyro_axis[2] - gz_2_offset)  / LSB_Sensitivity;

	gyro_roll += gx_2*SAMPLE_TIME_S;
	gyro_pitch += gy_2*SAMPLE_TIME_S;
	
	gyro_roll  += gyro_pitch * sin(deg2rad(gz_2) * SAMPLE_TIME_S);
	gyro_pitch -= gyro_roll * sin(deg2rad(gz_2) * SAMPLE_TIME_S);

	gyro_pitch = gyro_pitch*0.996 + pitch_w_offset*0.004;
	gyro_roll = gyro_roll*0.996 + roll_w_offset*0.004;
  
	// get angle rates degrees / seconds with an exponential filter
	/*rate_roll  = GYRO_SING*((1-ALPHA)*rate_roll  + ALPHA*((((float)gyro[0]) - gx_offset) / LSB_Sensitivity));
	rate_pitch = GYRO_SING*((1-ALPHA)*rate_pitch + ALPHA*((((float)gyro[1]) - gy_offset) / LSB_Sensitivity));
	rate_yaw   = GYRO_SING*((1-ALPHA)*rate_yaw   + ALPHA*((((float)gyro[2]) - gz_offset) / LSB_Sensitivity));

	//TODO only for debug:
	gyro_roll  +=rate_roll*0.003;//SAMPLE_TIME_S;
	gyro_roll = gyro_roll*0.996 + roll_w_offset*0.004;
	gyro_pitch +=rate_pitch*SAMPLE_TIME_S;
	gyro_pitch = gyro_pitch*0.996 + pitch_w_offset*0.004;*/

	//radio set-points
	throttle_radio = pulse_ch2;
	// yaw
	setpoint_yaw   = RADIO_MIDDLE_FREQ - (float)pulse_ch1;
	if (abs(setpoint_yaw)< RADIO_DEAD_BAND){
		setpoint_yaw=0.0;
	}else{
		float s =  (setpoint_yaw > 0) ? 1.0 : -1.0;
		setpoint_yaw = MAP(setpoint_yaw - s*RADIO_DEAD_BAND, -500.0, +500.0,-20.0,20.0);
	}
	// roll
	setpoint_roll  = RADIO_MIDDLE_FREQ - (float)pulse_ch4;
	if (abs(setpoint_roll)< RADIO_DEAD_BAND){
		setpoint_roll=0.0;
	} else{
		float s =  (setpoint_roll > 0) ? 1.0 : -1.0;
		setpoint_roll = MAP(setpoint_roll - s*RADIO_DEAD_BAND, -500.0, +500.0,-20.0,20.0);
	}
	// pitch
	setpoint_pitch = RADIO_MIDDLE_FREQ - (float)pulse_ch3;
	if (abs(setpoint_pitch)< RADIO_DEAD_BAND){
		setpoint_pitch=0.0;
	} else{
		float s =  (setpoint_pitch > 0) ? 1.0 : -1.0;
		setpoint_pitch = MAP(setpoint_pitch - s*RADIO_DEAD_BAND, -500.0, +500.0,+20.0,-20.0);
	}
	// PID controller
	pid_roll_output  = updatePID(&pid_roll, setpoint_roll, gyro_roll, gx_2);
	pid_pitch_output = updatePID(&pid_pitch, setpoint_pitch, gyro_pitch, gy_2);
	pid_yaw_output   = updatePID(&pid_yaw, setpoint_yaw, yaw_w_offset, gz_2);
	//measure_time = HAL_GetTick() - loop_timer;
	while (HAL_GetTick() - loop_timer < sec2milliseconds(SAMPLE_TIME_S));
	measure_time = HAL_GetTick() - loop_timer;
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
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICFilter = 1;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  sConfigOC.Pulse = 1000;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
