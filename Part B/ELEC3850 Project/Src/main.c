/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dwt_delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LSM9DS1_Acc_Read_address	0xD7
#define LSM9DS1_Acc_write_address	0xD6

#define LSM9DS1_Mag_Read_address	0x3D
#define LSM9DS1_Mag_write_address	0x3C

//Silkscreen on PCB list these values I don't know if they work
//#define LSM9DS1_Acc_address	0x68
//#define LSM9DS1_Mag_address	0x1E

// Define Control registers and Values to send
#define CTRL_REG1_G	0x10		// Angular rate sensor Control Register 1.
#define CTRL_REG1_G_VALUE	0xAB

#define CTRL_REG3_G	0x12		// Angular rate sensor Control Register 3.
#define CTRL_REG3_G_VALUE	0x40

#define CTRL_REG6_XL	0x20	//Linear acceleration sensor Control Register 6
#define CTRL_REG6_XL_VALUE	0xB0

#define CTRL_REG7_XL	0x21	// Linear acceleration sensor Control Register 7.
#define CTRL_REG7_XL_VALUE	0xE0//0xC0//0xC4

#define CTRL_REG1_M	0x20	// Magnetometer register
#define CTRL_REG1_M_VALUE	0xD0//0x50

#define	CTRL_REG2_M	0x21	//Magnetometer register
#define CTRL_REG2_M_VALUE	0x00

#define	CTRL_REG3_M	0x22	//Magnetometer register
#define	CTRL_REG3_M_Value	0x0

// Data Registers

// Temperature
#define OUT_TEMP_L	0x15
#define	OUT_TEMP_H	0x16

// Linear acceleration sensor output register
#define OUT_X_L_XL	0x28
#define OUT_X_H_XL	0x29
#define OUT_Y_L_XL	0x2A
#define OUT_Y_H_XL	0x2B
#define OUT_Z_L_XL	0x2C
#define OUT_Z_H_XL	0x2D

// Gyroscope (Angular rate)
#define OUT_X_L_G	0x18
#define OUT_X_H_G	0x19
#define OUT_Y_L_G	0x1A
#define OUT_Y_H_G	0x1B
#define OUT_Z_L_G	0x1C
#define OUT_Z_H_G	0x1D

//Magnetometer
#define OUT_X_L_M	0x28
#define OUT_X_H_M	0x29
#define OUT_Y_L_M	0x2A
#define OUT_Y_H_M	0x2B
#define OUT_Z_L_M	0x2C
#define OUT_Z_H_M	0x2D

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
uint32_t data[BUFF*NOM_ADC];
uint32_t count, start;
double current = 0;
double  speed;
double iu = 0;
double iu2 = 0;
double Iset = 0;	//Output of outer loop / Input to inner loop
//Variables for speed calcs in GPIO callback
uint32_t tnew = 0;
uint32_t told = 0;
uint32_t tdiff = 0;
uint32_t vtest = 0;
int FLAG1;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void stdio_setup(void);

/* MOTOR PWM */
void pwm_setvalue(int a, uint32_t pwm);

/* PI CONTTROL */
uint32_t pi_control (double u, double ref, uint32_t pwm);
double vel_control (double u, double ref, uint32_t pwm);
double dis_control(double u, double ref, double u2);

/* STEERING */
void steer_pwm_setvalue(uint16_t value);
void steer_straight(void);
void steer_right(void);
void steer_left(void);

/* ULTRA */
double ultrasonic(void);
double ultrasonic2(void);

/*ACCEL*/

void initalise_LSM9DS1(void);	// This function configures and sets the registers inside the IMU

void acceleration_get_raw(void);
void acceleration_convert_values(void);

void angle_get_raw(void);
void angle_convert_values(void);

void mag_get_raw(void);
double mag_convert_values();

void read_temperature(void);

uint8_t i2cBuf[8];		// Buffer for sending and receiving Data over I2C
uint8_t	AccBuf[8];		// Buffer to store Accelerometer values
uint8_t GyroBuff[8];	// Buffer to store gyroscope (angle values)
uint8_t MagBuff[8];		// Buffer to store Magnetometer values
uint8_t Deg_Buff[8];		// Buffer to store Magnetometer values

int16_t temp;
float deg;

int16_t ax,ay,az;			// Combined Raw Accelerometer values
float Xaccel,Yaccel,Zaccel;	// Scaled Raw Accelerometer values

int16_t	gx, gy, gz;
float Xangle, Yangle, Zangle;

int16_t	mx, my, mz;
double Xmag, Ymag, Zmag;

double direction;	// This is the direction used for the compass

double angle;

char update_angle;	// Flag to indicate time to update the angle measurement
// Final angle values
float Xangledeg;
float Yangledeg;
float Zangledeg;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	stdio_setup();
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  DWT_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  initalise_LSM9DS1();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)data, BUFF*NOM_ADC);
  FLAG1 = 0;
  	start = HAL_GetTick();
  	count = 0;
	double Vset;
  	Vset = VEL;
  	uint32_t pwm = 0;
  	uint32_t adaptive = Loop2;		//Counter to time distance loop
  	double dist = ultrasonic();
  	uint16_t timcount = LoopFreq;	//Counter to time inner/outer loops
  	steer_straight();
  	double u3 = VEL;		//Input for Adaptive Cruise Control (cm), should maintain 1s gap at Cruise Control speed
  	double buf1[500];
  	int loop=0;
	char c;
	uint8_t inp[30];
	double direc;
	double cardist = 0;
	double heading;
	double turn;
	double dist2;
	int k = 0;
	int d = 1;

	//steer_right();
	//steer_straight();
	HAL_Delay(100);
	printf("\nSTART\n");

	//pwm_setvalue(1, 25);
	steer_straight();
	while (1)
	{
	  c = getchar();

		inp[loop] = c;

		if(c == '\0')
		{
			loop = -1;
		}


		if(c == '\r')
		{
			inp[loop] = '\0';
			printf("%d: inp = %s\r\n", loop, inp);
			loop = -1;

			break;
		}
	}

		loop++;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	while(0)
	{
		acceleration_get_raw();
		acceleration_convert_values();
		HAL_Delay(500);
	}

	/*Reverse Parallel Park*/

	while(1)
	{
	for (int z = 0; z < 5; z++)
	{
		mag_get_raw();
		direc +=mag_convert_values();
		HAL_Delay(200);
	}
	direc = direc/5;
	printf("Direction: %f\n", direc);

	for (int z = 0; z < 5; z++)
	{
		cardist += ultrasonic2();
		HAL_Delay(10);
	}
	cardist = cardist/5;
	printf("Distance from car: %f\n", cardist);
	HAL_Delay(500);
	pwm_setvalue(2, 15);
	while(1)
	{

		dist2 = ultrasonic2();
		//printf("Distance: %f\n", dist2);
		while (dist2 >= cardist + 10)
		{
			if(!k)
			{
				steer_right();
				k = 1;
			}

			mag_get_raw();
			heading = mag_convert_values();
			//printf("Heading: %f\n", heading);
			while(heading <= direc - 1.5)
			{
				k = 0;
				if(!k)
				{
					steer_left();
					k = 1;
				}
				while(1)
				{
					mag_get_raw();
					heading = mag_convert_values();
					printf("Heading: %f\n", heading);
					while (heading >= direc + 5)
					{
						k = 0;
						pwm_setvalue(0, 15);

						steer_straight();
						k = 1;
						HAL_Delay(500);

						while(1)
						{
							dist2 = 0;
							pwm_setvalue(1, 15);
							dist2 = ultrasonic();
							printf("Distance: %f\n", dist2);
							while (dist2 <= 40)
							{
								pwm_setvalue(0, 15);
							}
						}
					}
				}
			}
		}
	}
}

	/*Curise control & adaptive cruise control*/
	while(0)
	{

		while(1)
		{
			if(FLAG1)
			{
				//Distance loop occurs every Loop2x of the inner loop
				if (adaptive == Loop2){
				//Adaptive Cruise Control Loop
				dist = ultrasonic();
				Vset = dis_control(u3, dist, VEL);
				adaptive = 0;
				//printf("Dist = %f\n", dist);
				//printf("Vset = %f\n", Vset);
				//printf("Dist = %f\n", dist);
				}

				//Ensures inner loop runs LoopFreqx faster than outer loop (should run every LoopFreq ms)
				if (timcount == LoopFreq){

					if(tdiff != 0){
						speed = (double)(((6*M_PI/5 * 1/ tdiff))*1000);						//(count/LoopFreq)
					}
					else{
						speed = 0;
						}

					//buf1[loop] = speed;
					//Velocity Control Loop
					Iset = vel_control(Vset, speed, pwm);
					//Update Variables
					timcount = 0;
					count = 0;
					loop++;
				}
				//Get current measurement
				current = (double)current*3300/4096*1/140;
				//Current Control Loop
				pwm = pi_control(Iset, current, pwm);
				//Clamp output
				if(pwm > MAX_PWM)
				{
				  pwm = MAX_PWM;
				}

				else if(pwm < 0)
				{
				  pwm = 0;
				}
				//Set pwm value according to control output
				pwm_setvalue(d, pwm);
				//Update variables
				FLAG1 = 0;
				timcount++;
				adaptive++;
			}
		}
	}

	/*Line following*/
	while(0)
	{
		 uint8_t a_right = HAL_GPIO_ReadPin(A0_right_GPIO_Port, A0_right_Pin);
		 uint8_t a_left = HAL_GPIO_ReadPin(A0_left_GPIO_Port, A0_left_Pin);

			  int p = 0;
			  if(p == 0)
			  {
			  if(a_right == 1 && a_left == 0)
			  {
				  pwm_setvalue(1, 15);
				  steer_left();
				  printf("Left\r\n");
				  //sprintf(Msg, "Turning Left\r\n");
				 // CDC_Transmit_FS(Msg, strlen(Msg));
			  }
			  else if(a_left == 1 && a_right == 0)
			  {
				  pwm_setvalue(1, 15);
				  steer_right();
				  printf("Right\r\n");
				  //sprintf(Msg, "Turning Right\r\n");
				 //CDC_Transmit_FS(Msg, strlen(Msg));
			  }
			  else if(a_right == 1 && a_left == 1)
			  {
				  pwm_setvalue(0, 30);
				  printf("Stop\r\n");
				  p = 1;
				  //sprintf(Msg, "Stopping\r\n");
				  //CDC_Transmit_FS(Msg, strlen(Msg));
			  }
			  else
			  {
				  pwm_setvalue(1, 15);
				  steer_straight();
				  printf("Straight\r\n");
				  //sprintf(Msg, "Turning Straight\r\n");
				  //CDC_Transmit_FS(Msg, strlen(Msg));
			  }
			  HAL_Delay(100);
			  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 540;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 234;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 840;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 168-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 84;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, INA_Pin|INB_Pin|TRIG2_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIGL_GPIO_Port, TRIGL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin TRIGL_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|TRIGL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INA_Pin INB_Pin TRIG2_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = INA_Pin|INB_Pin|TRIG2_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_left_Pin ECHO2_Pin OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = A0_left_Pin|ECHO2_Pin|OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_right_Pin ECHOL_Pin */
  GPIO_InitStruct.Pin = A0_right_Pin|ECHOL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void initalise_LSM9DS1(void)
{

	//Write to the accelerometer
	// Linear acceleration sensor Control Register 6
	/*
	i2cBuf[0] = 0x20;	// CTRL_REG6_XL (20h)
	i2cBuf[1] = 0x10;// 00010000 - (0x10) 4g           00011000 - (0x18) this is Accelerometer full-scale selection 8g
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Acc_write_address, i2cBuf, 2, 10);

	// Read from the same register to see if it worked
	i2cBuf[0] = 0x20;	// CTRL_REG6_XL (20h)
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Acc_Read_address, i2cBuf, 1, 10);

	i2cBuf[1] = 0x00;	// Clear value before reading value
	HAL_I2C_Master_Receive(&hi2c1, LSM9DS1_Acc_Read_address, &i2cBuf[1], 1, 10);
	printf("value = %x\n", i2cBuf[1]);
	fflush(stdout);

	// Turn on the gyroscope
	i2cBuf[0] = 0x10;	// CTRL_REG1_G (10h)
	i2cBuf[1] = 0xA0;	// 10100000
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Acc_write_address, i2cBuf, 2, 10);

	i2cBuf[0] = 0x10;	// CTRL_REG1_G (10h)
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Acc_Read_address, i2cBuf, 1, 10);

	i2cBuf[1] = 0x00;	// Clear value before reading value
	HAL_I2C_Master_Receive(&hi2c1, LSM9DS1_Acc_Read_address, &i2cBuf[1], 1, 10);
	printf("value = %x\n", i2cBuf[1]);
	fflush(stdout);


	*/

	// Setup Angular rate sensor Control Register 1
	i2cBuf[0] = CTRL_REG1_G;
	i2cBuf[1] = CTRL_REG1_G_VALUE;
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Acc_write_address, i2cBuf, 2, 10);

	// Setup Angular rate sensor Control Register 3
	i2cBuf[0] = CTRL_REG3_G;
	i2cBuf[1] = CTRL_REG3_G_VALUE;
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Acc_write_address, i2cBuf, 2, 10);

	// Setup Linear acceleration sensor Control Register 6
	i2cBuf[0] = CTRL_REG6_XL;
	i2cBuf[1] = CTRL_REG6_XL_VALUE;
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Acc_write_address, i2cBuf, 2, 10);

	// Setup Linear acceleration sensor Control Register 7
	i2cBuf[0] = CTRL_REG7_XL;
	i2cBuf[1] = CTRL_REG7_XL_VALUE;
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Acc_write_address, i2cBuf, 2, 10);

	// Setup Magnetometer register 1
	i2cBuf[0] = CTRL_REG1_M;
	i2cBuf[1] = CTRL_REG1_M_VALUE;
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Mag_write_address, i2cBuf, 2, 10);

	// Setup Magnetometer register 2
	i2cBuf[0] = CTRL_REG2_M;
	i2cBuf[1] = CTRL_REG2_M_VALUE;
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Mag_write_address, i2cBuf, 2, 10);

	// Setup Magnetometer register 3
	i2cBuf[0] = CTRL_REG3_M;
	i2cBuf[1] = CTRL_REG3_M_Value;
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Mag_write_address, i2cBuf, 2, 10);

	// lower y offset register
	i2cBuf[0] = 0x07;
	i2cBuf[1] = 0x00;	//S ?
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Mag_write_address, i2cBuf, 2, 10);

	// High y offset register
	i2cBuf[0] = 0x08;
	i2cBuf[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Mag_write_address, i2cBuf, 2, 10);

	// Low X offset register
	i2cBuf[0] = 0x05;
	i2cBuf[1] = 0x00;	//E
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Mag_write_address, i2cBuf, 2, 10);

	// High X offset register
	i2cBuf[0] = 0x06;
	i2cBuf[1] = 0x03;
	HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_Mag_write_address, i2cBuf, 2, 10);



	printf("Register setup Complete\n");
	fflush(stdout);

}
/**************************************
 * Function to get raw magnetic data
 **************************************/
void mag_get_raw(void)
{
	// Send address to read
   i2cBuf[0] = OUT_X_L_M;
   HAL_I2C_Master_Transmit(&hi2c1,LSM9DS1_Mag_Read_address, i2cBuf, 1,10);
   HAL_I2C_Master_Receive(&hi2c1, LSM9DS1_Mag_Read_address, &MagBuff[0], 6, 10);
}


/**********************************************************
 * Function to convert magnetic values to compass bearings
 ***********************************************************/
double mag_convert_values()
{
/*
	int i;
	for (i = 0; i < 6; i++)
	{
		printf("MagBuff[%d] = %d\n", i, MagBuff[i]);

	}
*/
	//Bit shift to combine upper and lower byte

	mx = (MagBuff[1] << 8 | MagBuff[0]);
	my = (MagBuff[3] << 8 | MagBuff[2]);
	mz = (MagBuff[5] << 8 | MagBuff[4]);

//	printf("mx = %d my = %d mz = %d\n", (int) mx, (int) my, (int) mz);

	//Convert to unit of degrees

	Xmag = (float)mx*0.14;
	Ymag = (float)my*0.14;
	Zmag = (float)mz*0.14;

//	printf("Xmag  = %0.2lfmg Ymag = %0.2lfmg Zmag = %0.2lfmg\n", Xmag , Ymag, Zmag);
//	fflush(stdout);

	angle = atan2f(mx,my)*(180/3.14);
	//printf("angle = %0.2f\n", angle);

	direction = fmod((450-angle),360);

	//printf("Direction = %0.2f\n", direction);

	return(direction);

}


/*******************************************
 * Function to get the raw angle data
 ********************************************/
void angle_get_raw(void)
{
	// Send address to read
   i2cBuf[0] = OUT_X_L_G;
   HAL_I2C_Master_Transmit(&hi2c1,LSM9DS1_Acc_Read_address, i2cBuf, 1,10);


   HAL_I2C_Master_Receive(&hi2c1, LSM9DS1_Acc_Read_address, &GyroBuff[0], 6, 10);
}

/******************************************
 * Function to convert the raw angle
 *******************************************/
void angle_convert_values(void)
{
	//int i;
	/*
	for (i = 0; i < 6; i++)
	{
		printf("GyroBuff[%d] = %d\n", i, GyroBuff[i]);

	}
	*/

	//Bit shift to combine upper and lower byte

	gx = (GyroBuff[1] << 8 | GyroBuff[0]);
	gy = (GyroBuff[3] << 8 | GyroBuff[2]);
	gz = (GyroBuff[5] << 8 | GyroBuff[4]);


	printf("ax = %0.2f ay = %0.2f az = %0.2f\n", (float) ax, (float) ay, (float) az);

	//Convert to unit of degrees

	Xangle = (float)gx*17.5;
	Yangle = (float)gy*17.5;
	Zangle = (float)gz*17.5;

	Xangledeg = (Xangledeg + Xangle) * 0.01;
	Yangledeg = (Yangledeg + Yangle) * 0.01;
	Zangledeg = (Zangledeg + Zangle) * 0.01;

	printf("Xangle = %0.2fmdps Yangle = %0.2fmdps Zangle = %0.2fmdps\n", Xangle, Yangle, Zangle);
	fflush(stdout);

	printf("Xangledeg = %0.2fmdps Yangledeg = %0.2fmdps Zangledeg = %0.2fmdps\n", Xangledeg, Yangledeg, Zangledeg);
	fflush(stdout);

}



/**************************************************
 * Function to read Raw acceleration data
 **************************************************/
void acceleration_get_raw(void)
{
	// Send address to read
   i2cBuf[0] = OUT_X_L_XL;
   HAL_I2C_Master_Transmit(&hi2c1,LSM9DS1_Acc_Read_address, i2cBuf, 1,10);


   HAL_I2C_Master_Receive(&hi2c1, LSM9DS1_Acc_Read_address, &AccBuf[0], 6, 10);

}

/************************************************
 * Function to convert the raw values
 ***********************************************/
void acceleration_convert_values(void)
{
//	int i;
//	for (i = 0; i < 6; i++)
//	{
//		printf("AccBuf[%d] = %d\n", i, AccBuf[i]);
//
//	}

	//Bit shift to combine upper and lower byte

	ax = (AccBuf[1] << 8 | AccBuf[0]);
	ay = (AccBuf[3] << 8 | AccBuf[2]);
	az = (AccBuf[5] << 8 | AccBuf[4]);

	//printf("ax = %0.2f ay = %0.2f az = %0.2f\n", (float) ax, (float) ay, (float) az);

	//Convert to unit of g

	Xaccel = (float)ax*0.122;
	Yaccel = (float)ay*0.122;
	Zaccel = (float)az*0.122;



	printf("Xaccel = %0.2fmg Yaccel = %0.2fmg Zaccel = %0.2fmg\n", Xaccel, Yaccel, Zaccel);
	fflush(stdout);

}

/*********************************************
 * Function to read and display temperature
 **********************************************/
void read_temperature(void)
{
	// Send address to read
   i2cBuf[0] = OUT_TEMP_L;
   HAL_I2C_Master_Transmit(&hi2c1,LSM9DS1_Acc_Read_address, i2cBuf, 1,10);


   HAL_I2C_Master_Receive(&hi2c1, LSM9DS1_Acc_Read_address, &Deg_Buff[0], 2, 10);

   temp = (Deg_Buff[1] << 8 | Deg_Buff[0]);

   deg = ((float)temp/16.0f) + 25.0f;
   printf("Temperature = %.2f\n",deg);


}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	//HAL_ADC_Stop_DMA(&hadc1);

	//speed = (double)count;
	current = (double)data[1];
	FLAG1 = 1;

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	tnew = HAL_GetTick();
	tdiff = tnew - told;
	told = tnew;
	//count++;		//Hall Effect
}

double ultrasonic(void){

	uint32_t localtime = 0;

	double distance = 0;

	HAL_GPIO_WritePin(TRIGL_GPIO_Port, TRIGL_Pin, GPIO_PIN_RESET);  // set the TRIG pin low initially
	DWT_Delay(2);

	localtime = 0;		//Restart local clock

	HAL_GPIO_WritePin(TRIGL_GPIO_Port, TRIGL_Pin, GPIO_PIN_SET);  // set TRIG pin HIGH for 10us
	DWT_Delay(10);

	HAL_GPIO_WritePin(TRIGL_GPIO_Port, TRIGL_Pin, GPIO_PIN_RESET);  // set the TRIG pin low

	//Read ECHO Pin to calculate length of echo pulse

	while (!(HAL_GPIO_ReadPin(ECHOL_GPIO_Port, ECHOL_Pin)));  // wait for the ECHO pin to go high
	while (HAL_GPIO_ReadPin(ECHOL_GPIO_Port, ECHOL_Pin))    // while the pin is high
	{
		localtime++;   // measure time for which the pin is high
		DWT_Delay(1);
	}

	distance = (0.034*(double)localtime) - 2;			//0.034 = Speed of sound in cm/microsecond
													//(-2) accounts for sensor dead zone (mount 2cm from front)
	//Possible Filtering

	return distance;
}

double ultrasonic2(void){

	uint32_t localtime = 0;

	double distance = 0;

	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);  // set the TRIG pin low initially
	DWT_Delay(2);

	localtime = 0;		//Restart local clock

	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_SET);  // set TRIG pin HIGH for 10us
	DWT_Delay(10);

	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);  // set the TRIG pin low

	//Read ECHO Pin to calculate length of echo pulse

	while (!(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin)));  // wait for the ECHO pin to go high
	while (HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin))    // while the pin is high
	{
		localtime++;   // measure time for which the pin is high
		DWT_Delay(1);
	}

	distance = (0.034*(double)localtime) - 2;			//0.034 = Speed of sound in cm/microsecond
													//(-2) accounts for sensor dead zone (mount 2cm from front)
	//Possible Filtering

	return distance;
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)(&ch), 1, 0xFFFF);
  return 1;
}

int __io_getchar(void)
{
  uint8_t ch;
  HAL_UART_Receive(&huart3, (uint8_t *)(&ch), 1, 0xFFFF);
  return (int)ch;
}

void stdio_setup(void)
{
    // Turn off buffers, so I/O occurs immediately
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

void pwm_setvalue(int a, uint32_t pwm)
{
	//Timer configuration setup

	TIM_OC_InitTypeDef sConfigOC;

	//Timer setup
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	//Stop motors: turn on both low gates
	switch(a){

		// Stop - i think this is baaddd

		case 0:

			//set pulse to high_a
			sConfigOC.Pulse = pwm;
			if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
			{
				Error_Handler();
			}
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

			//Enable INA and enable INB- stop
			HAL_GPIO_WritePin(GPIOD, INA_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, INB_Pin, GPIO_PIN_SET);

		break;

		//Go forwards

		case 1:

			//set pulse to high_a
			sConfigOC.Pulse = pwm;
			if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
			{
				Error_Handler();
			}
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

			//Enable INA and disable INB- forwards direction
			HAL_GPIO_WritePin(GPIOD, INA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, INB_Pin, GPIO_PIN_SET);

		break;

		//Go backwards

		case 2:

			//set pulse to high_a
			sConfigOC.Pulse = pwm;
			if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
			{
				Error_Handler();
			}
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

			//Enable INB and disable INA- backwards direction
			HAL_GPIO_WritePin(GPIOD, INA_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, INB_Pin, GPIO_PIN_RESET);

		break;
	}
}

void steer_straight(void)
{
	steer_pwm_setvalue(234);
}

void steer_left()
{
	steer_pwm_setvalue(200);
}

void steer_right()
{
	steer_pwm_setvalue(290);
}



void steer_pwm_setvalue(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

double dis_control(double u, double ref, double u2)
{
	float y;			/* Control output */
	float e;			//error


	/* Compute new integrator and the final control output. */
	e = u - ref; 	//calculate error
	y = kp3 * e;

	if(ref>100)
	{
	  y = u2;
	}

	return y;
}

double vel_control (double u, double ref, uint32_t pwm)
{

	int int_update;      /* Integrator update variable*/
	long new_i;       /*  new integrator value */
	float y;			/* Control output */
	float e;			//error


	/* Compute new integrator and the final control output. */
	e = u - ref; 	//calculate error
	y = kp2 * e;

	if((pwm < MAX_PWM) && (pwm > 0))
	{
	  new_i = iu2 + e;	//calculate integral
	  y = y + (ki2 * new_i); // calculate control output
	}

	iu2 = new_i;

	//Check for saturation

	return y;
}

uint32_t pi_control (double u, double ref, uint32_t pwm)
{
	double y;			/* Control output */
	double e;			//error
	double new_i;       /*  new integrator value */


	/* Compute new integrator and the final control output. */
	e = u - ref; 	//calculate error
	y = (kp * e); // calculate control output

	if((pwm < MAX_PWM) && (pwm > 0))
	{
		new_i = iu + e;	//calculate integral
		y = y + (ki * new_i); // calculate control output
	}

	iu = new_i;

	return (unsigned int) y;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
