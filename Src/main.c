/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "opamp.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hw.h"
#include "bsp.h"
#include "vcom.h"
#include "version.h"
#include "ism330dlc/ism330dlc_reg.h"
#include "ism330dlc/ism330dlc_app.h"
#include "HTS221/HTS221Sensor.h"
#include "HTS221_old/hts221.h"
#include "BERKELEY/BERKELEY.h"
#include "BERKELEY/berkeley_app.h"
#include "analog/analog.h"
#include <stdio.h>
#include <stdarg.h>
#include "../LoRaWAN/Core/lora.h"
#include "../LoRaWAN/Utilities/low_power_manager.h"
#include "../LoRaWAN/Utilities/timeServer.h"
#define PRINTF(...)     vcom_Send(__VA_ARGS__)
#define BUFSIZE 128
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define RELE
#define ACC

/*!
 * Defines the application data transmission duty cycle. value in [ms].
 */
#define START_TX_DUTYCYCLE                               30000
#define DEFAULT_TX_DUTYCYCLE                            600000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_CONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           64
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
//static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
lora_AppData_t AppData={ AppDataBuff,  0 ,0 };

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded ( void );

/* LoRa endNode send request*/
static void Send( void );

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* tx timer callback function*/
static void OnTxTimerEvent( void );

/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks = { HW_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LORA_RxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded};

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppState = RESET;

static TimerEvent_t TxTimer;
static TimerEvent_t AccTimer;
static TimerEvent_t TiltTimer;

/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,
                                    LORAWAN_PUBLIC_NETWORK};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;

uint8_t _start_count = 3;
uint8_t _first_time_flag = 0;
uint32_t app_tx_dutycyle = START_TX_DUTYCYCLE;

extern uint8_t timer_ism330dlc_read_acc_data;
extern uint8_t tim_berkeley_read_tilt_data;

void acc_acq(uint16_t ms, int16_t* acc_mean, int16_t* acc_min, int16_t* acc_max, uint16_t* acc_std);
void tilt_acq(uint16_t ms, int16_t* tilt_mean, int16_t* tilt_min, int16_t* tilt_max, uint16_t* tilt_std);

char data[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t iw;
static char buff[BUFSIZE+16];
void vcom_Send( char *format, ... )
{
  va_list args;
  va_start(args, format);

  /*convert into string at buff[0] of length iw*/
  iw = vsprintf(&buff[0], format, args);

  HAL_UART_Transmit(&huart3,(uint8_t *)&buff[0], iw, 300);

  va_end(args);
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
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_OPAMP1_Init();
  MX_ADC1_Init();
  MX_OPAMP2_Init();
  MX_ADC2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  PRINTF("START\n");

//  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_SET);
////  HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);
////  HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_SET);
////  HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);

//  HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_RESET);
////  HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_SET);
////  HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_RESET);
//
//  HAL_GPIO_WritePin(EN_RELE1_GPIO_Port, EN_RELE1_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(EN_RELE1_GPIO_Port, EN_RELE1_Pin, GPIO_PIN_SET);
//
//  HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE2_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE2_Pin, GPIO_PIN_SET);
//
//  HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_SET);
//
////  HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);
//
//  HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE1_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE1_Pin, GPIO_PIN_SET);

#ifdef ACC
  BERKELEY_Init(208,2000);
#endif
	uint8_t err = 0;

	err = ISM330DLC_Init(ISM330DLC_XL_ODR_12Hz5, ISM330DLC_2g, ISM330DLC_XL_ANA_BW_400Hz, ISM330DLC_XL_LOW_NOISE_LP_ODR_DIV_100);
	if(err!=0)
	{
	  sprintf(data,"Error: ISM330DLC_Init");
	  PRINTF("%s\r\n", data);
	}

  /* Configure the hardware*/
  HW_Init();

  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  HAL_Delay(10000);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );

  PRINTF("VERSION: %X\n\r", VERSION);

  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);

  LORA_Join();

  LoraStartTx( TX_ON_TIMER) ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	LoRaMacProcess( );
	DISABLE_IRQ( );
	/* if an interrupt has occurred after DISABLE_IRQ, it is kept pending
	 * and cortex will not enter low power anyway  */

	#ifndef LOW_POWER_DISABLE
		LPM_EnterLowPower( );
	#endif

	ENABLE_IRQ();


//	uint16_t v = 0;
//	uint8_t res = 0;
//	res = getVIN(&v);
//	if(res == 0)
//	{
//		sprintf(data,"VIN ADC: %d", v);
//		PRINTF("%s\r\n", data);
//		sprintf(data,"VIN I: %f", ((float)v)*0.001611328125);
//		PRINTF("%s\r\n", data);
//	}
//
//// -------------------- S1 S2 ------------------------
////	uint16_t s1 = 0;
////	uint16_t s2 = 0;
////	uint8_t res1 = 0, res2 = 0;
////	HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_SET);
////	HAL_Delay(1000);
////	//---VSTEPUP---//
////	res = getVSTEPUP(&v);
////	if(res == 0)
////	{
////		sprintf(data,"VSTEPUP ADC: %d", v);
////		PRINTF("%s\r\n", data);
////		sprintf(data,"VSTEPUP I: %f", ((float)v)*0.0072509765625);
////		PRINTF("%s\r\n", data);
////	}
////	//---S1---//
////	res1 = get420_1(&s1);
////	//---S2---//
////	res2 = get420_2(&s2);
////	if(res1 == 0)
////	{
////		sprintf(data,"S1 ADC: %d", s1);
////		PRINTF("%s\r\n", data);
////		sprintf(data,"S1 I: %f", (float)((float)s1)*0.000008824359940);
////		PRINTF("%s\r\n", data);
////	}
////	if(res2 == 0)
////	{
////		sprintf(data,"S2 ADC: %d", s2);
////		PRINTF("%s\r\n", data);
////		sprintf(data,"S2 I: %f", (float)((float)s2)*0.000008824359940);
////		PRINTF("%s\r\n", data);
////	}
////	HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);
//
//// -------------------- CON RELE'------------------------
//	uint16_t s = 0;
//	HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_SET);
//	HAL_Delay(500);
//
//	res = getVSTEPUP(&v);
//	if(res == 0)
//	{
//		sprintf(data,"VSTEPUP ADC: %d", v);
//		PRINTF("%s\r\n", data);
//		sprintf(data,"VSTEPUP I: %f", ((float)v)*0.0072509765625);
//		PRINTF("%s\r\n", data);
//	}
//
//
//	HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_RESET);
//	HAL_Delay(100);
//
//	//---S1---//
//
//	res = get420_1(&s);
//	HAL_Delay(100);
//	if(res == 0)
//	{
//		sprintf(data,"S1a ADC: %d", s);
//		PRINTF("%s\r\n", data);
//		sprintf(data,"S1a I: %f", (float)((double)s)*0.000008824359940);
//		PRINTF("%s\r\n", data);
//	}
//	HAL_GPIO_WritePin(EN_RELE1_GPIO_Port, EN_RELE1_Pin, GPIO_PIN_RESET);
//	HAL_Delay(1000);
//	res = get420_1(&s);
//	HAL_Delay(100);
//	if(res == 0)
//	{
//		sprintf(data,"S1b ADC: %d", s);
//		PRINTF("%s\r\n", data);
//		sprintf(data,"S1b I: %f", (float)((double)s)*0.000008824359940);
//		PRINTF("%s\r\n", data);
//	}
//	HAL_GPIO_WritePin(EN_RELE1_GPIO_Port, EN_RELE1_Pin, GPIO_PIN_SET);
//	HAL_Delay(100);
//
//	//---S2---//
//
//	res = get420_2(&s);
//	HAL_Delay(100);
//	if(res == 0)
//	{
//		sprintf(data,"S2a ADC: %d", s);
//		PRINTF("%s\r\n", data);
//		sprintf(data,"S2a I: %f", (float)((double)s)*0.000008824359940);
//		PRINTF("%s\r\n", data);
//	}
//	HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE2_Pin, GPIO_PIN_RESET);
//	HAL_Delay(1000);
//	res = get420_2(&s);
//	HAL_Delay(100);
//	if(res == 0)
//	{
//		sprintf(data,"S2b ADC: %d", s);
//		PRINTF("%s\r\n", data);
//		sprintf(data,"S2b I: %f", (float)((double)s)*0.000008824359940);
//		PRINTF("%s\r\n", data);
//	}
//	HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE2_Pin, GPIO_PIN_SET);
//	HAL_Delay(100);
//
//	HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_SET);
//	HAL_Delay(100);
//	HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);
//
//	uint8_t ReadValue = 0;
//	BERKELEY_ReadReg(BERKELEY_REG_ADDR_WHO_AM_I, &ReadValue, 0x01);
//	sprintf((char*)data, "BERK id: %d", ReadValue );
//	PRINTF("%s\r\n", data);
//	int16_t ptrRawData[2];
//	BERKELEY_AccBitDataRead(ptrRawData);
//	sprintf((char*)data, "ptrRawData[0]: %d\r\nptrRawData[1]: %d\r\n", ptrRawData[0], ptrRawData[1] );
//	PRINTF("%s", data);
//	float ber_x=0, ber_y=0;
//	BERKELEY_GetAcceleration(&ber_x, &ber_y, 2000);
//	sprintf((char*)data, "x: %6.6f\r\ny: %6.6f\r\n", ber_x, ber_y );
//	PRINTF("%s", data);
//
//
////	HTS221Sensor(&hi2c2, HTS221_I2C_ADDRESS);
////	HTS221SensorEnable();
////	uint8_t id = 0;
////	HTS221SensorReadID(&id);
////	sprintf((char*)data, "id: %d", id );
////	PRINTF("%s\r\n", data);
////	HAL_Delay(100);
////	float temp = 0;
////	HTS221SensorGetTemperature(&temp);
////	sprintf((char*)data, "Temp: %6.2f", temp );
////	PRINTF("%s\r\n", data);
////	float hum = 0;
////	HTS221SensorGetHumidity(&hum);
////	sprintf((char*)data, "Hum: %6.2f", hum );
////	PRINTF("%s\r\n", data);
////	HTS221SensorDisable();
//
//	HTS221_Init();
//	uint8_t id = 0;
//	HTS221_ReadID(&id);
//	sprintf((char*)data, "id: %d", id );
//	PRINTF("%s\r\n", data);
//	HAL_Delay(100);
//	float temp = 0;
//    HTS221_GetTemperature(&temp);
//	sprintf((char*)data, "Temp: %6.2f", temp );
//	PRINTF("%s\r\n", data);
//	float hum = 0;
//    HTS221_GetHumidity(&hum);
//	sprintf((char*)data, "Hum: %6.2f", hum );
//	PRINTF("%s\r\n", data);
//	HTS221_Power_OFF();
//
//
//	float x=0.0, y=0.0, z=0.0;
//	float arx=0.0, ary=0.0, arz=0.0;
//	float t=0.0;
//
//	err = ISM330DLC_ReadAcceleration(&x, &y, &z);
//	if(err!=0)
//	{
//	  sprintf(data,"Error: ISM330DLC_ReadAcceleration");
//	  PRINTF("%s\r\n", data);
//	}
//	else
//	{
//	  sprintf((char*)data, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f", x, y, z);
//	  PRINTF("%s\r\n", data);
//	}
//
//	err = ISM330DLC_ReadAngularRate(&arx, &ary, &arz);
//	if(err!=0)
//	{
//	  sprintf(data,"Error: ISM330DLC_ReadAngularRate");
//	  PRINTF("%s\r\n", data);
//	}
//	else
//	{
//	  sprintf((char*)data, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f", arx, ary, arz);
//	  PRINTF("%s\r\n", data);
//	}
//
//	err = ISM330DLC_ReadTemperature(&t);
//	if(err!=0)
//	{
//	  sprintf(data,"Error: ISM330DLC_ReadTemperature");
//	  PRINTF("%s\r\n", data);
//	}
//	else
//	{
//	  sprintf((char*)data, "Temperature [degC]:%6.2f", t);
//	  PRINTF("%s\r\n", data);
//	}
//
//
//	int16_t acc_mean[3] = {0.0};
//	uint16_t acc_std[3] = {0.0};
//	int16_t acc_min[3] = {0.0};
//	int16_t acc_max[3] = {0.0};
//	acc_acq(5000, acc_mean, acc_min, acc_max, acc_std);
//
//	int16_t tilt_mean[3] = {0.0};
//	uint16_t tilt_std[3] = {0.0};
//	int16_t tilt_min[3] = {0.0};
//	int16_t tilt_max[3] = {0.0};
//	tilt_acq(1000, tilt_mean, tilt_min, tilt_max, tilt_std);
//
//
//	  HAL_Delay(30000);
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

  /**Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

static void LORA_HasJoined( void )
{
#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF("JOINED\n\r");
#endif
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
}

static void Send( void )
{
  /* USER CODE BEGIN 3 */

	PRINTF("preSEND\r\n", data);

	uint16_t _vin = 0.0;
	int16_t _temp = 0.0;
	int16_t _hum = 0.0;
	float _x=0.0, _y=0.0, _z=0.0;
	float arx=0.0, ary=0.0, arz=0.0;
	float tmp=0.0;
	float _berkx=0.0, _berky=0.0;
	int16_t _acc_mean[3] = {0.0};
	uint16_t _acc_std[3] = {0.0};
	int16_t _acc_min[3] = {0.0};
	int16_t _acc_max[3] = {0.0};
	int16_t _tilt_mean[3] = {0.0};
	uint16_t _tilt_std[3] = {0.0};
	int16_t _tilt_min[3] = {0.0};
	int16_t _tilt_max[3] = {0.0};

#ifndef ACC
	uint16_t _vstepup = 0.0;
#ifndef RELE
	uint16_t _s1 = 0.0;
	uint16_t _s2 = 0.0;
#else
	uint16_t _s1a = 0.0;
	uint16_t _s1b = 0.0;
	uint16_t _s2a = 0.0;
	uint16_t _s2b = 0.0;
#endif
#endif
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    LORA_Join();
    return;
  }

  if(_start_count>0)
	  _start_count--;
  if( _start_count == 0 && _first_time_flag == 0)
  {
	app_tx_dutycyle = DEFAULT_TX_DUTYCYCLE;
	_first_time_flag = 1;
  }

	PRINTF("preSEND\r\n", data);

	uint32_t i = 0;
	AppData.Port = LORAWAN_APP_PORT;

    uint8_t _type = 0x00;
#if defined(ACC)
    _type = 0x05;
#else
#ifndef RELE
    _type = 0x01;
#else
    _type = 0x02;
#endif
#endif


	uint16_t v = 0;
	float vin = 0.0;
	uint8_t res = 0;
	res = getVIN(&v);
	if(res == 0)
	{
	sprintf(data,"VIN ADC: %d", v);
	PRINTF("%s\r\n", data);
	_vin = v;
	vin = ((float)v)*0.001611328125;
	sprintf(data,"VIN I: %f", vin);
	PRINTF("%s\r\n", data);
	}

//#if defined(ACC)
//	uint8_t ReadValue = 0;
//	BERKELEY_ReadReg(BERKELEY_REG_ADDR_WHO_AM_I, &ReadValue, 0x01);
//	sprintf((char*)data, "BERK id: %d", ReadValue );
//	PRINTF("%s\r\n", data);
//	BERKELEY_GetAcceleration(&_berkx, &_berky, 500);
//	sprintf((char*)data, "x: %6.4f\r\ny: %6.4f\r\n", _berkx, _berky );
//	PRINTF("%s", data);
//#endif

//	float t = 0.0;
//	float h = 0.0;
//	uint8_t id = 0;
//	HTS221Sensor(&hi2c2, HTS221_I2C_ADDRESS);
//	HTS221SensorEnable();
//	HTS221SensorReadID(&id);
//	sprintf((char*)data, "id: %d", id );
//	PRINTF("%s\r\n", data);
//	HTS221SensorGetTemperature(&t);
//	_temp = (uint16_t) (( t + 80 ) * 100);
//	sprintf((char*)data, "Temp: %6.2f", t );
//	PRINTF("%s\r\n", data);
//	HTS221SensorGetHumidity(&h);
//	_hum = (uint16_t) (h * 100);
//	sprintf((char*)data, "Hum: %6.2f", h );
//	PRINTF("%s\r\n", data);
//	HTS221SensorDisable();

	float t = 0.0;
	float h = 0.0;
	uint8_t id = 0;
	HTS221_Init();
	HTS221_ReadID(&id);
	sprintf((char*)data, "id: %d", id );
	PRINTF("%s\r\n", data);
	HTS221_GetTemperature(&t);
	_temp = (int16_t) (t * 100);
	sprintf((char*)data, "Temp: %6.2f", t );
	PRINTF("%s\r\n", data);
	HTS221_GetHumidity(&h);
	_hum = (int16_t) (h * 100);
	sprintf((char*)data, "Hum: %6.2f", h );
	PRINTF("%s\r\n", data);
	HTS221_Power_OFF();

	uint8_t err = 0;
	err = ISM330DLC_ReadAcceleration(&_x, &_y, &_z);
	if(err!=0)
	{
	  sprintf(data,"Error: ISM330DLC_ReadAcceleration");
	  PRINTF("%s\r\n", data);
	}
	else
	{
	  sprintf((char*)data, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f", _x, _y, _z);
	  PRINTF("%s\r\n", data);
	}

	err = ISM330DLC_ReadAngularRate(&arx, &ary, &arz);
	if(err!=0)
	{
	  sprintf(data,"Error: ISM330DLC_ReadAngularRate");
	  PRINTF("%s\r\n", data);
	}
	else
	{
	  sprintf((char*)data, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f", arx, ary, arz);
	  PRINTF("%s\r\n", data);
	}

	err = ISM330DLC_ReadTemperature(&tmp);
	if(err!=0)
	{
	  sprintf(data,"Error: ISM330DLC_ReadTemperature");
	  PRINTF("%s\r\n", data);
	}
	else
	{
	  sprintf((char*)data, "Temperature [degC]:%6.2f", tmp);
	  PRINTF("%s\r\n", data);
	}

#ifdef ACC
	acc_acq(3000, _acc_mean, _acc_min, _acc_max, _acc_std);
	tilt_acq(3000, _tilt_mean, _tilt_min, _tilt_max, _tilt_std);
#else
	acc_acq(3000, _acc_mean, _acc_min, _acc_max, _acc_std);
#endif

#ifndef ACC
	float vstepup = 0;
	int8_t c_power = 3;
	do{
		// STEPUP POWER ON
		HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_SET);

		int8_t c_time = 4;
		do{
			HAL_Delay(500);
			v = 0;
			res = 0;
			res = getVSTEPUP(&v);
			vstepup = ((float)v)*0.0072509765625;
		}while(vstepup < 24 || c_time-- > 0 );

		// STEPUP OK
		if(vstepup >= 24)
			break;

		// STEPUP POWER OFF
		HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
	}while(vstepup < 24 || c_power-- > 0 );

	if(res == 0)
	{
		sprintf(data,"VSTEPUP ADC: %d", v);
		PRINTF("%s\r\n", data);
		_vstepup = v;
		sprintf(data,"VSTEPUP I: %f", ((float)v)*0.0072509765625);
		PRINTF("%s\r\n", data);
	}

	if( (vstepup >= 24) && (vin >= 3.4) )
	{
#ifndef RELE
//	 -------------------- SOLO S1 e S2 ------------------------
		uint16_t s1 = 0;
		uint16_t s2 = 0;
		uint8_t res1 = 0, res2 = 0;
		//---S1---//
		res1 = get420_1(&s1);
		//---S2---//
		res2 = get420_2(&s2);
		if(res1 == 0)
		{
			sprintf(data,"S1 ADC: %d", s1);
			PRINTF("%s\r\n", data);
			_s1 = s1;
			sprintf(data,"S1 I: %f", ((float)s1)*0.000008824359940);
			PRINTF("%s\r\n", data);
		}
		if(res2 == 0)
		{
			sprintf(data,"S2 ADC: %d", s2);
			PRINTF("%s\r\n", data);
			_s2 = s2;
			sprintf(data,"S2 I: %f", ((float)s2)*0.000008824359940);
			PRINTF("%s\r\n", data);
		}
		// STEPUP POWER OFF
		HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);

#else
// -------------------- CON RELE'------------------------
		uint16_t s = 0;
		// RELE' BOARD POWER ON
		HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);

		//---S1---//
		s = 0;
		res = get420_1(&s);
		if(res == 0)
		{
			sprintf(data,"S1a ADC: %d", s);
			PRINTF("%s\r\n", data);
			_s1a = s;
			sprintf(data,"S1a I: %f", (float)((float)s)*0.000008824359940);
			PRINTF("%s\r\n", data);
		}
		// SWITCH RELE'
		HAL_GPIO_WritePin(EN_RELE1_GPIO_Port, EN_RELE1_Pin, GPIO_PIN_RESET);
		HAL_Delay(300);
		s = 0;
		res = get420_1(&s);
		if(res == 0)
		{
			sprintf(data,"S1b ADC: %d", s);
			PRINTF("%s\r\n", data);
			_s1b = s;
			sprintf(data,"S1b I: %f", (float)((float)s)*0.000008824359940);
			PRINTF("%s\r\n", data);
		}
		// DE-SWITCH RELE'
		HAL_GPIO_WritePin(EN_RELE1_GPIO_Port, EN_RELE1_Pin, GPIO_PIN_SET);

		//---S2---//
		s = 0;
		res = get420_2(&s);
		if(res == 0)
		{
			sprintf(data,"S2a ADC: %d", s);
			PRINTF("%s\r\n", data);
			_s2a = s;
			sprintf(data,"S2a I: %f", (float)((double)s)*0.000008824359940);
			PRINTF("%s\r\n", data);
		}
		// SWITCH RELE'
		HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE2_Pin, GPIO_PIN_RESET);
		HAL_Delay(300);
		s = 0;
		res = get420_2(&s);
		if(res == 0)
		{
			sprintf(data,"S2b ADC: %d", s);
			PRINTF("%s\r\n", data);
			_s2b = s;
			sprintf(data,"S2b I: %f", (float)((double)s)*0.000008824359940);
			PRINTF("%s\r\n", data);
		}
		// DE-SWITCH RELE'
		HAL_GPIO_WritePin(EN_RELE2_GPIO_Port, EN_RELE2_Pin, GPIO_PIN_SET);
		// RELE' BOARD POWER OFF
		HAL_GPIO_WritePin(EN_PWR_OUT_GPIO_Port, EN_PWR_OUT_Pin, GPIO_PIN_SET);
		// STEPUP POWER OFF
		HAL_GPIO_WritePin(EN_STEPUP_GPIO_Port, EN_STEPUP_Pin, GPIO_PIN_RESET);
#endif
	}
#endif

  memcpy(&AppData.Buff[i], &_type, sizeof(_type));
  i+=sizeof(_type);
  memcpy(&AppData.Buff[i], &_vin, sizeof(_vin));
  i+=sizeof(_vin);
  memcpy(&AppData.Buff[i], &_temp, sizeof(_temp));
  i+=sizeof(_temp);
  memcpy(&AppData.Buff[i], &_hum, sizeof(_hum));
  i+=sizeof(_hum);
  memcpy(&AppData.Buff[i], _acc_mean, sizeof(_acc_mean[0])*3);
  i+=sizeof(_acc_mean[0])*3;
#ifdef ACC
  memcpy(&AppData.Buff[i], _acc_min, sizeof(_acc_min[0])*3);
  i+=sizeof(_acc_min[0])*3;
  memcpy(&AppData.Buff[i], _acc_max, sizeof(_acc_max[0])*3);
  i+=sizeof(_acc_max[0])*3;
  memcpy(&AppData.Buff[i], _acc_std, sizeof(_acc_std[0])*3);
  i+=sizeof(_acc_std[0])*3;
  memcpy(&AppData.Buff[i], _tilt_mean, sizeof(_tilt_mean[0])*2);
  i+=sizeof(_tilt_mean[0])*2;
  memcpy(&AppData.Buff[i], _tilt_min, sizeof(_tilt_min[0])*2);
  i+=sizeof(_tilt_min[0])*2;
  memcpy(&AppData.Buff[i], _tilt_max, sizeof(_tilt_max[0])*2);
  i+=sizeof(_tilt_max[0])*2;
  memcpy(&AppData.Buff[i], _tilt_std, sizeof(_tilt_std[0])*2);
  i+=sizeof(_tilt_std[0])*2;
#endif
#ifndef ACC
  memcpy(&AppData.Buff[i], &_vstepup, sizeof(_vstepup));
  i+=sizeof(_vstepup);
#ifndef RELE
  memcpy(&AppData.Buff[i], &_s1, sizeof(_s1));
  i+=sizeof(_s1);
  memcpy(&AppData.Buff[i], &_s2, sizeof(_s2));
  i+=sizeof(_s2);
#else
  memcpy(&AppData.Buff[i], &_s1a, sizeof(_s1a));
  i+=sizeof(_s1a);
  memcpy(&AppData.Buff[i], &_s1b, sizeof(_s1b));
  i+=sizeof(_s1b);
  memcpy(&AppData.Buff[i], &_s2a, sizeof(_s2a));
  i+=sizeof(_s2a);
  memcpy(&AppData.Buff[i], &_s2b, sizeof(_s2b));
  i+=sizeof(_s2b);
#endif
#endif

//  AppData.Buff[i++] = 'c';
//  AppData.Buff[i++] = 'i';
//  AppData.Buff[i++] = 'a';
//  AppData.Buff[i++] = 'o';

  AppData.BuffSize = i;

  LORA_send( &AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);

  /* USER CODE END 3 */
}


static void LORA_RxData( lora_AppData_t *AppData )
{
  /* USER CODE BEGIN 4 */
  PRINTF("PACKET RECEIVED ON PORT %d\n\r", AppData->Port);

  switch (AppData->Port)
  {
    case LORAWAN_APP_PORT:
		switch ( AppData->Buff[0] )
		{
			// soft RESET
			case 1:
			{
				sprintf(data,"RESET\n");
				PRINTF("%s\r\n", data);
			    NVIC_SystemReset();
			break;
			}
			// change duty-cycle
			case 2:
			{
				// duty-cycle in seconds
				uint16_t _dc = 180;
				memcpy(&_dc, &AppData->Buff[1], sizeof(uint16_t));
				sprintf(data,"duty-cycle %d\n", _dc);
				PRINTF("%s\r\n", data);
			    app_tx_dutycyle = _dc * 1000;
			break;
			}
			default:
			break;
		}
    break;
  default:
    break;
  }
}

static void OnTxTimerEvent( void )
{
  /*Wait for next tx slot*/
	TimerStop(&TxTimer);
	TimerInit( &TxTimer, OnTxTimerEvent );
	TimerSetValue( &TxTimer,  app_tx_dutycyle);
  TimerStart( &TxTimer);
  /*Send*/
  Send( );
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
//	TimerInit( &TxTimer, OnTxTimerEvent );
//	TimerSetValue( &TxTimer,  app_tx_dutycyle);
    OnTxTimerEvent();
  }
  else
  {
    /* send everytime button is pushed */
//    GPIO_InitTypeDef initStruct={0};
//
//    initStruct.Mode =GPIO_MODE_IT_RISING;
//    initStruct.Pull = GPIO_PULLUP;
//    initStruct.Speed = GPIO_SPEED_HIGH;
//
//    HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
//    HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send );
  }
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void LORA_TxNeeded ( void )
{
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void EndAccAcqusitionTimerEvent( void )
{
	TimerStop(&AccTimer);
	timer_ism330dlc_read_acc_data = 1;
	sprintf((char*)data, "stop acc timer");
	PRINTF("%s\r\n", data);
}

void acc_acq(uint16_t ms, int16_t* acc_mean, int16_t* acc_min, int16_t* acc_max, uint16_t* acc_std)
{
	uint32_t acc_samples = 0;

	ISM330DLC_ResetAccInternals();

	timer_ism330dlc_read_acc_data = 0;
	TimerInit( &AccTimer, EndAccAcqusitionTimerEvent );
	TimerSetValue( &AccTimer,  ms);
	TimerStart( &AccTimer);

	sprintf((char*)data, "start acc timer");
	PRINTF("%s\r\n", data);

	while(timer_ism330dlc_read_acc_data == 0)
	{
		ISM330DLC_ReadAccData();
	}
	ISM330DLC_AccComputeStats(&acc_samples, 2);

	sprintf((char*)data, "counter_acc_data :%d", acc_samples);
	PRINTF("%s\r\n", data);

	double acc_var[3] = {0.0};

	ISM330DLC_GetMeanAcc(acc_mean);
	ISM330DLC_GetVarAcc(acc_var);
	ISM330DLC_GetStdAcc(acc_std);
	ISM330DLC_GetMinAcc(acc_min);
	ISM330DLC_GetMaxAcc(acc_max);

	sprintf((char*)data, "Mean [16]:%d\t%d\t%d", acc_mean[0], acc_mean[1], acc_mean[2]);
	PRINTF("%s\r\n", data);
	sprintf((char*)data, "Min [16]:%d\t%d\t%d", acc_min[0], acc_min[1], acc_min[2]);
	PRINTF("%s\r\n", data);
	sprintf((char*)data, "Max [16]:%d\t%d\t%d", acc_max[0], acc_max[1], acc_max[2]);
	PRINTF("%s\r\n", data);

	float acc_mean_f[3] = {0.0};
	float acc_min_f[3] = {0.0};
	float acc_max_f[3] = {0.0};

	ISM330DLC_AccGetmg(ISM330DLC_2g, acc_mean, acc_mean_f);
	ISM330DLC_AccGetmg(ISM330DLC_2g, acc_min, acc_min_f);
	ISM330DLC_AccGetmg(ISM330DLC_2g, acc_max, acc_max_f);

	sprintf((char*)data, "Mean [mg]:%4.3f\t%4.3f\t%4.3f", acc_mean_f[0], acc_mean_f[1], acc_mean_f[2]);
	PRINTF("%s\r\n", data);
	sprintf((char*)data, "Var [mg]:%4.6f\t%4.6f\t%4.6f", acc_var[0], acc_var[1], acc_var[2]);
	PRINTF("%s\r\n", data);
	sprintf((char*)data, "Min [mg]:%4.3f\t%4.3f\t%4.3f", acc_min_f[0], acc_min_f[1], acc_min_f[2]);
	PRINTF("%s\r\n", data);
	sprintf((char*)data, "Max [mg]:%4.3f\t%4.3f\t%4.3f", acc_max_f[0], acc_max_f[1], acc_max_f[2]);
	PRINTF("%s\r\n", data);
	sprintf((char*)data, "Sdt [16]:%d\t%d\t%d", acc_std[0], acc_std[1], acc_std[2]);
	PRINTF("%s\r\n", data);

}

static void EndTiltAcqusitionTimerEvent( void )
{
	TimerStop(&TiltTimer);
	tim_berkeley_read_tilt_data = 1;
	sprintf((char*)data, "stop tilt timer");
	PRINTF("%s\r\n", data);
}

void tilt_acq(uint16_t ms, int16_t* tilt_mean, int16_t* tilt_min, int16_t* tilt_max, uint16_t* tilt_std)
{
	uint32_t tilt_samples = 0;

	BERKELEY_ResetTiltInternals();

	tim_berkeley_read_tilt_data = 0;
	TimerInit( &TiltTimer, EndTiltAcqusitionTimerEvent );
	TimerSetValue( &TiltTimer,  ms);
	TimerStart( &TiltTimer);

	sprintf((char*)data, "start tilt timer");
	PRINTF("%s\r\n", data);

	while(tim_berkeley_read_tilt_data == 0)
	{
		BERKELEY_ReadTiltData();
	}
	BERKELEY_TiltComputeStats(&tilt_samples, 2000);

	sprintf((char*)data, "counter_tilt_data :%d", tilt_samples);
	PRINTF("%s\r\n", data);

	double tilt_var[2] = {0.0};

	BERKELEY_GetMeanTilt(tilt_mean);
	BERKELEY_GetVarTilt(tilt_var);
	BERKELEY_GetStdTilt(tilt_std);
	BERKELEY_GetMinTilt(tilt_min);
	BERKELEY_GetMaxTilt(tilt_max);

	sprintf((char*)data, "Tilt Mean [16]:%d\t%d", tilt_mean[0], tilt_mean[1]);
	PRINTF("%s\r\n", data);
	sprintf((char*)data, "Tilt Min [16]:%d\t%d", tilt_min[0], tilt_min[1]);
	PRINTF("%s\r\n", data);
	sprintf((char*)data, "Tilt Max [16]:%d\t%d", tilt_max[0], tilt_max[1]);
	PRINTF("%s\r\n", data);

	float tilt_mean_f[2] = {0.0};
	float tilt_min_f[2] = {0.0};
	float tilt_max_f[2] = {0.0};

	int8_t i=0;
	for (i=0;i<2;i++)
	{
		tilt_mean_f[i] = BERKELEY_ConvertMG(tilt_mean[i], 2000);
		tilt_min_f[i] = BERKELEY_ConvertMG(tilt_min[i], 2000);
		tilt_max_f[i] = BERKELEY_ConvertMG(tilt_max[i], 2000);
	}

	sprintf((char*)data, "Tilt Mean [mg]:%4.6f\t%4.6f", tilt_mean_f[0], tilt_mean_f[1]);
	PRINTF("%s\r\n", data);
	sprintf((char*)data, "Tilt Var [mg]:%4.6f\t%4.6f", tilt_var[0], tilt_var[1]);
	PRINTF("%s\r\n", data);
	sprintf((char*)data, "Tilt Min [mg]:%4.6f\t%4.6f", tilt_min_f[0], tilt_min_f[1]);
	PRINTF("%s\r\n", data);
	sprintf((char*)data, "Tilt Max [mg]:%4.6f\t%4.6f", tilt_max_f[0], tilt_max_f[1]);
	PRINTF("%s\r\n", data);
	sprintf((char*)data, "Tilt Sdt [16]:%d\t%d", tilt_std[0], tilt_std[1]);
	PRINTF("%s\r\n", data);

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
	NVIC_SystemReset();
	PRINTF("Error_Handler\n\r");
	  while (1);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
