/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ISM330DLC_INT_Pin GPIO_PIN_13
#define ISM330DLC_INT_GPIO_Port GPIOC
#define _1_WIRE_Pin GPIO_PIN_0
#define _1_WIRE_GPIO_Port GPIOC
#define EN_STEPUP_Pin GPIO_PIN_1
#define EN_STEPUP_GPIO_Port GPIOC
#define EN_ADC_VIN_Pin GPIO_PIN_2
#define EN_ADC_VIN_GPIO_Port GPIOC
#define OPAMP1_ANALOG1_Pin GPIO_PIN_0
#define OPAMP1_ANALOG1_GPIO_Port GPIOA
#define ADC_STEPUP_Pin GPIO_PIN_1
#define ADC_STEPUP_GPIO_Port GPIOA
#define BERK_INT_Pin GPIO_PIN_2
#define BERK_INT_GPIO_Port GPIOA
#define OPAMP2_ANALOG2_Pin GPIO_PIN_6
#define OPAMP2_ANALOG2_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOA
#define ADC_VIN_Pin GPIO_PIN_4
#define ADC_VIN_GPIO_Port GPIOC
#define INT_NFC_Pin GPIO_PIN_5
#define INT_NFC_GPIO_Port GPIOC
#define EXT_INT_Pin GPIO_PIN_1
#define EXT_INT_GPIO_Port GPIOB
#define EXT_GPIO_Output_Pin GPIO_PIN_2
#define EXT_GPIO_Output_GPIO_Port GPIOB
#define BERK_CS_Pin GPIO_PIN_12
#define BERK_CS_GPIO_Port GPIOB
#define BERK_SCK_Pin GPIO_PIN_13
#define BERK_SCK_GPIO_Port GPIOB
#define BERK_MISO_Pin GPIO_PIN_14
#define BERK_MISO_GPIO_Port GPIOB
#define BERK_MOSI_Pin GPIO_PIN_15
#define BERK_MOSI_GPIO_Port GPIOB
#define RADIO_DIO0_Pin GPIO_PIN_6
#define RADIO_DIO0_GPIO_Port GPIOC
#define RADIO_DIO1_Pin GPIO_PIN_7
#define RADIO_DIO1_GPIO_Port GPIOC
#define RADIO_DIO2_Pin GPIO_PIN_8
#define RADIO_DIO2_GPIO_Port GPIOC
#define RADIO_DIO3_Pin GPIO_PIN_9
#define RADIO_DIO3_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define RADIO_NRESET_Pin GPIO_PIN_9
#define RADIO_NRESET_GPIO_Port GPIOA
#define EN_CAN_Pin GPIO_PIN_10
#define EN_CAN_GPIO_Port GPIOA
#define EXT_GPIO_Analog_Pin GPIO_PIN_15
#define EXT_GPIO_Analog_GPIO_Port GPIOA
#define DEBUG_USART3_TX_Pin GPIO_PIN_10
#define DEBUG_USART3_TX_GPIO_Port GPIOC
#define DEBUG_USART3_RX_Pin GPIO_PIN_11
#define DEBUG_USART3_RX_GPIO_Port GPIOC
#define RFDIS_Pin GPIO_PIN_12
#define RFDIS_GPIO_Port GPIOC
#define RADIO_NSS_Pin GPIO_PIN_2
#define RADIO_NSS_GPIO_Port GPIOD
#define RADIO_SCK_Pin GPIO_PIN_3
#define RADIO_SCK_GPIO_Port GPIOB
#define RADIO_MISO_Pin GPIO_PIN_4
#define RADIO_MISO_GPIO_Port GPIOB
#define RADIO_MOSI_Pin GPIO_PIN_5
#define RADIO_MOSI_GPIO_Port GPIOB
#define EN_RELE2_Pin GPIO_PIN_6
#define EN_RELE2_GPIO_Port GPIOB
#define EN_RELE1_Pin GPIO_PIN_7
#define EN_RELE1_GPIO_Port GPIOB
#define EN_PWR_OUT_Pin GPIO_PIN_8
#define EN_PWR_OUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* --------------------------- USART HW definition -------------------------------*/


#define USARTx                           USART3
#define USARTx_CLK_ENABLE()              __USART3_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART3_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART3_RELEASE_RESET()


#define USARTx_TX_PIN                  GPIO_PIN_10
#define USARTx_TX_GPIO_PORT            GPIOC
#define USARTx_TX_AF                   GPIO_AF7_USART3
#define USARTx_RX_PIN                  GPIO_PIN_11
#define USARTx_RX_GPIO_PORT            GPIOC
#define USARTx_RX_AF                   GPIO_AF7_USART3

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART3_IRQn
#define USARTx_IRQHandler                USART3_IRQHandler //USART2_IRQHandler

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Channel2

/* Definition for USARTx's DMA Request */
#define USARTx_TX_DMA_REQUEST             DMA_REQUEST_2

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Channel2_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Channel2_IRQHandler

#define USARTx_Priority 0
#define USARTx_DMA_Priority 0

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
