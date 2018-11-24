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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "../cs43l22/cs43l22.h"
#include "../common/audio.h"
#include "../stm32l476g_discovery.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define AUDIO_RST_Pin GPIO_PIN_3
#define AUDIO_RST_GPIO_Port GPIOE
#define ADC0_Pin GPIO_PIN_0
#define ADC0_GPIO_Port GPIOC
#define ADC1_Pin GPIO_PIN_1
#define ADC1_GPIO_Port GPIOC
#define ADC2_Pin GPIO_PIN_2
#define ADC2_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_6
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOA
#define SMOKE_HTR_Pin GPIO_PIN_7
#define SMOKE_HTR_GPIO_Port GPIOE
#define GAS_HTR_Pin GPIO_PIN_8
#define GAS_HTR_GPIO_Port GPIOE
#define C0_HTR_Pin GPIO_PIN_9
#define C0_HTR_GPIO_Port GPIOE
#define INT_2_Pin GPIO_PIN_10
#define INT_2_GPIO_Port GPIOE
#define INT_2_EXTI_IRQn EXTI15_10_IRQn
#define INT_1_Pin GPIO_PIN_11
#define INT_1_GPIO_Port GPIOE
#define INT_1_EXTI_IRQn EXTI15_10_IRQn
#define G7_Pin GPIO_PIN_12
#define G7_GPIO_Port GPIOE
#define B7_Pin GPIO_PIN_13
#define B7_GPIO_Port GPIOE
#define R7_Pin GPIO_PIN_14
#define R7_GPIO_Port GPIOE
#define B6_Pin GPIO_PIN_13
#define B6_GPIO_Port GPIOB
#define G6_Pin GPIO_PIN_14
#define G6_GPIO_Port GPIOB
#define R6_Pin GPIO_PIN_15
#define R6_GPIO_Port GPIOB
#define G5_Pin GPIO_PIN_8
#define G5_GPIO_Port GPIOD
#define B5_Pin GPIO_PIN_9
#define B5_GPIO_Port GPIOD
#define R5_Pin GPIO_PIN_10
#define R5_GPIO_Port GPIOD
#define B4_Pin GPIO_PIN_11
#define B4_GPIO_Port GPIOD
#define G4_Pin GPIO_PIN_12
#define G4_GPIO_Port GPIOD
#define R4_Pin GPIO_PIN_13
#define R4_GPIO_Port GPIOD
#define B3_Pin GPIO_PIN_14
#define B3_GPIO_Port GPIOD
#define G3_Pin GPIO_PIN_15
#define G3_GPIO_Port GPIOD
#define R3_Pin GPIO_PIN_6
#define R3_GPIO_Port GPIOC
#define B2_Pin GPIO_PIN_7
#define B2_GPIO_Port GPIOC
#define G2_Pin GPIO_PIN_8
#define G2_GPIO_Port GPIOC
#define R2_Pin GPIO_PIN_9
#define R2_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_10
#define B1_GPIO_Port GPIOC
#define G1_Pin GPIO_PIN_11
#define G1_GPIO_Port GPIOC
#define R1_Pin GPIO_PIN_12
#define R1_GPIO_Port GPIOC
#define SMOKE_Pin GPIO_PIN_1
#define SMOKE_GPIO_Port GPIOD
#define GAS_Pin GPIO_PIN_2
#define GAS_GPIO_Port GPIOD
#define RING_B_Pin GPIO_PIN_3
#define RING_B_GPIO_Port GPIOD
#define RING_G_Pin GPIO_PIN_4
#define RING_G_GPIO_Port GPIOD
#define RING_R_Pin GPIO_PIN_5
#define RING_R_GPIO_Port GPIOD
#define SOUND_Pin GPIO_PIN_6
#define SOUND_GPIO_Port GPIOD
#define BT_Pin GPIO_PIN_7
#define BT_GPIO_Port GPIOD
#define NITE_Pin GPIO_PIN_3
#define NITE_GPIO_Port GPIOB
#define MOTION_Pin GPIO_PIN_4
#define MOTION_GPIO_Port GPIOB
#define PWR_Pin GPIO_PIN_5
#define PWR_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
