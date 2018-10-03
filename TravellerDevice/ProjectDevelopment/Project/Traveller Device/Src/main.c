
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SAI_HandleTypeDef hsai_BlockA1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define REFdebounce 50

/* Private variables ---------------------------------------------------------*/
_Bool POWERON = 0x00;
_Bool POWERON_State = 0x00;
_Bool PWSW = 0x00;
uint8_t cntboot = 0x00;
uint32_t POWERON_1 = 0;
uint32_t POWERON_0 = 0;
uint32_t sensors 	=	0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SAI1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
void heartBeat_loop(uint8_t initial_val, uint8_t end_val, uint8_t n);
void heartBeat_loop2(uint8_t initial_val, uint8_t end_val, uint8_t n);
void test_pwm_blink(void);
void startupF(void);
void shutF(void);
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SAI1_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(PWSW) // PWR ON [TurnON001]
		{
			//HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_SET);
			//HAL_Delay(500);
			//test_pwm_blink();
			if(cntboot < 1) //Run this for only first time booting
			{		
				HAL_GPIO_WritePin(RING_B_GPIO_Port, RING_B_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_SET);
				cntboot=+1;
				heartBeat_loop2(0,39,1);	
				heartBeat_loop(39,9,0);
				heartBeat_loop(9,39,1);
				heartBeat_loop(39,3,0);
				heartBeat_loop(3,39,1);
				heartBeat_loop(39,3,0);
				heartBeat_loop(3,39,1);
				heartBeat_loop(39,3,0);
				heartBeat_loop(3,9,1);
				
			}
			if((sensors) & (PWSW))
			{
				HAL_GPIO_WritePin(RING_B_GPIO_Port, RING_B_Pin, GPIO_PIN_SET);
				heartBeat_loop2(0,39,1);	
				heartBeat_loop(39,9,0);
				heartBeat_loop(9,39,1);
				heartBeat_loop(39,3,0);
				heartBeat_loop(3,39,1);
				heartBeat_loop(39,3,0);
				heartBeat_loop(3,39,1);
				heartBeat_loop(39,3,0);
				heartBeat_loop(3,9,1);
				sensors = 0; 		
			}
			
		}
		else //PWR OFF  [TurnOFF001]
		{
			if(cntboot>=1) // run this if the turn on routine have run before. 
				{
					//Off light S
					heartBeat_loop2(9,3,0);
					heartBeat_loop(3,39,1);
					heartBeat_loop(39,3,0);
					heartBeat_loop(3,39,1);
					heartBeat_loop(39,3,0);
					heartBeat_loop(3,39,1);
					heartBeat_loop(39,9,0);
					heartBeat_loop(9,39,1);
					heartBeat_loop(39,1,0);
				}
				//HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RING_B_GPIO_Port, RING_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_RESET);	
			//HAL_Delay(2500);
			//PWSW=0;
			cntboot=0;	
		}
		
		

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
	
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
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
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SAI1 init function */
static void MX_SAI1_Init(void)
{

  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_24;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 8;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 1;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 20000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 40;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 10000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 40-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim16);

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 10000-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 40-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 9;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim17);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, AUDIO_RST_Pin|SMOKE_HTR_Pin|GAS_HTR_Pin|C0_HTR_Pin 
                          |G7_Pin|B7_Pin|R7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, B6_Pin|G6_Pin|R6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, G5_Pin|B5_Pin|R5_Pin|B4_Pin 
                          |G4_Pin|R4_Pin|B3_Pin|G3_Pin 
                          |RING_B_Pin|RING_G_Pin|RING_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R3_Pin|B2_Pin|G2_Pin|R2_Pin 
                          |B1_Pin|G1_Pin|R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AUDIO_RST_Pin SMOKE_HTR_Pin GAS_HTR_Pin C0_HTR_Pin 
                           G7_Pin B7_Pin R7_Pin */
  GPIO_InitStruct.Pin = AUDIO_RST_Pin|SMOKE_HTR_Pin|GAS_HTR_Pin|C0_HTR_Pin 
                          |G7_Pin|B7_Pin|R7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_2_Pin INT_1_Pin */
  GPIO_InitStruct.Pin = INT_2_Pin|INT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : B6_Pin G6_Pin R6_Pin */
  GPIO_InitStruct.Pin = B6_Pin|G6_Pin|R6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : G5_Pin B5_Pin R5_Pin B4_Pin 
                           G4_Pin R4_Pin B3_Pin G3_Pin 
                           RING_B_Pin RING_G_Pin RING_R_Pin */
  GPIO_InitStruct.Pin = G5_Pin|B5_Pin|R5_Pin|B4_Pin 
                          |G4_Pin|R4_Pin|B3_Pin|G3_Pin 
                          |RING_B_Pin|RING_G_Pin|RING_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : R3_Pin B2_Pin G2_Pin R2_Pin 
                           B1_Pin G1_Pin R1_Pin */
  GPIO_InitStruct.Pin = R3_Pin|B2_Pin|G2_Pin|R2_Pin 
                          |B1_Pin|G1_Pin|R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTION_Pin SMOKE_Pin GAS_Pin SOUND_Pin */
  GPIO_InitStruct.Pin = MOTION_Pin|SMOKE_Pin|GAS_Pin|SOUND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_Pin NITE_Pin */
  GPIO_InitStruct.Pin = BT_Pin|NITE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_Pin */
  GPIO_InitStruct.Pin = PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void test_pwm_blink(void)
{
		for(int x = 0; x<40; ) 
  {
		
		HAL_GPIO_WritePin(RING_R_GPIO_Port, RING_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RING_G_GPIO_Port, RING_G_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RING_B_GPIO_Port, RING_B_Pin, GPIO_PIN_SET);
		TIM16->CCR1= x;
		x=x+5;
		HAL_Delay(50);		
  }
	
		HAL_GPIO_TogglePin(RING_R_GPIO_Port, RING_R_Pin);
		HAL_GPIO_TogglePin(RING_G_GPIO_Port, RING_G_Pin);
		HAL_GPIO_TogglePin(RING_B_GPIO_Port, RING_B_Pin);
		HAL_Delay(50);
}
void HAL_SYSTICK_Callback(void)
{
	POWERON = HAL_GPIO_ReadPin(PWR_GPIO_Port, PWR_Pin);
	
	if(POWERON != 1)  //Pressed 
	{
		//PWSW = 0;
		POWERON_1 ++; // Is a count var of button preess increment 1 by 1ms 
		POWERON_0 = 0; // debounce the zero value 
		if(POWERON_1 >= REFdebounce)
		{	
			if(POWERON_1>2000) // Detecing Switch off for trigger action 
			{
				if(PWSW)
				{
					PWSW = 0;
				}
				else
				{
					PWSW = 1;
				}
				//PWSW = 1;
				POWERON_1 = REFdebounce +1;	
				//POWERON_1 = REFdebounce +1;
			}
			
			else if ((POWERON_1> 150) & (POWERON_1< 250)) // Detecing Switch On for trigger action 
				{				
					//sensors &= ~( HAL_GPIO_ReadPin(CH1_1_GPIO_Port, CH1_1_Pin) << 23);
				if(sensors)
					{
						sensors = 0;
					}
				else
					{
						sensors = 1;
					}
				}
			//POWERON_1 = REFdebounce +1;	
			//POWERON_1 = REFdebounce +1;
			
			POWERON_State = 1;
		}
	}
		else
		{
			POWERON_1 =0;
			POWERON_0++;
			if(POWERON_0 >= REFdebounce)
			{
			POWERON_0 = REFdebounce +1 ;
			POWERON_State = 0;
			}
		}
		
	
	
}

void startupF(void)
{
	
	for(int x = 0; x<40; ) 
  {
		
		HAL_GPIO_WritePin(RING_R_GPIO_Port, RING_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RING_G_GPIO_Port, RING_G_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RING_B_GPIO_Port, RING_B_Pin, GPIO_PIN_SET);
		TIM16->CCR1= x;
		x=x+5;
		HAL_Delay(250);	
		switch (x)
      {
      	case 15:
					HAL_GPIO_WritePin(G4_GPIO_Port,G4_Pin,GPIO_PIN_SET); 
      		break;
      	case 25:
					HAL_GPIO_WritePin(G6_GPIO_Port,G6_Pin,GPIO_PIN_SET); //NT
					HAL_GPIO_WritePin(G5_GPIO_Port,G5_Pin,GPIO_PIN_SET); //BLE
					HAL_GPIO_WritePin(G3_GPIO_Port,G3_Pin,GPIO_PIN_SET); //GS
					HAL_GPIO_WritePin(G2_GPIO_Port,G2_Pin,GPIO_PIN_SET); // Smoke
      		break;
				case 40:
					HAL_GPIO_WritePin(G1_GPIO_Port,G1_Pin,GPIO_PIN_SET);
      		break;
      	default:
      		break;
      }	
		
  }
	
	for(uint8_t i =0; i<4; i++)
	{
		HAL_GPIO_TogglePin(RING_R_GPIO_Port, RING_R_Pin);
		HAL_GPIO_TogglePin(RING_G_GPIO_Port, RING_G_Pin);
		HAL_GPIO_TogglePin(RING_B_GPIO_Port, RING_B_Pin);
		
		HAL_Delay(1000);
	}
	
		HAL_GPIO_WritePin(RING_R_GPIO_Port, RING_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RING_G_GPIO_Port, RING_G_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RING_B_GPIO_Port, RING_B_Pin, GPIO_PIN_SET);
	
}

void shutF(void)
{
	
	for(int x = 40; x>0; ) 
  {
		
		HAL_GPIO_WritePin(RING_R_GPIO_Port, RING_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RING_G_GPIO_Port, RING_G_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RING_B_GPIO_Port, RING_B_Pin, GPIO_PIN_SET);
		TIM16->CCR1= x;
		x=x-5;
		HAL_Delay(250);	
		switch (x)
      {
      	case 35:
					HAL_GPIO_WritePin(G4_GPIO_Port,G4_Pin,GPIO_PIN_RESET); //snd
      		break;
      	case 20:
					HAL_GPIO_WritePin(G6_GPIO_Port,G6_Pin,GPIO_PIN_RESET); //NT
					HAL_GPIO_WritePin(G5_GPIO_Port,G5_Pin,GPIO_PIN_RESET); //BLE
					HAL_GPIO_WritePin(G3_GPIO_Port,G3_Pin,GPIO_PIN_RESET); //GS
					HAL_GPIO_WritePin(G2_GPIO_Port,G2_Pin,GPIO_PIN_RESET); // Smoke
      		break;
				case 15:
					HAL_GPIO_WritePin(G1_GPIO_Port,G1_Pin,GPIO_PIN_RESET); // motion
      		break;
      	default:
      		break;
      }	
		
  }
	
	for(uint8_t i =0; i<4; i++)
	{
		HAL_GPIO_TogglePin(RING_R_GPIO_Port, RING_R_Pin);
		HAL_GPIO_TogglePin(RING_G_GPIO_Port, RING_G_Pin);
		HAL_GPIO_TogglePin(RING_B_GPIO_Port, RING_B_Pin);
		
		HAL_Delay(1000);
	}
	
		HAL_GPIO_WritePin(RING_R_GPIO_Port, RING_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RING_G_GPIO_Port, RING_G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RING_B_GPIO_Port, RING_B_Pin, GPIO_PIN_RESET);
	
}



void heartBeat_loop(uint8_t initial_val, uint8_t end_val, uint8_t n)
{
		int delayl = 0;
		HAL_GPIO_WritePin(RING_B_GPIO_Port, RING_B_Pin, GPIO_PIN_SET);
	//only run this loop when the initial value max = 40
		if(n==0) // when light ramp down slope
	{
			for(uint8_t x = initial_val; x>=end_val;) // 0- 100 in 2 sec
			{
			TIM16->CCR1= x;
			x = x - 1;
				
			//Detect 13 times 10ms of TIM6 has elapsed.
			delayl = 1000/(initial_val-end_val);
			HAL_Delay(delayl);	
					
			//Create a delay of 130ms
			//loop will end with PWM val = 10 so 25% brightness 
			}
	}
	//carry this loop when initial value is at lowest = 20
		else if (n == 1)// up slope
	{
			for(uint8_t x = initial_val; x<=end_val;)
			{
			TIM16->CCR1= x;
			x = x + 1;
			delayl = 1000/(end_val-initial_val);
			//Create a delay of 130ms
				
			HAL_Delay(delayl);	
			//loop will end with PWM val = 10 so 25% brightness 
			}
	}
}




void heartBeat_loop2(uint8_t initial_val, uint8_t end_val, uint8_t n)
{
		int delayl = 0;
		HAL_GPIO_WritePin(RING_B_GPIO_Port, RING_B_Pin, GPIO_PIN_SET);
	//only run this loop when the initial value max = 40
		if(n==0) // when light ramp down slope
	{
			for(uint8_t x = initial_val; x>=end_val;) // 0- 100 in 2 sec
			{
			TIM16->CCR1= x;
			x = x - 1;
				
			//Detect 13 times 10ms of TIM6 has elapsed.
			delayl = 2000/(initial_val-end_val);
			HAL_Delay(delayl);	
					
			//Create a delay of 130ms
			//loop will end with PWM val = 10 so 25% brightness 
			}
	}
	//carry this loop when initial value is at lowest = 20
		else if (n == 1)// up slope
	{
			for(uint8_t x = initial_val; x<=end_val;)
			{
			TIM16->CCR1= x;
			x = x + 1;
			delayl = 2000/(end_val-initial_val);
			//Create a delay of 130ms
				
			HAL_Delay(delayl);	
			//loop will end with PWM val = 10 so 25% brightness 
			}
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
