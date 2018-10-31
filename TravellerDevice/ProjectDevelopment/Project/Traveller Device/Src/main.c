
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
#define AUDIO_FILE_ADDRESS   0x08080000
__IO int16_t                 UpdatePointer = -1;

#define AUDIO_FILE_SIZE      (180*1024)
#define PLAY_HEADER          0x2C
#define PLAY_BUFF_SIZE       4096
DMA_HandleTypeDef            hSaiDma;

uint16_t                      PlayBuff[PLAY_BUFF_SIZE];
void readSensors();
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SAI_HandleTypeDef    SaiHandle;
AUDIO_DrvTypeDef            *audio_drv;
#define REFdebounce 50

/* Private variables ---------------------------------------------------------*/
uint32_t flame_a0 = 0x00;
uint32_t CO_a1 = 0x00;
uint32_t smoke_a2 = 0x00;
uint32_t heaterTime = 0x00;
uint32_t offHeaterTime = 0x00;
_Bool offCountHeater = 0x00;
_Bool countHeater = 0x00;
_Bool POWERON = 0x00;
_Bool	SMOKEON =	0x00;
_Bool	GASON		=	0x00;
_Bool	MOVEON	=	0x00;
_Bool	SOUNDON	=	0x00;
_Bool NITEON	= 0x00;
_Bool	 SCANROOM = 0x00;
_Bool	BTON		=	0x00;
_Bool POWERON_State = 0x00;
_Bool PWSW = 0x00;
_Bool clear_1 = 1;
_Bool clear_2 = 1;
_Bool clear_3 = 1;
_Bool clear_4 = 1;
_Bool clear_5 = 1;
_Bool clear_6 = 1;
uint8_t cntboot = 0x00;
uint32_t POWERON_1 = 0;
uint32_t POWERON_0 = 0;
uint32_t sensors 	=	0;

uint32_t	SMOKEON_1 =	0x00;
uint32_t	SMOKEON_0 =	0x00;

uint32_t	GASON_1		=	0x00;
uint32_t	GASON_0		=	0x00;

uint32_t	MOVEON_1	=	0x00;
uint32_t	MOVEON_0	=	0x00;

uint32_t	SOUNDON_1	=	0x00;
uint32_t	SOUNDON_0	=	0x00;

uint32_t NITEON_1	= 0x00;
uint32_t NITEON_0	= 0x00;

uint32_t	 SCANROOM_1 = 0x00;
uint32_t	 SCANROOM_0 = 0x00;

uint32_t	BTON_1		=	0x00;
uint32_t	BTON_0		=	0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
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
static void Playback_Init(void)
{
		
	 __HAL_SAI_ENABLE(&hsai_BlockA1);
	
	if(CS43L22_ID!= cs43l22_drv.ReadID(AUDIO_I2C_ADDRESS))
  {
    Error_Handler();
  }
  
  audio_drv = &cs43l22_drv;
  audio_drv->Reset(AUDIO_I2C_ADDRESS);  
  if(0 != audio_drv->Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_SPEAKER, 100, AUDIO_FREQUENCY_22K))
  {
    Error_Handler();
  }
}
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
  uint32_t PlaybackPosition   = PLAY_BUFF_SIZE + PLAY_HEADER;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
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
	if(*((uint64_t *)AUDIO_FILE_ADDRESS) != 0x017EFE2446464952 ) Error_Handler();
	Playback_Init();
	
	for(int i=0; i < PLAY_BUFF_SIZE; i+=2)
  {
    PlayBuff[i]=*((__IO uint16_t *)(AUDIO_FILE_ADDRESS + PLAY_HEADER + i));
  }
    
  /* Start the playback */
  if(0 != audio_drv->Play(AUDIO_I2C_ADDRESS, NULL, 0))
  {
    Error_Handler();
  }
  if(HAL_OK != HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)PlayBuff, PLAY_BUFF_SIZE))
  {
    Error_Handler();
  }
	
	
	
	countHeater = 0x01;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		while(UpdatePointer==-1){
		
		}
    
    int position = UpdatePointer;
    UpdatePointer = -1;

    /* Upate the first or the second part of the buffer */
    for(int i = 0; i < PLAY_BUFF_SIZE/2; i++)
    {
      PlayBuff[i+position] = *(uint16_t *)(AUDIO_FILE_ADDRESS + PlaybackPosition);
      PlaybackPosition+=2; 
    }

    /* check the end of the file */
    if((PlaybackPosition+PLAY_BUFF_SIZE/2) > AUDIO_FILE_SIZE)
    {
      PlaybackPosition = PLAY_HEADER;
    }
    
    if(UpdatePointer != -1)
    {
      /* Buffer update time is too long compare to the data transfer time */
      Error_Handler();
    }
		
		//Playback_Init();
		if(PWSW) // PWR ON [TurnON001]
		{
			//HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_SET);
			//HAL_Delay(500);
			//test_pwm_blink();
			//START
			if(cntboot < 1) //Run this for only first time booting
			{		
				HAL_SAI_DMAPause(&hsai_BlockA1);
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
				HAL_SAI_DMAResume(&hsai_BlockA1);
			}
			
			//Read sensors
			readSensors();
		
			if((SCANROOM) & (PWSW))
			{
				HAL_SAI_DMAPause(&hsai_BlockA1);
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
				SCANROOM = 0; 		
				HAL_SAI_DMAResume(&hsai_BlockA1);
			}
			
			if((sensors >> 4) & 1)
			{
				HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);
			}
			
			if((sensors >> 5) & 1)
			{
				HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_RESET);
			}
			if((sensors >> 6) & 1)
			{
				HAL_GPIO_WritePin(B5_GPIO_Port, B5_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(B5_GPIO_Port, B5_Pin, GPIO_PIN_RESET);
			}
			
			if((sensors >> 7) & 1)
			{
				HAL_GPIO_WritePin(B6_GPIO_Port, B6_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(B6_GPIO_Port, B6_Pin, GPIO_PIN_RESET);
			}
			
			if((sensors >> 8) & 1)
			{
				HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, GPIO_PIN_RESET);
			}
			
			if((sensors >> 9) & 1)
			{
				HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, GPIO_PIN_RESET);
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
				
			HAL_GPIO_WritePin(RING_B_GPIO_Port, RING_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(B7_GPIO_Port,B7_Pin,GPIO_PIN_RESET);	
			
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
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV17;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
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
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_22K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 32;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 16;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 2;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000003;
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

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

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

  /*Configure GPIO pins : SMOKE_Pin GAS_Pin SOUND_Pin BT_Pin */
  GPIO_InitStruct.Pin = SMOKE_Pin|GAS_Pin|SOUND_Pin|BT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : NITE_Pin MOTION_Pin PWR_Pin */
  GPIO_InitStruct.Pin = NITE_Pin|MOTION_Pin|PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	SMOKEON =	HAL_GPIO_ReadPin(SMOKE_GPIO_Port,SMOKE_Pin); 
	GASON		= HAL_GPIO_ReadPin(GAS_GPIO_Port,GAS_Pin); 
	MOVEON	=	HAL_GPIO_ReadPin(MOTION_GPIO_Port,MOTION_Pin); 
	SOUNDON	= HAL_GPIO_ReadPin(SOUND_GPIO_Port,SOUND_Pin);
	BTON		= HAL_GPIO_ReadPin(BT_GPIO_Port,BT_Pin);
	NITEON	= HAL_GPIO_ReadPin(NITE_GPIO_Port,NITE_Pin);
	
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
				if(SCANROOM)
					{
						SCANROOM = 0;
					}
				else
					{
						SCANROOM = 1;
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
		
if(SMOKEON != 1)  //Pressed 
	{
		
		SMOKEON_1 ++; // Is a count var of button preess increment 1 by 1ms 
		SMOKEON_0 = 0; // debounce the zero value 
		if(SMOKEON_1 >= REFdebounce)
		{	
			if(SMOKEON_1>150) // Detecing Switch off for trigger action 
			{
				if(clear_1)
				{
				sensors =  sensors ^ (1 << 4);
				clear_1 = 0;
				}
				SMOKEON_1 = REFdebounce +1;	
			}
			
		}
	}
		else
		{
			clear_1 = 1;
			SMOKEON_1 =0;
			SMOKEON_0++;
			if(SMOKEON_0 >= REFdebounce)
			{
			SMOKEON_0 = REFdebounce +1 ;
			}
		}
		


if(GASON != 1)  //Pressed 
	{
		
		GASON_1 ++; // Is a count var of button preess increment 1 by 1ms 
		GASON_0 = 0; // debounce the zero value 
		if(GASON_1 >= REFdebounce)
		{	
			if(GASON_1>150) // Detecing Switch off for trigger action 
			{
				if(clear_2)
				{
				sensors =  sensors ^ (1 << 5);
				clear_2 = 0;
				}
				GASON_1 = REFdebounce +1;	
			}
			
		}
	}
		else
		{
			clear_2 = 1;
			GASON_1 =0;
			GASON_0++;
			if(GASON_0 >= REFdebounce)
			{
			GASON_0 = REFdebounce +1 ;
			}
		}

if(MOVEON != 1)  //Pressed 
	{
		
		MOVEON_1 ++; // Is a count var of button preess increment 1 by 1ms 
		MOVEON_0 = 0; // debounce the zero value 
		if(MOVEON_1 >= REFdebounce)
		{	
			if(MOVEON_1>150) // Detecing Switch off for trigger action 
			{
				if(clear_3)
				{
				sensors =  sensors ^ (1 << 7);
				clear_3 = 0;
				}
				MOVEON_1 = REFdebounce +1;	
			}
			
		}
	}
		else
		{
			clear_3 = 1;
			MOVEON_1 =0;
			MOVEON_0++;
			if(MOVEON_0 >= REFdebounce)
			{
			MOVEON_0 = REFdebounce +1 ;
			}
		}

if(NITEON != 1)  //Pressed 
	{
		
		NITEON_1 ++; // Is a count var of button preess increment 1 by 1ms 
		NITEON_0 = 0; // debounce the zero value 
		if(NITEON_1 >= REFdebounce)
		{	
			if(NITEON_1>150) // Detecing Switch off for trigger action 
			{
				if(clear_4)
				{
				sensors =  sensors ^ (1 << 6);
				clear_4 = 0;
				}
				NITEON_1 = REFdebounce +1;	
			}
			
		}
	}
		else
		{
			clear_4 = 1;
			NITEON_1 =0;
			NITEON_0++;
			if(NITEON_0 >= REFdebounce)
			{
			NITEON_0 = REFdebounce +1 ;
			}
		}

if(SOUNDON != 1)  //Pressed 
	{
		
		SOUNDON_1 ++; // Is a count var of button preess increment 1 by 1ms 
		SOUNDON_0 = 0; // debounce the zero value 
		if(SOUNDON_1 >= REFdebounce)
		{	
			if(SOUNDON_1>150) // Detecing Switch off for trigger action 
			{
				if(clear_5)
				{
				sensors =  sensors ^ (1 << 8);
				clear_5 = 0;
				}
				SOUNDON_1 = REFdebounce +1;	
			}
			
		}
	}
		else
		{
			clear_5 = 1;
			SOUNDON_1 =0;
			SOUNDON_0++;
			if(SOUNDON_0 >= REFdebounce)
			{
			SOUNDON_0 = REFdebounce +1 ;
			}
		}
		
if(BTON != 1)  //Pressed 
	{
		
		BTON_1 ++; // Is a count var of button preess increment 1 by 1ms 
		BTON_0 = 0; // debounce the zero value 
		if(BTON_1 >= REFdebounce)
		{	
			if(BTON_1>150) // Detecing Switch off for trigger action 
			{
				if(clear_6)
				{
				sensors =  sensors ^ (1 << 9);
				clear_6 = 0;
				}
				BTON_1 = REFdebounce +1;	
			}
			
		}
	}
		else
		{
			clear_6 = 1;
			BTON_1 =0;
			BTON_0++;
			if(BTON_0 >= REFdebounce)
			{
			BTON_0 = REFdebounce +1 ;
			}
		}		
	
	if(countHeater)
	{
		heaterTime ++;
	}
	else
	{
		heaterTime = 0;
	}
	if(offCountHeater)
	{
		offHeaterTime ++;
	}
	else
	{
		offHeaterTime = 0;
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

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SAI_TxCpltCallback could be implemented in the user file
   */ 
  UpdatePointer = PLAY_BUFF_SIZE/2;
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SAI_TxHalfCpltCallback could be implenetd in the user file
   */ 
  UpdatePointer = 0;
}

void readSensors()
{
if(countHeater)
	{
	HAL_GPIO_WritePin(C0_HTR_GPIO_Port,C0_HTR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GAS_HTR_GPIO_Port,GAS_HTR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SMOKE_HTR_GPIO_Port,SMOKE_HTR_Pin, GPIO_PIN_SET);
	}
if(heaterTime >= 30000U)
	{
		//readValues analog/Digital.
		HAL_ADC_Stop(&hadc1);
		HAL_ADC_Start(&hadc1);
		
		
		HAL_ADC_PollForConversion(&hadc1,100);
		flame_a0 =  HAL_ADC_GetValue(&hadc1);

		HAL_ADC_PollForConversion(&hadc1,100);
		CO_a1 =  HAL_ADC_GetValue(&hadc1);
		
		HAL_ADC_PollForConversion(&hadc1,100);
		smoke_a2 =  HAL_ADC_GetValue(&hadc1);
		
		HAL_ADC_Stop(&hadc1);
		
		countHeater = 0x00;
		HAL_GPIO_WritePin(C0_HTR_GPIO_Port,C0_HTR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GAS_HTR_GPIO_Port,GAS_HTR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SMOKE_HTR_GPIO_Port,SMOKE_HTR_Pin, GPIO_PIN_RESET);
		
		//Turn on counter for 30 Seconds.
		offCountHeater = 0x01;
	}
if(offHeaterTime >=30000U)
	{
		offCountHeater = 0x0;
		countHeater = 0x01;	
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
