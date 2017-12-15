/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
//DALI STATES
#define SENDING_DATA 1

//pin states
#define DALI_START_BIT_PULSE 0
#define DALI_END_BIT_PULSE 1
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// MANCHESTER ENCODING VARIABLES
/* Half-bit period T value in usec(microsecond)s*/
/* static uint32_t T = 416; */

//MASTER DATA SENDING VARIABLES
/* This array represents the forward frame with address and cmd bytes */
unsigned char dali_master_array_cmd[17] = {};

/* /\* This array represents the received response from slave *\/ */
volatile unsigned char dali_master_array_receive_buffer[9] = {};

// uncertain
unsigned char ballastAddr = 0xD5;
unsigned char cmd = 0x56;

// actual and former value variables
unsigned char actual_val;
unsigned char former_val;

unsigned char dali_state;

// MANCHESTER DECODING VARIABLES 

/* This variable designates first interrupt for Manchester-Decoding*/
unsigned char start_timer = 0;

/* This variable is used to measure the ticks of timer2 */
int tick_count = 0;

unsigned char bit_count = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void DALI_Master_Sending_Data(void);
void PrepareDataToSend(unsigned char *commandArray, unsigned char *tx_array, unsigned char bytesInCmd);
void DALI_Send_Cmd(unsigned char ballastAddr, unsigned char cmd);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);
  /* HAL_Delay(1); */
  if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
  // using DALI_Send_Cmd function modified!! without typeofCmd and followingType variables)
  DALI_Send_Cmd(ballastAddr, cmd);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
    
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 103;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED_Green_Pin|LED_Blue_Pin|LED_Yellow_Pin|LED_Red_Pin 
                          |Manch_Tx_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Green_Pin LED_Blue_Pin LED_Yellow_Pin LED_Red_Pin 
                           Manch_Tx_Pin */
  GPIO_InitStruct.Pin = LED_Green_Pin|LED_Blue_Pin|LED_Yellow_Pin|LED_Red_Pin 
                          |Manch_Tx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : Manch_Rx_Pin */
  GPIO_InitStruct.Pin = Manch_Rx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Manch_Rx_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/* This function is used together with PrepareDatatoSend fucntion to manchester encode the ballastAddr and cmd values */
void DALI_Send_Cmd(unsigned char ballastAddr, unsigned char cmd)
{
  unsigned char data_array[2] = {};
  /* unsigned char i; */

  // Set Manch_Tx pin as high
  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);

  // reset tick_count and bit_count values
  tick_count = 0;
  bit_count = 0;

  // fetch ballast address and command
  data_array[0] = (char)ballastAddr;
  data_array[1] = (char)cmd;
  PrepareDataToSend(data_array, dali_master_array_cmd, 2);
  // set DALI state to send DATA
  dali_state = SENDING_DATA;
    
}
void PrepareDataToSend(unsigned char *commandArray, unsigned char *tx_array, unsigned char bytesInCmd)
{
  // set default value for the mask
  unsigned char mask = 0x80;
  // variable which hold one byte value - one element from commandArray
  unsigned char dummy;

  // number of bytes in command
  unsigned char bytes_counter;
  unsigned char i;
  // number of active bit
  unsigned char bitCounter = 0;

  for (i = 0; i < 9; i++)
    {
      tx_array[0] = 0;
    }

  // loop through all bytes in commandArray
  for(bytes_counter = 0; bytes_counter < bytesInCmd; bytes_counter++)
    {
      // assign byte for use
      dummy = commandArray[bytes_counter];
      // set mask to default value
      mask = 0x80;
      // increment number of active bit
      bitCounter++;
      // check if active bit is the first one
      if(bitCounter == 1)
	{
	  // Start bit is always 1 - in manchester that is END_BIT_PULSE
	  tx_array[0] = DALI_END_BIT_PULSE;
	}

      // 2 byte command
      for(i = 1; i < 9; i++)
	{
	  // check if bit is one
	  if(dummy & mask)
	    {
	      // assign pulse value
	      tx_array[i + (8 * bytes_counter)] = DALI_END_BIT_PULSE;
	    }
	  else
	    {
	      tx_array[i + (8 * bytes_counter)] = DALI_START_BIT_PULSE;
	    }
	  // check mask value
	  if(mask == 0x01)
	    {
	      mask <<= 7; // shift mask bit to MSB
	    }
	  else
	    {
	      mask >>= 1; // shift mask bit to 1 right
	    }
	}
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(dali_state == SENDING_DATA)
    {
      DALI_Master_Sending_Data();
    }
}

/* void DALI_Master_Receiving_Data(void) */
/* { */
/*   // Yet to configure */

/* } */
/* Included start_bit as 1 */
void DALI_Master_Sending_Data(void)
{
  unsigned char pulsePosition = 0;
  if(tick_count < 8)
    {
      if(tick_count < 4)
	{
          HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_RESET);
	}
      else
	{
	  HAL_GPIO_WritePin(LED_Yellow_GPIO_Port, LED_Yellow_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);
	}
    }
  else if(bit_count < 17)
    {
      if(tick_count % 4 == 0)
  	{
  	  pulsePosition = tick_count / 4;
  	  if(pulsePosition % 2 == 0)
  	    {
	      if(dali_master_array_cmd[bit_count] == 0)
  		{
  		  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);
  		}
  	      else
  		{
  		  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_RESET);
  		}
  	    }
  	  else
  	    {
  	      if(dali_master_array_cmd[bit_count] == 0)
  		{
  		  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_RESET);
  		}
  	      else
  		{
  		  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);
  		}
  	    }
  	}
    }
  // increment tick_count
  tick_count++;

  if(tick_count % 8 == 0)
    {
      bit_count++;
    }

  if(bit_count > 16)
    {
      HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);
      if(HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK)
  	{
  	  _Error_Handler(__FILE__, __LINE__);
  	}
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  
  /* Turn LED Yellow on */
  HAL_GPIO_WritePin(LED_Yellow_GPIO_Port, LED_Yellow_Pin, GPIO_PIN_SET);

  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  _Error_Handler(__FILE__, __LINE__);
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
