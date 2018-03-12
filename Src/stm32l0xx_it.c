/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32l0xx_hal.h"
#include "stm32l0xx.h"
#include "stm32l0xx_it.h"

/* USER CODE BEGIN 0 */
#include "match_dsp.h"
#include "math.h"
#include "adc.h"
#include <string.h>

#define	N	16

// для 8 битного номера


#define ALERT_H 43690	
#define ALERT_L 21845
#define TEST_H  65280
#define TEST_L	255
#define REBOOT	0

extern uint32_t buff1;	// буфер

static float retf11,retf12,retf21,retf22,retfl, buff0, buffl0;	// фильтры


static uint8_t numbit = 0;

static uint16_t j, bin;//, binx;
extern uint16_t name;


uint16_t bit_1 = 0;
uint16_t bit_0 = 0;
uint16_t bit_n = 0;
uint16_t binn;

extern	uint16_t hpt_rept_cnt;
extern	uint8_t hpt_rept;
extern	uint8_t hpt_rept_type;
extern	uint8_t en_cnt;
extern 	uint8_t IRQ_abort;



extern	uint8_t databuff[];


extern uint8_t blink_type;
extern uint16_t blink_ext;
uint16_t blink_cnt = 0;
extern uint8_t blink_8sec;
extern uint8_t blink_trg;





_Bool frame = 0;
_Bool pass = 0;
_Bool det = 0;
_Bool numb[N];
_Bool numbn[N];


static uint8_t  nop = 0;


extern float history11[], history12[], history21[], history22[], historyl[];
extern SettingParametrs_t SETUP;
extern Cmd_Type CMD;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc;
extern LPTIM_HandleTypeDef hlptim1;
extern DMA_HandleTypeDef hdma_tim2_ch2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim21;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M0+ Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RCC global interrupt.
*/
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
* @brief This function handles EXTI line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel 1 interrupt.
*/
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel 2 and channel 3 interrupts.
*/
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  HAL_DMA_IRQHandler(&hdma_tim2_ch2);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel 4, channel 5, channel 6 and channel 7 interrupts.
*/
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 0 */

  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 1 */

  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 1 */
}


/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM21 global interrupt.
*/
void TIM21_IRQHandler(void)
{
  /* USER CODE BEGIN TIM21_IRQn 0 */
	
//if(blink_cnt>0)
//		blink_cnt++;
//	if((blink_cnt>25600))
//	{
//		blink_cnt = 1;
//		blink_8sec++;
//		if(blink_8sec <= 4)
//		{
//			blink(blink_type);
//		}
//		else
//		{
//			blink_8sec = 0;
//			blink_cnt = 0;
//		}
//	}
	
	if(!blink_trg)
	{
		if((blink_ext < 3200))
			blink_ext++;
		if(blink_ext == 3200)
			HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_RESET);
	}
	

	if(en_cnt)
	{
		switch(hpt_rept_cnt)
		{
			case 624:			// подтверждение
				if(hpt_rept_type == CONFIRM)
				{
					HAL_GPIO_WritePin(HPT_Answer_OUT_GPIO_Port,HPT_Answer_OUT_Pin, GPIO_PIN_RESET);
					en_cnt = 0;
					HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);						// включение прерываний по входу НРТ
				}
				break;
			case 1264: //1264:		// запрос		
				if(hpt_rept_type == REQUEST)
				{
					HAL_GPIO_WritePin(HPT_Answer_OUT_GPIO_Port,HPT_Answer_OUT_Pin, GPIO_PIN_RESET);
					HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);						// включение прерываний по входу НРТ
					IRQ_abort = 0;
				}
				break;
			case 18500:		// запрос
				if(hpt_rept_type == REQUEST)
				{
					IRQ_abort = 1;
					en_cnt = 0;
				}
				break;
			case 1888: //1888:		// проверка
				if(hpt_rept_type == TEST)
				{
					HAL_GPIO_WritePin(HPT_Answer_OUT_GPIO_Port,HPT_Answer_OUT_Pin, GPIO_PIN_RESET);
					en_cnt = 0;
					HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);						// включение прерываний по входу НРТ
				}
				break;
			case 32000:		// запрос при включении
				if(hpt_rept_type == ENABLE)
					HPT_Transmite(REQUEST);
				break;
		}
		hpt_rept_cnt++;
		
		if(!IRQ_abort)
			S_UART();
	}
	
	
	
//	if(!blink_trg)
//	{
//		if((blink_ext < 3200))
//			blink_ext++;
//		if(blink_ext == 3200)
//			HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_RESET);
//	}
	

	buff0 = 3*((float)buff1)/65520; //65535;							// ?????? ? ?????? 
	
	
	// cheby 2
	retf11 = IIR_SOS(buff0,SETUP.cf1s1,history11);			//f1
		retf12 = IIR_SOS(retf11,SETUP.cf1s2,history12);
	
	retf21 = IIR_SOS(buff0,SETUP.cf2s1,history21);			//f2
		retf22 = IIR_SOS(retf21,SETUP.cf2s2,history22);
	
		buffl0 = fabsf(retf12)-fabsf(retf22);
		retfl  = IIR_SOS(buffl0,SETUP.cflp,historyl);
	

		switch(CMD)
		{
			case START_DTR_FLP:
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&retfl,sizeof(retfl));
			break;
			case START_DTR_F1:
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&retf12,sizeof(retf12));
			break;
			case START_DTR_F2:
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&retf22,sizeof(retf22));
			break;
			case START_DTR_IN:
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&buff0,sizeof(buff0));
			break;
			default:
				break;
		}

		
	if(!det)
  {
		if ((retfl >= SETUP.ratio)||(retfl <= -SETUP.ratio))
			{
				j=0;
        det=1;
      }
  }
  else
	{
		if(j < SETUP.samplenum)
    {
			if(retfl >= SETUP.ratio)
				bit_1++;
			if(retfl <= -SETUP.ratio)
				bit_0++;
			if((retfl < SETUP.ratio)&&(retfl > -SETUP.ratio))
				bit_n++;
			
    }
    if(j == SETUP.samplenum)
		{
			if(numbit < N)
			{	
				if((bit_1 > bit_0)&&(bit_1 > bit_n)&&(bit_1 > (SETUP.samplenum/2)))
					{
						bin = bin<<1;
						bin = bin|1;
					//	binx = dataBuff(1);
					//	numb[numbit] = 1;
						numbit++;
					}
				if((bit_0 > bit_1)&&(bit_0 > bit_n)&&(bit_0 > (SETUP.samplenum/2)))
					{
						bin = bin<<1;
						bin = bin|0;
					//	binx = dataBuff(0);
					//	numb[numbit] = 0;
						numbit++;
					}
				if((bit_n > bit_0)&&(bit_n > bit_1))
					{
						nop++;
						if((nop <= 0)&&(bit_n > (SETUP.samplenum/2)))
							{
								//bin = bin<<1;
								//bin = bin|0;
								//numb[numbit] = 0;
						//		binx = dataBuff(0);
								numbit++;
							}
						else
							{
								bit_1 = 0;
								bit_0 = 0;
								bit_n = 0;
								nop = 0;
								det=0;
								j=0;
								bin=0;
								numbit = 0;
							//	memset(numb,0,sizeof(numb));
							}
					}
			
			}
			
			bit_1 = 0;
			bit_0 = 0;
			bit_n = 0;
			j=0;
			
		}
  }
  j++;
	
	if( numbit == N )
	{
		
		binn = bin;
	
		if(bin == SETUP.hpt_name)
		{
			blink_cnt = 1;
			blink(PERSONAL);
		}
		if((bin == ALERT_H)||(bin == ALERT_L))
		{
			blink_cnt = 1;
			blink(ALARM);
		}
		if((bin == TEST_H)||(bin == TEST_L))
		{
			HPT_Transmite(TEST);
		}
		/*if(bin == REBOOT)
			SCB->AIRCR = 0x05FA0004;*/
		
		det=0;
		bin=0;
		numbit = 0;
	}
  /* USER CODE END TIM21_IRQn 0 */
  HAL_TIM_IRQHandler(&htim21);
  /* USER CODE BEGIN TIM21_IRQn 1 */

  /* USER CODE END TIM21_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
