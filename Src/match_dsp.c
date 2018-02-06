
#include	"stm32l0xx_hal.h"
#include "adc.h"
#include	"match_dsp.h"
#include <string.h>






float history11[] = {0,0},history12[] = {0,0},history21[] = {0,0},history22[] = {0,0},historyl[] = {0,0};

#define DATALEN	16
#define INVERT_HALFWORD(a)   ((a&1)<<15) | ((a&2)<<13) | ((a&4)<<11) | ((a&8)<<9) | ((a&16)<<7) | ((a&32)<<5) | ((a&64)<<3) | ((a&128)<<1) | ((a&256)>>1) | ((a&512)>>3) | ((a&1024)>>5) | ((a&2048)>>7) | ((a&4096)>>9) | ((a&8192)>>11) | ((a&16384)>>13) | ((a&32768)>>15)




uint8_t alarm[] = {1,0,1,0, 1,0,1,0, 1,0,1,0, 1,0,1,0};
													//			a1					a2					b0					b1						b2



const uint32_t pdat0[]		= {0,125,125,125,0};	// старт
const uint32_t pdat2[]		= {0,250,125,250,0}; // прошивка

													//{125,125,125,125,0,250,0,250,0,250,0,250,125,125,125,125,125,0,250,0,250,0,250,125,125,125,125,125,0,250,0,250,0,250,125,125,125,125,125,0,0};	
const uint32_t pdat[]			= {125,125,125,125,125,0,250,0,250,0,250,0,125,125,125,125,125,250,0,250,0,250,0,125,125,125,125,125,250,0,250,0,250,0,125,125,125,125,125,0,0};	// авария
const uint32_t pdat1[]		= {250,0,250,0,250,0,250,250,250,0,250,0,250,0,250,0,250,250,250,0,250,0,250,0,250,0,250,250,250,0,250,0,250,0,250,0,0};													// персональный


_Bool blink_end = 0;
	
uint8_t blink_type = 0;		// тип моргания
uint8_t blink_8sec;				// количество 8-ми секундных ожиданий
uint16_t blink_ext;				// внешний импульс
uint8_t blink_trg;	
extern DMA_HandleTypeDef hdma_tim2_ch1;

	
	
	
	uint8_t hpt_rept = 0;		

	uint8_t rx_buff_cnt = 0;





	
uint8_t databuff[DATALEN];
uint8_t SoftUart[TXBUFF];

#define TB 40
uint8_t UartBuffByte[TB];			// start 8data stop 
//uint8_t delay[4][8] = {{ 25, 35, 25, 35, 25, 35, 25, 35},					// 0 - 115200
//											 { 50, 70, 50, 70, 50, 70, 50, 70},					// 1 - 57600
//											 { 50,120, 75,120, 75,120, 75,120},					// 2 - 38400
//											 { 30,110, 75,110, 75,110, 75,110}					// 3 - 36000
//											};
//uint8_t br = 2;				
//											
											
											
	

	uint8_t hpt_rept_type = ENABLE; // тип запроса НРТ
	uint16_t hpt_rept_cnt;		// счетчик для ответа НРТ
	uint8_t IRQ_abort = 1;		// 0 - ждем уарт, 1 - ждем моргание
	uint8_t en_cnt = 0;				// запуск счета 1 - запущен, 0 - остановлен

SettingParametrs_t SETUP;
UART2Recv_t UART2_RecvType;
UART2_Queue_Data UART2_Trans_Data;

Cmd_Type CMD;
Cmd_Type CMD_Rept;

extern UART_HandleTypeDef huart2;
extern LPTIM_HandleTypeDef hlptim1;

// режим моргания 
void blink(char mode)
{
	uint32_t temp;
	temp = CAPLAMP_OUT1_GPIO_Port->MODER;
	temp &= ~(GPIO_MODER_MODE1 << 0);
	temp |= ((GPIO_MODE_AF_PP & ((uint32_t)0x00000003U)) << 2);
	CAPLAMP_OUT1_GPIO_Port->MODER = temp;
	
	switch(mode)
	{
		case START:
			blink_trg = 1;
			HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_2, (uint32_t *)&pdat0, sizeof(pdat0)/sizeof(uint32_t));
		break;
		case ALARM:
			blink_trg = 1;
			blink_type = ALARM;
			HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_2);
			HPT_Transmite(CONFIRM);
			HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_2, (uint32_t *)&pdat, sizeof(pdat)/sizeof(uint32_t));
		break;
		case PERSONAL:
			blink_trg = 1;
			blink_type = PERSONAL;
			HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_2);
			HPT_Transmite(CONFIRM);
			HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_2, (uint32_t *)&pdat1, sizeof(pdat1)/sizeof(uint32_t));
		break;
		case OK_SET:
			blink_trg = 1;
			HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_2);
			HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_2, (uint32_t *)&pdat2, sizeof(pdat2)/sizeof(uint32_t));
		break;
	}

}


float IIR_SOS(float in, float *coef, float *his)
{
		float out;
    float new_his;
    float *hist1_ptr, *hist2_ptr;
    float *coef_ptr;
    float hist1, hist2;
    coef_ptr = coef;

    hist1_ptr = his;
    hist2_ptr = hist1_ptr+1;
	
	
		hist1 = *hist1_ptr;
    hist2 = *hist2_ptr;
	
		out = in  - hist1 * (*coef_ptr++);
		new_his = out - hist2 * (*coef_ptr++);
	
		out = new_his * (*coef_ptr++);
		out = out + hist1 * (*coef_ptr++);
    out = out + hist2 * (*coef_ptr++);

    *hist2_ptr = *hist1_ptr;
    *hist1_ptr = new_his;

    return out;
}
void S_UART(void)
{
	__disable_irq();
	
	uint16_t j;
	uint8_t i;
//	HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET);
//	Interrupt_OUT2_GPIO_Port->BSRR = Interrupt_OUT2_Pin;
	

			for(j = 0; j < 30; j++)						// пауза
				__nop();
	
		for(i = 0; i < 19; i++)
		{
				UartBuffByte[i] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);

			for(j = 0; j < 100; j++)						// пауза
				__nop();

		}
		
		UartBuffByte[19] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);
		
		for(j = 0; j < 90; j++)						// пауза
				__nop();
		
		for(i = 20; i < 29; i++)
		{
				UartBuffByte[i] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);

			for(j = 0; j < 96; j++)						// пауза
				__nop();

		}
		
		UartBuffByte[29] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);
		
		for(j = 0; j < 75; j++)						// пауза
				__nop();
		
		for(i = 30; i < TB; i++)
		{
				UartBuffByte[i] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);

			for(j = 0; j < 96; j++)						// пауза
				__nop();

		}
//		Interrupt_OUT2_GPIO_Port->BRR = Interrupt_OUT2_Pin;
//		HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_RESET);
		
		if(!UartBuffByte[0] && UartBuffByte[9] && !UartBuffByte[10] && UartBuffByte[19] && !UartBuffByte[20] && UartBuffByte[29] && !UartBuffByte[30] && UartBuffByte[39])
		{			
		SoftUart[0] = UartBuffByte[ 1] | UartBuffByte[ 2]<<1 | UartBuffByte[ 3]<<2 | UartBuffByte[ 4]<<3 | UartBuffByte[ 5]<<4 | UartBuffByte[ 6]<<5 | UartBuffByte[ 7]<<6 | UartBuffByte[ 8]<<7 ;
		SoftUart[1] = UartBuffByte[11] | UartBuffByte[12]<<1 | UartBuffByte[13]<<2 | UartBuffByte[14]<<3 | UartBuffByte[15]<<4 | UartBuffByte[16]<<5 | UartBuffByte[17]<<6 | UartBuffByte[18]<<7 ;
		SoftUart[2] = UartBuffByte[21] | UartBuffByte[22]<<1 | UartBuffByte[23]<<2 | UartBuffByte[24]<<3 | UartBuffByte[25]<<4 | UartBuffByte[26]<<5 | UartBuffByte[27]<<6 | UartBuffByte[28]<<7 ;
		SoftUart[3] = UartBuffByte[31] | UartBuffByte[32]<<1 | UartBuffByte[33]<<2 | UartBuffByte[34]<<3 | UartBuffByte[35]<<4 | UartBuffByte[36]<<5 | UartBuffByte[37]<<6 | UartBuffByte[38]<<7 ;
		
		if(SoftUart[0] == 0x12U)
		SETUP.hpt_name = (SoftUart[1] << 8) | SoftUart[2];			// проверить порядок байт
		SaveSetting(&SETUP);
		IRQ_abort = 1;		// поменять на 1, убрать из майна, вернуть запрос ENABLE
		__enable_irq ();
		}
		else
		{
			__enable_irq ();
			HPT_Transmite(REQUEST);
		}
	
	
//	__enable_irq ();
}


// сейчас передача идет старшим вперед databuff[0] - старший
	// должна быть младшим вперед databuff[0] - младший
	//uint16_t dataBuff(uint8_t data)
	//{
	//	uint16_t databin;
		
		// кольцевой сдвиг
	//	for(uint8_t i = DATALEN-1; i > 0; i--)
	//	{
	//		databuff[i] = databuff[i-1];
	//	}
		// запись нового бита
	//	databuff[0] = data;
		
		// восстановление числа
		
//		databin = databuff[0] | databuff[1]<<1 | databuff[2]<<2 | databuff[3]<<3 | databuff[4]<<4 | databuff[5]<<5 | databuff[6]<<6 | databuff[7]<<7 | databuff[8]<<8 | databuff[9]<<9 | databuff[10]<<10 | databuff[11]<<11 | databuff[12]<<12 | databuff[13]<<13 | databuff[14]<<14 | databuff[15]<<15 ; // младшим вперед
	//	databin = databuff[15] | databuff[14]<<1 | databuff[13]<<2 | databuff[12]<<3 | databuff[11]<<4 | databuff[10]<<5 | databuff[9]<<6 | databuff[8]<<7 | databuff[7]<<8 | databuff[6]<<9 | databuff[5]<<10 | databuff[4]<<11 | databuff[3]<<12 | databuff[2]<<13 | databuff[1]<<14 | databuff[0]<<15 ; // младшим вперед
		
		
		// сравнение массивов
//		if(memcmp(&databuff, &alarm, sizeof(databuff)))
//			blink(ALARM);
		
	//	return databin;
	//}
	


void SaveSetting(SettingParametrs_t *Settings)
{
	uint32_t *flash_addr = (uint32_t *)ADR_START;
  uint32_t *settings_addr = (uint32_t *)Settings;
	
	uint32_t flash_addr_e = ADR_START;
  HAL_FLASHEx_DATAEEPROM_Unlock();
	HAL_FLASHEx_DATAEEPROM_Erase(flash_addr_e);
	
	for (uint8_t i = 0; i < sizeof(SETUP); i+=4) {
    HAL_FLASHEx_DATAEEPROM_Erase(flash_addr_e);
    flash_addr_e +=4;
  }
	

	
	for (uint32_t i = 0; i < SETTING_WORDS; i++) {
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)flash_addr, *(uint32_t *)settings_addr);
    flash_addr++;
    settings_addr++;
  }
	
	
	HAL_FLASHEx_DATAEEPROM_Lock();
	
}

void LoadSetting(SettingParametrs_t *Settings)
{
	
	uint32_t *flash_addr = (uint32_t *)ADR_START;
	uint32_t *settings_addr = (uint32_t *)Settings;
	
	memcpy((void *)settings_addr, (const void *)flash_addr, sizeof(SettingParametrs_t));
		
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		uint32_t temp;
		switch(GPIO_Pin)
		{
			case External_IN_Pin:							// ?????? ??????
				if(!blink_trg){
								
				if(External_IN_GPIO_Port->IDR & External_IN_Pin)																	// прерывание по фронту
				{
					if(!IRQ_abort)
//						HAL_LPTIM_Counter_Start_IT(&hlptim1,350);			//10 35000 кбод
						S_UART();

					if(IRQ_abort)
					{
						blink_ext = 0;
						HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_RESET); // включение большого света
					}
					
//					if(hpt_rept_type == ENABLE)
//					{
//						hpt_rept_cnt = 0;
//						en_cnt = 1;
//					}
					
					temp = CAPLAMP_OUT1_GPIO_Port -> MODER;																				  //
					temp &= ~(GPIO_MODER_MODE1 << 0);																								//	перенастройка выхода с таймера на GPIO
					temp |= ((GPIO_MODE_OUTPUT_PP & ((uint32_t)0x00000001U)) << 2);									//
					CAPLAMP_OUT1_GPIO_Port -> MODER = temp;																				  //
					
					CAPLAMP_OUT1_GPIO_Port -> ODR |= CAPLAMP_OUT1_Pin;														  //	включение малого света
				}
				else																																							// прерывание по спаду
				{
//					if(blink_end)
//						blink_end = 0;
//					else
					if(IRQ_abort)
						HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET); // выключение большого света
						
					

					temp = CAPLAMP_OUT1_GPIO_Port -> MODER;																				  //
					temp &= ~(GPIO_MODER_MODE1 << 0);																								//	перенастройка выхода с таймера на GPIO
					temp |= ((GPIO_MODE_OUTPUT_PP & ((uint32_t)0x00000001U)) << 2);									//
					CAPLAMP_OUT1_GPIO_Port -> MODER = temp;																				  //
					
					CAPLAMP_OUT1_GPIO_Port -> ODR &= ~CAPLAMP_OUT1_Pin;														  //	выключение малого света
				}
			}
				break;
			
		}
	}
	

void HPT_Transmite(uint8_t type)
{
	HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);							// отключение прерываний по входу НРТ
	hpt_rept_type = type;
	if(type == REQUEST)
		HAL_ADC_Stop_DMA(&hadc);
	hpt_rept_cnt = 0;
	en_cnt = 1;
	HAL_GPIO_WritePin(HPT_Answer_OUT_GPIO_Port,HPT_Answer_OUT_Pin, GPIO_PIN_SET);
	
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM21)
		if(blink_trg)
		{
			if(CAPLAMP_OUT1_GPIO_Port->IDR & CAPLAMP_OUT1_Pin)
				HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_RESET);
			else
				HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET);
		}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	blink_end = 1;
	blink_trg = 0;
	HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_2);
}

	
	void DefaultSettings(void)
	{
		SETUP.hpt_name = 61680;	// 1111 0000 1111 0000 43690;			// 1010 1010 1010 1010
		
		SETUP.repeatnum = 1;
		
		SETUP.serailnum	= 0x00000000U;
		SETUP.firmware	= 0x3117U;
		
		SETUP.ratio = 5e-3f;
		
		SETUP.f1 = 1025;
		SETUP.f2 = 1015;
		SETUP.BR =   20;
 // f = 1025 , BR = 2.0
/*a1*/   SETUP.cf1s1[0] = 0.851022195929615;
/*a2*/   SETUP.cf1s1[1] = 0.994532979119457;
/*b1*/   SETUP.cf1s1[2] = 0.072308525004472;
/*b2*/   SETUP.cf1s1[3] = 0.077052881495139;
/*b3*/   SETUP.cf1s1[4] = 0.072308525004472;

/*a1*/   SETUP.cf1s2[0] = 0.860901393554830;
/*a2*/   SETUP.cf1s2[1] = 0.994547157552342;
/*b1*/   SETUP.cf1s2[2] = 0.015160627875001;
/*b2*/   SETUP.cf1s2[3] = 0.009485054055582;
/*b3*/   SETUP.cf1s2[4] = 0.015160627875001;

 // f = 1015 , BR = 2.0
/*a1*/   SETUP.cf2s1[0] = 0.815279528023444;
/*a2*/   SETUP.cf2s1[1] = 0.994533336545261;
/*b1*/   SETUP.cf2s1[2] = 0.072438722516149;
/*b2*/   SETUP.cf2s1[3] = 0.074780562643580;
/*b3*/   SETUP.cf2s1[4] = 0.072438722516149;

/*a1*/   SETUP.cf2s2[0] = 0.825248777003602;
/*a2*/   SETUP.cf2s2[1] = 0.994546800121572;
/*b1*/   SETUP.cf2s2[2] = 0.015046644461024;
/*b2*/   SETUP.cf2s2[3] = 0.008853269634262;
/*b3*/   SETUP.cf2s2[4] = 0.015046644461024;

 // flp = 2.00 , BR = 2.0
/*a1*/   SETUP.cflp[0] = -0.996080699674494;
/*a2*/   SETUP.cflp[1] = 0.000000000000000;
/*b1*/   SETUP.cflp[2] = 0.001959650162753;
/*b2*/   SETUP.cflp[3] = 0.001959650162753;
/*b3*/   SETUP.cflp[4] = 0.000000000000000;

 // BR = 2.0
SETUP.samplenum =  1600;
		
//		 // f =  984 , BR = 2.0
//		SETUP.samplenum = 1600;
//		SETUP.f1 = 984;
//		SETUP.f2 = 966;
//		SETUP.BR = 20;
///*a1*/   SETUP.cf1s1[0] = 0.702541857155492;
///*a2*/   SETUP.cf1s1[1] = 0.994534406218589;
///*b1*/   SETUP.cf1s1[2] = 0.071590504732271;
///*b2*/   SETUP.cf1s1[3] = 0.066341650450648;
///*b3*/   SETUP.cf1s1[4] = 0.071590504732271;

///*a1*/   SETUP.cf1s2[0] = 0.712765626378700;
///*a2*/   SETUP.cf1s2[1] = 0.994545730434914;
///*b1*/   SETUP.cf1s2[2] = 0.014814725949727;
///*b2*/   SETUP.cf1s2[3] = 0.006985641330531;
///*b3*/   SETUP.cf1s2[4] = 0.014814725949727;

// // f =  966 , BR = 2.0
///*a1*/   SETUP.cf2s1[0] = 0.635860005955117;
///*a2*/   SETUP.cf2s1[1] = 0.994535004235001;
///*b1*/   SETUP.cf2s1[2] = 0.069094363053546;
///*b2*/   SETUP.cf2s1[3] = 0.059679724517278;
///*b3*/   SETUP.cf2s1[4] = 0.069094363053546;

///*a1*/   SETUP.cf2s2[0] = 0.646214193388075;
///*a2*/   SETUP.cf2s2[1] = 0.994545132412054;
///*b1*/   SETUP.cf2s2[2] = 0.014722092858333;
///*b2*/   SETUP.cf2s2[3] = 0.005930783953626;
///*b3*/   SETUP.cf2s2[4] = 0.014722092858333;

// // flp = 2.00 , BR = 2.0
///*a1*/   SETUP.cflp[0] = -0.996080699674494;
///*a2*/   SETUP.cflp[1] = 0.000000000000000;
///*b1*/   SETUP.cflp[2] = 0.001959650162753;
///*b2*/   SETUP.cflp[3] = 0.001959650162753;
///*b3*/   SETUP.cflp[4] = 0.000000000000000;

		

	}
	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		
		if (UART2_RECV_CMD == UART2_RecvType)
		{
				HAL_TIM_Base_Stop_IT(&htim21);
			
		switch(CMD)
		{
			case SET_NAME_HPT:								// номер метки
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.hpt_name,sizeof(SETUP.hpt_name));
				break;
			case SET_BODRATE:							// бодрейт
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.BR,sizeof(SETUP.BR));
				break;
			case SET_REPEATNUM:							// количество повторов
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.hpt_name,sizeof(SETUP.hpt_name));
				break;
			case SET_SAMPLENUM:							// коэффициент децимации
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.samplenum,sizeof(SETUP.samplenum));
				break;
			case SET_F1S1:								// частота 1 секция 1
				HAL_TIM_Base_Stop_IT(&htim21);
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cf1s1,sizeof(SETUP.cf1s1));
				break;
			case SET_F1S2:								// частота 1 секция 2
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cf1s2,sizeof(SETUP.cf1s2));
				break;
			case SET_F2S1:								// частота 2 секция 1
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cf2s1,sizeof(SETUP.cf2s1));
				break;
			case SET_F2S2:								// частота 2 секция 2
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cf2s2,sizeof(SETUP.cf2s2));
				break;
			case SET_FLP:								// частота ФНЧ
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cflp,sizeof(SETUP.cflp));
				break;
			case SET_DET_THRES:				// порог обнаружения
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.ratio,sizeof(SETUP.ratio));
				break;
			case SET_F1:							// f1
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.f1,sizeof(SETUP.f1));
				break;
			case SET_F2:							// f2
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.f2,sizeof(SETUP.f2));
				break;
			case SET_SN:							// установка серийного номера
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.serailnum,sizeof(SETUP.serailnum));
				break;
			case SET_FW:							// установка версии по
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.firmware,sizeof(SETUP.firmware));
				break;
			case START_DTR_FLP:				// запуск передачи данных
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case START_DTR_F1:				// запуск передачи данных
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case START_DTR_F2:				// запуск передачи данных
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case START_DTR_IN:				// запуск передачи данных
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case STOP_DTR:					// остановка передачи данных
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_SETUP:				// отправить структуру
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP,sizeof(SETUP));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_F1:					// отправить частоту 1
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.f1,sizeof(SETUP.f1));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_F2:					// отправить частоту 2
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.f2,sizeof(SETUP.f2));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_BR:					// отправить бодрейт
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.BR,sizeof(SETUP.BR));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_DET_THRES:					// отправить бодрейт
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.ratio,sizeof(SETUP.ratio));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_NAME_HPT:		// отправить номер HPT
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.hpt_name,sizeof(SETUP.hpt_name));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_SN:		// отправить серийный номер
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.serailnum,sizeof(SETUP.serailnum));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_FW:		// отправить версию ПО
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.firmware,sizeof(SETUP.firmware));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_LAST_CALL:		// отправить последний принятый номер HPT
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.hpt_name,sizeof(SETUP.hpt_name));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_LAST_CALL_TYPE:		// отправить последний тип вызова 
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.hpt_name,sizeof(SETUP.hpt_name));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case DEF_SETT:								// отправить ответ на запрос подключения
				UART2_RecvType = UART2_RECV_CMD;
				DefaultSettings();
				SaveSetting(&SETUP);
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case CONNECT:								// отправить ответ на запрос подключения
				UART2_RecvType = UART2_RECV_CMD;
				CMD_Rept = OK;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&CMD_Rept,sizeof(CMD_Rept));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			
			
			default:										// отправка ошибки на неправильный запрос
				UART2_RecvType = UART2_RECV_CMD;
				CMD_Rept = ERR;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&CMD_Rept,sizeof(CMD_Rept));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
		}}
		else{
			
			SaveSetting(&SETUP);
			blink(OK_SET);
			
			if(!(TIM21->CR1))
				HAL_TIM_Base_Start_IT(&htim21);
		
		UART2_RecvType = UART2_RECV_CMD;
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
	}
		
	}
	
	
		
//		for(j = 0; j < delay[br][0]; j++)						// пауза				задержка старт бита
//				__nop();
//	
//		for(i = 0; i < 9; i++)											// первый байт
//		{
//				UartBuffByte[i] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);

//			for(j = 0; j < delay[br][1]; j++)					// пауза
//				__nop();

//		}
//		
//		UartBuffByte[9] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);			// стоп бит 1й байт 
//		
//		
//		for(j = 0; j < delay[br][2]; j++)						// пауза				корректировка 
//				__nop();
//		
//		for(i = 10; i < 19; i++)										// второй байт
//		{
//				UartBuffByte[i] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);

//			for(j = 0; j < delay[br][3]; j++)					// пауза
//				__nop();

//		}
//		
//		UartBuffByte[19] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);			// стоп бит 2й байт 
//		
//		
//		for(j = 0; j < delay[br][4]; j++)						// пауза				корректировка
//				__nop();
//		
//		for(i = 20; i < 29; i++)										// третий байт
//		{
//				UartBuffByte[i] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);

//			for(j = 0; j < delay[br][5]; j++)					// пауза
//				__nop();

//		}
//		
//		UartBuffByte[29] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);			// стоп бит 3й байт 
//		
//		for(j = 0; j < delay[br][6]; j++)						// пауза				корректировка
//				__nop();
//		
//		for(i = 30; i < 40; i++)										// четвертый байт
//		{
//				UartBuffByte[i] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);

//			for(j = 0; j < delay[br][7]; j++)					// пауза
//				__nop();

//		}
//		
