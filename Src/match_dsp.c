
#include	"stm32l0xx_hal.h"
//#include	"math.h"
#include	"match_dsp.h"
#include <string.h>



/* Table of CRC values for high–order byte */
    const static unsigned char auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
    };
    /* Table of CRC values for low–order byte */
    const static char auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
    };

		
		


float history11[] = {0,0},history12[] = {0,0},history21[] = {0,0},history22[] = {0,0},historyl[] = {0,0};


													//			a1					a2					b0					b1						b2



const uint32_t pdat0[]		= {0,125,125,125,0};
const uint32_t pdat2[]		= {0,250,125,250,0};
													//{125,125,125,125,0,250,0,250,0,250,0,250,125,125,125,125,125,0,250,0,250,0,250,125,125,125,125,125,0,250,0,250,0,250,125,125,125,125,125,0,0};	
const uint32_t pdat[]			= {125,125,125,125,125,0,250,0,250,0,250,0,125,125,125,125,125,250,0,250,0,250,0,125,125,125,125,125,250,0,250,0,250,0,125,125,125,125,125,0,0};	// лампа
const uint32_t pdat1[]		= {250,0,250,0,250,0,250,250,250,0,250,0,250,0,250,0,250,250,250,0,250,0,250,0,250,0,250,250,250,0,250,0,250,0,250,0,0};

_Bool blink_trg = 0;
_Bool blink_end = 0;
uint8_t blink_type = 0;
uint8_t blink_8sec;
uint16_t blink_ext;
extern DMA_HandleTypeDef hdma_tim2_ch1;
	


SettingParametrs_t SETUP;
UART2Recv_t UART2_RecvType;
Cmd_Type CMD;
Cmd_Type CMD_Rept;

extern UART_HandleTypeDef huart2;
// режим моргания 
void blink(char mode)
{
	uint32_t temp;
	temp = GPIOA->MODER;
	temp &= ~(GPIO_MODER_MODE0 << 0);
	temp |= ((GPIO_MODE_AF_PP & ((uint32_t)0x00000003U)) << 0);
	GPIOA->MODER = temp;
	
	switch(mode)
	{
		case START:
			blink_trg = 1;
			HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_1, (uint32_t *)&pdat0, sizeof(pdat0)/sizeof(uint32_t));
		break;
		case ALARM:
			blink_type = ALARM;
			blink_trg = 1;
			HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_1);
			if(blink_8sec == 0)
				HAL_GPIO_WritePin(HPT_Answer_OUT_GPIO_Port,HPT_Answer_OUT_Pin, GPIO_PIN_RESET);
			HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_1, (uint32_t *)&pdat, sizeof(pdat)/sizeof(uint32_t));
		break;
		case PERSONAL:
			blink_trg = 1;
			blink_type = PERSONAL;
			HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_1);
			if(blink_8sec == 0)
				HAL_GPIO_WritePin(HPT_Answer_OUT_GPIO_Port,HPT_Answer_OUT_Pin, GPIO_PIN_RESET);
			HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_1, (uint32_t *)&pdat1, sizeof(pdat1)/sizeof(uint32_t));
		break;
		case OK_SET:
			blink_trg = 1;
			HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET);
			HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_1);
			HAL_TIM_PWM_Start_DMA(&htim2,TIM_CHANNEL_1, (uint32_t *)&pdat2, sizeof(pdat2)/sizeof(uint32_t));
		break;
					
	}

}


float iir_test(float in, float *coef, float *his)
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


unsigned int crc_calculating(unsigned char *puchMsg, unsigned short usDataLen)
{
        unsigned char uchCRCHi = 0xFF ;             /* Инициализация последнего байта CRC  */
        unsigned char uchCRCLo = 0xFF ;             /* Инициализация первого байта CRC   */
        unsigned uIndex ;                           /* will index into CRC lookup table  */
        while (usDataLen--)                         /* pass through message buffer  */
        {
        uIndex = uchCRCHi ^ *puchMsg++ ;            /* Расчёт CRC */
        uchCRCLo = uchCRCLo ^ auchCRCHi[uIndex] ;
        uchCRCHi = auchCRCLo[uIndex] ;
        }
        return (uchCRCHi << 8 | uchCRCLo) ;
}

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
					
				if(GPIOA->IDR & GPIO_PIN_1)
				{
					blink_ext = 0;
					HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_RESET);		// включение большого света
					temp = GPIOA->MODER;																														//
					temp &= ~(GPIO_MODER_MODE0 << 0);																								//	перенастройка выхода с таймера на GPIO
					temp |= ((GPIO_MODE_OUTPUT_PP & ((uint32_t)0x00000003U)) << 0);									//
					GPIOA->MODER = temp;																														//
					
					GPIOA->ODR 	|= GPIO_PIN_0;																											//	включение малого света
				}
				else
				{
					if(blink_end)
						blink_end = 0;
					else
						HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET);		// выключение большого света
						
					temp = GPIOA->MODER;																														//
					temp &= ~(GPIO_MODER_MODE0 << 0);																								//	перенастройка выхода с таймера на GPIO
					temp |= ((GPIO_MODE_OUTPUT_PP & ((uint32_t)0x00000003U)) << 0);									//
					GPIOA->MODER = temp;																														//
					
					GPIOA->ODR 	&= ~GPIO_PIN_0;																											//	выключение малого света
				}
			}
				break;
			
		}
	}
	
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(blink_trg)
	{
		if(GPIOA->IDR & GPIO_PIN_0)
			HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET);
	}//if(!blink_trg)
	//	HAL_GPIO_WritePin(Interrupt_OUT_GPIO_Port,Interrupt_OUT_Pin, GPIO_PIN_RESET);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	blink_end = 1;
	HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_1);
	HAL_GPIO_WritePin(HPT_Answer_OUT_GPIO_Port,HPT_Answer_OUT_Pin, GPIO_PIN_SET);
	blink_trg = 0;
}

	/*
			SETUP.cf1s1[0] = 0.637533889133465;
		SETUP.cf1s1[1] =  0.997270126915337;
		SETUP.cf1s1[2] =	0.0685673070457099;
		SETUP.cf1s1[3] = 0.0517152562295210;
		SETUP.cf1s1[4] =  0.0685673070457099;
		
		SETUP.cf1s2[0] = 0.642709230388310;
		SETUP.cf1s2[1] =  0.997272648542179;
		SETUP.cf1s2[2] =	0.0149389164695805;
		SETUP.cf1s2[3] = 0.00781299255819078;
		SETUP.cf1s2[4] =  0.0149389164695805;
		
		SETUP.cf2s1[0] = 0.704087877691119;
		SETUP.cf2s1[1] =  0.997269978139115;
		SETUP.cf2s1[2] =	0.0668224706024191;
		SETUP.cf2s1[3] = 0.0394794137842006;
		SETUP.cf2s1[4] =  0.0668224706024192;
		
		SETUP.cf2s2[0] = 0.709198240016098;
		SETUP.cf2s2[1] =  0.997272797318793;
		SETUP.cf2s2[2] =	0.0158081555248921;
		SETUP.cf2s2[3] =  0.0129491034326102;
		SETUP.cf2s2[4] =  0.0158081555248921;
		
	
	*/
//	[0]	a1
//	[1]	a2
//	[2]	b0
//	[3]	b1
//	[4]	b2
	
	void DefaultSettings(void)
	{
		SETUP.name = 43690;			// 1010 1010 1010 1010
		SETUP.ratio = 5e-3f;
 SETUP.f1 =  984;
 SETUP.f2 =  960;
 SETUP.BR =   10;
 // f =  984 , BR = 1.0
/*a1*/   SETUP.cf1s1[0] = 0.704087877691286;
/*a2*/   SETUP.cf1s1[1] = 0.997269978139192;
/*b1*/   SETUP.cf1s1[2] = 0.069981351162423;
/*b2*/   SETUP.cf1s1[3] = 0.057324572315164;
/*b3*/   SETUP.cf1s1[4] = 0.069981351162423;

/*a1*/   SETUP.cf1s2[0] = 0.709198240015931;
/*a2*/   SETUP.cf1s2[1] = 0.997272797318719;
/*b1*/   SETUP.cf1s2[2] = 0.015094592920749;
/*b2*/   SETUP.cf1s2[3] = 0.008918043203879;
/*b3*/   SETUP.cf1s2[4] = 0.015094592920749;

 // f =  960 , BR = 1.0
/*a1*/   SETUP.cf2s1[0] = 0.615168140978587;
/*a2*/   SETUP.cf2s1[1] = 0.997270175699907;
/*b1*/   SETUP.cf2s1[2] = 0.068216336626507;
/*b2*/   SETUP.cf2s1[3] = 0.049959917153119;
/*b3*/   SETUP.cf2s1[4] = 0.068216336626507;

/*a1*/   SETUP.cf2s2[0] = 0.620363710782997;
/*a2*/   SETUP.cf2s2[1] = 0.997272599757482;
/*b1*/   SETUP.cf2s2[2] = 0.014673871821915;
/*b2*/   SETUP.cf2s2[3] = 0.007340481901426;
/*b3*/   SETUP.cf2s2[4] = 0.014673871821915;

 // flp = 1.00 , BR = 1.0
/*a1*/   SETUP.cflp[0] = -0.998038429728411;
/*a2*/   SETUP.cflp[1] = 0.000000000000000;
/*b1*/   SETUP.cflp[2] = 0.000980785135794;
/*b2*/   SETUP.cflp[3] = 0.000980785135794;
/*b3*/   SETUP.cflp[4] = 0.000000000000000;

 // BR = 1.0
SETUP.samplenum =  3200;

		/*SETUP.f1 = 984;
		SETUP.f2 = 966;
		SETUP.BR = 20;*/
		 // f =  984 , BR = 2.0
/*a1*/   //SETUP.cf1s1[0] = 0.702541857155492;
/*a2*/   //SETUP.cf1s1[1] = 0.994534406218589;
/*b1*/   //SETUP.cf1s1[2] = 0.071590504732271;
/*b2*/   //SETUP.cf1s1[3] = 0.066341650450648;
/*b3*/   //SETUP.cf1s1[4] = 0.071590504732271;

/*a1*/   //SETUP.cf1s2[0] = 0.712765626378700;
/*a2*/   //SETUP.cf1s2[1] = 0.994545730434914;
/*b1*/   //SETUP.cf1s2[2] = 0.014814725949727;
/*b2*/   //SETUP.cf1s2[3] = 0.006985641330531;
/*b3*/   //SETUP.cf1s2[4] = 0.014814725949727;

 // f =  966 , BR = 2.0
/*a1*/   //SETUP.cf2s1[0] = 0.635860005955117;
/*a2*/   //SETUP.cf2s1[1] = 0.994535004235001;
/*b1*/   //SETUP.cf2s1[2] = 0.069094363053546;
/*b2*/   //SETUP.cf2s1[3] = 0.059679724517278;
/*b3*/   //SETUP.cf2s1[4] = 0.069094363053546;

/*a1*/   //SETUP.cf2s2[0] = 0.646214193388075;
/*a2*/   //SETUP.cf2s2[1] = 0.994545132412054;
/*b1*/   //SETUP.cf2s2[2] = 0.014722092858333;
/*b2*/   //SETUP.cf2s2[3] = 0.005930783953626;
/*b3*/   //SETUP.cf2s2[4] = 0.014722092858333;

 // flp = 2.00 , BR = 2.0
/*a1*/   //SETUP.cflp[0] = -0.996080699674494;
/*a2*/   //SETUP.cflp[1] = 0.000000000000000;
/*b1*/   //SETUP.cflp[2] = 0.001959650162753;
/*b2*/   //SETUP.cflp[3] = 0.001959650162753;
/*b3*/   //SETUP.cflp[4] = 0.000000000000000;

 // BR = 2.0
//SETUP.samplenum =  1600;

	}
	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		
		if (UART2_RECV_CMD == UART2_RecvType)
		{
				HAL_TIM_Base_Stop_IT(&htim21);			
			//UART2_RecvType = UART2_RECV_VALUE;
		switch(CMD)
		{
			case NUMBER:								// номер метки
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.name,sizeof(SETUP.name));
				break;
			case BODRATE:							// бодрейт
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.BR,sizeof(SETUP.BR));
				break;
			case REPEATNUM:							// количество повторов
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.name,sizeof(SETUP.name));
				break;
			case F1S1:								// частота 1 секция 1
				HAL_TIM_Base_Stop_IT(&htim21);
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cf1s1,sizeof(SETUP.cf1s1));
				break;
			case F1S2:								// частота 1 секция 2
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cf1s2,sizeof(SETUP.cf1s2));
				break;
			case F2S1:								// частота 2 секция 1
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cf2s1,sizeof(SETUP.cf2s1));
				break;
			case F2S2:								// частота 2 секция 2
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cf2s2,sizeof(SETUP.cf2s2));
				break;
			case FLP:								// частота ФНЧ
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cflp,sizeof(SETUP.cflp));
				break;
			case DETECTION_THRESHOLD:				// порог обнаружения
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.ratio,sizeof(SETUP.ratio));
				break;
			case START_DATA_TRANSMIT_FLP:				// запуск передачи данных
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case START_DATA_TRANSMIT_F1:				// запуск передачи данных
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case START_DATA_TRANSMIT_F2:				// запуск передачи данных
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case START_DATA_TRANSMIT_IN:				// запуск передачи данных
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case STOP_DATA_TRANSMIT:					// остановка передачи данных
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_F1:
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.f1,sizeof(SETUP.f1));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_F2:
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.f2,sizeof(SETUP.f2));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_BR:
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.BR,sizeof(SETUP.BR));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_NAME:
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.name,sizeof(SETUP.name));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_SETUP:
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP,sizeof(SETUP));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case CONNECT:
				UART2_RecvType = UART2_RECV_CMD;
				CMD_Rept = OK;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&CMD_Rept,sizeof(CMD_Rept));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case F1:							// f1
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.f1,sizeof(SETUP.f1));
				break;
			case F2:							// f2
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.f2,sizeof(SETUP.f2));
				break;
			case SAMPLENUM:							// f2
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.samplenum,sizeof(SETUP.samplenum));
				break;
			default:
				
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
		/*HAL_TIM_Base_Stop_IT(&htim21);
		
		if(rxDATA.f1 != 0)
			f1 = rxDATA.f1;
		
		if(rxDATA.f2 != 0)
			f2 = rxDATA.f2;
		
		if(rxDATA.br != 0)
			BR = rxDATA.br;
		
		if(rxDATA.name != 0)
			name = rxDATA.name;
		
		if(rxDATA.res == 1)
			SCB->AIRCR = 0x05FA0004;
			
		
		SetCoefficiens(f1,f2,BR);
		SaveSetting();
		
		HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxDATA,sizeof(rxDATA));
		HAL_TIM_Base_Start_IT(&htim21);*/
	}
