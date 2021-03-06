
#include	"stm32l0xx_hal.h"
#include "adc.h"
#include	"match_dsp.h"
#include <string.h>





// ������ ��� ������������� �������� ���������������� �������
float history11[] = {0,0};
float history12[] = {0,0};
float history21[] = {0,0};
float history22[] = {0,0};
float historyl[]  = {0,0};

#define DATALEN	16	// ������ ������
uint8_t databuff[DATALEN];	//��������� �����

// �������� �����
#define INVERT_HALFWORD(a)   ((a&1)<<15) | ((a&2)<<13) | ((a&4)<<11) | ((a&8)<<9) | ((a&16)<<7) | ((a&32)<<5) | ((a&64)<<3) | ((a&128)<<1) | ((a&256)>>1) | ((a&512)>>3) | ((a&1024)>>5) | ((a&2048)>>7) | ((a&4096)>>9) | ((a&8192)>>11) | ((a&16384)>>13) | ((a&32768)>>15)



// ������ ���������� ������ ��� ���������� ��������
uint8_t alarm[] = {1,0,1,0, 1,0,1,0, 1,0,1,0, 1,0,1,0};
													//			a1					a2					b0					b1						b2

// ������� ��� ������� ���������� �������
const uint32_t pdat0[]		= {0,125,125,125,0};	// �����
const uint32_t pdat2[]		= {0,250,125,250,0}; // ��������

													//{125,125,125,125,0,250,0,250,0,250,0,250,125,125,125,125,125,0,250,0,250,0,250,125,125,125,125,125,0,250,0,250,0,250,125,125,125,125,125,0,0};	
const uint32_t pdat[]			= {125,125,125,125,125,0,250,0,250,0,250,0,125,125,125,125,125,250,0,250,0,250,0,125,125,125,125,125,250,0,250,0,250,0,125,125,125,125,125,0,0};	// ������
const uint32_t pdat1[]		= {250,0,250,0,250,0,250,250,250,0,250,0,250,0,250,0,250,250,250,0,250,0,250,0,250,0,250,250,250,0,250,0,250,0,250,0,0};													// ������������

// ���������� ��� �������� �������
_Bool blink_end = 0;			// ��������� ��������
uint8_t blink_type = 0;		// ��� ��������
uint8_t blink_8sec;				// ���������� 8-�� ��������� ��������
uint16_t blink_ext;				// ������� �������
uint8_t blink_trg;				// ������� ��������
	
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern uint32_t buff1;
	


uint8_t hpt_rept_type = ENABLE; // ��� ������� ���
uint16_t hpt_rept_cnt;		// ������� ��� ������ ���
uint8_t IRQ_abort = 1;		// 0 - ���� ����, 1 - ���� ��������
uint8_t en_cnt = 0;				// ������ ����� 1 - �������, 0 - ����������


SoftUART_15Baud_t SUART;
SettingParametrs_t SETUP;
UART2Recv_t UART2_RecvType;

Cmd_Type CMD;
Cmd_Type CMD_Rept;

extern UART_HandleTypeDef huart2;

// ����� �������� 
void blink(char mode)
{
	//������������ ���� ������ 
	uint32_t temp;
	temp = CAPLAMP_OUT1_GPIO_Port->MODER;
	temp &= ~(GPIO_MODER_MODE1 << 0);
	temp |= ((GPIO_MODE_AF_PP & ((uint32_t)0x00000003U)) << 2);
	CAPLAMP_OUT1_GPIO_Port->MODER = temp;
	
	// ����� ���� ��������
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

// ������������ ����� ���������� ����� ������� ������� 
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
	
	// ����������� �����
		out = in  - hist1 * (*coef_ptr++);				// �1
		new_his = out - hist2 * (*coef_ptr++);		// �2
	
	// ������������ �����
		out = new_his * (*coef_ptr++);						// b0
		out = out + hist1 * (*coef_ptr++);				// b1
    out = out + hist2 * (*coef_ptr++);				// b2

    *hist2_ptr = *hist1_ptr;
    *hist1_ptr = new_his;

    return out;
}

// ���������� ����������� UART
void S_UART(void)
{
	if(SUART.rx_cnt == 0)		// ����� ������� ����
	{

		if(SUART.tim_cnt == 105)			// �������� ��������� ����� ��� ����
		{
			SUART.rx_buff[SUART.rx_cnt] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);			// ������ ��������� �����
			SUART.rx_cnt++;
			SUART.tim_cnt = 0;
		}
	}
	else if(SUART.rx_cnt < 30)		// ����� ���������� ���
	{
		if(SUART.tim_cnt == 210)		// ������ ���� ����� 65,5 ��
		{
			SUART.rx_buff[SUART.rx_cnt] = !(External_IN_GPIO_Port->IDR & External_IN_Pin);	// ������ ��������� ����
			SUART.rx_cnt++;
			SUART.tim_cnt = 0;
		}
	}
	else if(SUART.rx_cnt == 30)		// ��������� ���� ������� ��� ����
	{
			IRQ_abort = 1;		// �������� �� 1, ������ �� �����, ������� ������ ENABLE
			SUART.tim_en = 0;
			SUART.tim_cnt = 0;
			SUART.rx_cnt = 0;
			HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);			// ��������� ����������

		if(!SUART.rx_buff[0] && SUART.rx_buff[9] && !SUART.rx_buff[10] && SUART.rx_buff[19] && !SUART.rx_buff[20] && SUART.rx_buff[29])		// ������ ����, ���� ��������� � �������� ���� ���������
		{			
		SUART.rx_data[0] = SUART.rx_buff[ 1] | SUART.rx_buff[ 2]<<1 | SUART.rx_buff[ 3]<<2 | SUART.rx_buff[ 4]<<3 | SUART.rx_buff[ 5]<<4 | SUART.rx_buff[ 6]<<5 | SUART.rx_buff[ 7]<<6 | SUART.rx_buff[ 8]<<7 ;
		SUART.rx_data[1] = SUART.rx_buff[11] | SUART.rx_buff[12]<<1 | SUART.rx_buff[13]<<2 | SUART.rx_buff[14]<<3 | SUART.rx_buff[15]<<4 | SUART.rx_buff[16]<<5 | SUART.rx_buff[17]<<6 | SUART.rx_buff[18]<<7 ;
		SUART.rx_data[2] = SUART.rx_buff[21] | SUART.rx_buff[22]<<1 | SUART.rx_buff[23]<<2 | SUART.rx_buff[24]<<3 | SUART.rx_buff[25]<<4 | SUART.rx_buff[26]<<5 | SUART.rx_buff[27]<<6 | SUART.rx_buff[28]<<7 ;
		
			switch(SUART.rx_data_cnt){ // ��������� ������
				case 0:
					SUART.rx_tmp = (SUART.rx_data[1] << 8) | (SUART.rx_data[2]);
					SUART.rx_data_cnt++;
					HPT_Transmite(REQUEST);
					break;
				case 1:
					if(SUART.rx_tmp == ((SUART.rx_data[1] << 8) | (SUART.rx_data[2]))){
						SETUP.hpt_name = (SUART.rx_data[1] << 8) | SUART.rx_data[2];			
						SaveSetting(&SETUP);
						blink(OK_SET);
						SUART.err_cnt = 0;
						SUART.rx_data_cnt = 0;
					}
					break;
			}
		
		}
		else									// ������ ���������� �������, ���� ������ ������������ �����, �� �� ������ 10��
		{
			SUART.err_cnt++;
			if(SUART.err_cnt < 10)
			HPT_Transmite(REQUEST);
		}
		
	}
	
	if(SUART.tim_en)
		SUART.tim_cnt++;
	
}

// ������ ����������� UART
void StartSUART(void)
{
	SUART.tim_en = 1;			// ��������� �������
	SUART.tim_cnt = 0;		// ����� ��������
	SUART.rx_cnt = 0;			// ����� �������� ���
	
	HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);		// ���������� ����������
}




// ������ �������� ���� ������� ������ databuff[0] - �������
	// ������ ���� ������� ������ databuff[0] - �������
uint16_t dataBuff(uint8_t data)
{
	uint16_t databin;
		
		// ��������� �����
	for(uint8_t i = DATALEN-1; i > 0; i--)
	{
		databuff[i] = databuff[i-1];
	}
		// ������ ������ ����
	databuff[0] = data;
		
		// �������������� �����
		
		databin = databuff[0] | databuff[1]<<1 | databuff[2]<<2 | databuff[3]<<3 | databuff[4]<<4 | databuff[5]<<5 | databuff[6]<<6 | databuff[7]<<7 | databuff[8]<<8 | databuff[9]<<9 | databuff[10]<<10 | databuff[11]<<11 | databuff[12]<<12 | databuff[13]<<13 | databuff[14]<<14 | databuff[15]<<15 ; // ������� ������
	//	databin = databuff[15] | databuff[14]<<1 | databuff[13]<<2 | databuff[12]<<3 | databuff[11]<<4 | databuff[10]<<5 | databuff[9]<<6 | databuff[8]<<7 | databuff[7]<<8 | databuff[6]<<9 | databuff[5]<<10 | databuff[4]<<11 | databuff[3]<<12 | databuff[2]<<13 | databuff[1]<<14 | databuff[0]<<15 ; // ������� ������
		
		
		// ��������� ��������
		if(memcmp(&databuff, &alarm, sizeof(databuff)))
			blink(ALARM);
		
	return databin;
}
	

// ���������� ��������� � �����������
void SaveSetting(SettingParametrs_t *Settings)
{
	uint32_t *flash_addr = (uint32_t *)ADR_START;
  uint32_t *settings_addr = (uint32_t *)Settings;
	
	uint32_t flash_addr_e = ADR_START;
  HAL_FLASHEx_DATAEEPROM_Unlock();			// ���������� ������ EEPROM
	HAL_FLASHEx_DATAEEPROM_Erase(flash_addr_e);		
	
	for (uint8_t i = 0; i < sizeof(SETUP); i+=4) {	// ������� EEPROM 
    HAL_FLASHEx_DATAEEPROM_Erase(flash_addr_e);
    flash_addr_e +=4;
  }
	


	
	for (uint32_t i = 0; i < SETTING_WORDS; i++) {// ������ ��������� �� ������ 
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)flash_addr, *(uint32_t *)settings_addr);
    flash_addr++;
    settings_addr++;
  }
	
	
	HAL_FLASHEx_DATAEEPROM_Lock();		// ������ ������ EEPROM
	
}


// ������ �������� �� ������
void LoadSetting(SettingParametrs_t *Settings)
{
	
	uint32_t *flash_addr = (uint32_t *)ADR_START;
	uint32_t *settings_addr = (uint32_t *)Settings;
	
	memcpy((void *)settings_addr, (const void *)flash_addr, sizeof(SettingParametrs_t));
		
}


// ���������� ������� ���������� 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		uint32_t temp;
		switch(GPIO_Pin)
		{
			case External_IN_Pin:							// ��������� ���������� �� ����� ���
				if(!blink_trg){									// ��������� ����� ��� ��������
								
				if(External_IN_GPIO_Port->IDR & External_IN_Pin)																	// ���������� �� ������
				{
					if(!IRQ_abort)								// ������ ����������� �����
							StartSUART();
					
					if(IRQ_abort)									// ���������� �������� � ���
					{
						blink_ext = 0;
						HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_RESET); // ��������� �������� �����
					}
					
					if(hpt_rept_type == ENABLE)		// ������ ������� �� ��������� ����������� UART ��� ��������� 
					{
						hpt_rept_cnt = 0;
						en_cnt = 1;
					}
					
					temp = CAPLAMP_OUT1_GPIO_Port -> MODER;																				  //
					temp &= ~(GPIO_MODER_MODE1 << 0);																								//	������������� ������ � ������� �� GPIO
					temp |= ((GPIO_MODE_OUTPUT_PP & ((uint32_t)0x00000001U)) << 2);									//
					CAPLAMP_OUT1_GPIO_Port -> MODER = temp;																				  //
					
					CAPLAMP_OUT1_GPIO_Port -> ODR |= CAPLAMP_OUT1_Pin;														  //	��������� ������ �����
				}
				else																																							// ���������� �� �����
				{
//					if(blink_end)
//						blink_end = 0;
//					else
					if(IRQ_abort)
						HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_SET); // ���������� �������� �����
						
					

					temp = CAPLAMP_OUT1_GPIO_Port -> MODER;																				  //
					temp &= ~(GPIO_MODER_MODE1 << 0);																								//	������������� ������ � ������� �� GPIO
					temp |= ((GPIO_MODE_OUTPUT_PP & ((uint32_t)0x00000001U)) << 2);									//
					CAPLAMP_OUT1_GPIO_Port -> MODER = temp;																				  //
					
					CAPLAMP_OUT1_GPIO_Port -> ODR &= ~CAPLAMP_OUT1_Pin;														  //	���������� ������ �����
				}
			}
				break;
			
		}
	}
	
// �������� ������� ��� ���
void HPT_Transmite(uint8_t type)
{
	HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);							// ���������� ���������� �� ����� ���
	hpt_rept_type = type;
	hpt_rept_cnt = 0;
	en_cnt = 1;
	HAL_GPIO_WritePin(HPT_Answer_OUT_GPIO_Port,HPT_Answer_OUT_Pin, GPIO_PIN_SET);
	
}

// �� ������� ������� ����� 
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

// ��������� �������� ����� ��� ��������� ��������
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	blink_end = 1;
	blink_trg = 0;
	HAL_GPIO_WritePin(Interrupt_OUT2_GPIO_Port,Interrupt_OUT2_Pin, GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_2);
	if(SUART.err_cnt >= 10)				// ����������� ��������, ���� ��� ����������� ������ �� ���
		blink(START);
}

	// ����� �� ��������� ���������
	void DefaultSettings(void)
	{
		SETUP.hpt_name = 61680;	// 1111 0000 1111 0000 43690;			// 1010 1010 1010 1010
		
		SETUP.repeatnum = 1;
		
		SETUP.serailnum	= 0x00000000U;
		SETUP.firmware	= 0x6197U;			// ������ �� 12.3.18-7
		
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
		
	SaveSetting(&SETUP);

	}
	// ��������� ������ �� UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		
		if (UART2_RECV_CMD == UART2_RecvType)
		{
				HAL_TIM_Base_Stop_IT(&htim21);
			HAL_ADC_Stop_DMA(&hadc);
			
		switch(CMD)
		{
			case SET_NAME_HPT:								// ����� �����
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.hpt_name,sizeof(SETUP.hpt_name));
				break;
			case SET_BODRATE:							// �������
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.BR,sizeof(SETUP.BR));
				break;
			case SET_REPEATNUM:							// ���������� ��������
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.hpt_name,sizeof(SETUP.hpt_name));
				break;
			case SET_SAMPLENUM:							// ����������� ���������
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.samplenum,sizeof(SETUP.samplenum));
				break;
			case SET_F1S1:								// ������� 1 ������ 1
				//HAL_TIM_Base_Stop_IT(&htim21);
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cf1s1,sizeof(SETUP.cf1s1));
				break;
			case SET_F1S2:								// ������� 1 ������ 2
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cf1s2,sizeof(SETUP.cf1s2));
				break;
			case SET_F2S1:								// ������� 2 ������ 1
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cf2s1,sizeof(SETUP.cf2s1));
				break;
			case SET_F2S2:								// ������� 2 ������ 2
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cf2s2,sizeof(SETUP.cf2s2));
				break;
			case SET_FLP:								// ������� ���
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.cflp,sizeof(SETUP.cflp));
				break;
			case SET_DET_THRES:				// ����� �����������
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
			case SET_SN:							// ��������� ��������� ������
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.serailnum,sizeof(SETUP.serailnum));
				break;
			case SET_FW:							// ��������� ������ ��
				UART2_RecvType = UART2_RECV_VALUE;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&SETUP.firmware,sizeof(SETUP.firmware));
				break;
			case START_DTR_FLP:				// ������ �������� ������
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case START_DTR_F1:				// ������ �������� ������
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case START_DTR_F2:				// ������ �������� ������
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case START_DTR_IN:				// ������ �������� ������
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case STOP_DTR:					// ��������� �������� ������
				UART2_RecvType = UART2_RECV_CMD;
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_SETUP:				// ��������� ���������
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP,sizeof(SETUP));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_F1:					// ��������� ������� 1
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.f1,sizeof(SETUP.f1));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_F2:					// ��������� ������� 2
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.f2,sizeof(SETUP.f2));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_BR:					// ��������� �������
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.BR,sizeof(SETUP.BR));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_DET_THRES:					// ��������� �������
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.ratio,sizeof(SETUP.ratio));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_NAME_HPT:		// ��������� ����� HPT
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.hpt_name,sizeof(SETUP.hpt_name));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_SN:		// ��������� �������� �����
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.serailnum,sizeof(SETUP.serailnum));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_FW:		// ��������� ������ ��
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.firmware,sizeof(SETUP.firmware));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_LAST_CALL:		// ��������� ��������� �������� ����� HPT
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.hpt_name,sizeof(SETUP.hpt_name));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case GET_LAST_CALL_TYPE:		// ��������� ��������� ��� ������ 
				UART2_RecvType = UART2_RECV_CMD;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&SETUP.hpt_name,sizeof(SETUP.hpt_name));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case DEF_SETT:								// ��������� ����� �� ������ �����������
				UART2_RecvType = UART2_RECV_CMD;
				DefaultSettings();
				SaveSetting(&SETUP);
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			case CONNECT:								// ��������� ����� �� ������ �����������
				UART2_RecvType = UART2_RECV_CMD;
				CMD_Rept = OK;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&CMD_Rept,sizeof(CMD_Rept));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
			
			
			default:										// �������� ������ �� ������������ ������
				UART2_RecvType = UART2_RECV_CMD;
				CMD_Rept = ERR;
				HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&CMD_Rept,sizeof(CMD_Rept));
				if(!(TIM21->CR1))
					HAL_TIM_Base_Start_IT(&htim21);
				HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
				break;
		}}
		else{
			//__disable_irq();
			SaveSetting(&SETUP);
			//__enable_irq();
			blink(OK_SET);
			HAL_ADC_Start_DMA(&hadc, (uint32_t*)&buff1,1);
			if(!(TIM21->CR1))
				HAL_TIM_Base_Start_IT(&htim21);
		
		UART2_RecvType = UART2_RECV_CMD;
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&CMD,sizeof(CMD));
	}
		
	}
	
	
