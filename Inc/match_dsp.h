

#include "stm32l0xx_hal.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"

// ����� �������� ������
#define START			0			// ��������� 
#define ALARM 		1			// ������
#define PERSONAL	2			// ������������ �����
#define OK_SET		3			// ��������� 

// ��� ������� ��� ��� �����
#define CONFIRM		0		//	195 ms
#define REQUEST		1		//	395 ms
#define	TEST			2		//	590 ms
#define ENABLE		3		// ��������� HPT

// ��������� ����� ��� ����������
#define ADR_START 0x08080000

#define FLASH_WORD_SIZE 4UL		// ������ ������������ �����

// ������ ������� ���������
#define SETTING_RECORD_SIZE ((sizeof(SettingParametrs_t) + FLASH_WORD_SIZE - 1) & (~(FLASH_WORD_SIZE - 1)))
// ������ ��������� �������� � ������
#define SETTING_WORDS (SETTING_RECORD_SIZE / FLASH_WORD_SIZE)


	//	������ ������ ��� UART
typedef enum {	UNDEFINED = -1, 
				SET_NAME_HPT,									// ����� �����
				SET_BODRATE,									// �������
				SET_REPEATNUM,								// ���������� ��������
				SET_SAMPLENUM,								// ��������� �������� ������� �� ���
				SET_F1S1,											// ������� 1 ������ 1
				SET_F1S2,											// ������� 1 ������ 2
				SET_F2S1,											// ������� 2 ������ 1
				SET_F2S2,											// ������� 2 ������ 2
				SET_FLP,											// ������� ���
				SET_DET_THRES,								// ����� �����������
				SET_F1,												// ��������� �������� f1
				SET_F2,												// ��������� �������� f2
				SET_SN,												// ��������� ��������� ������
				SET_FW,												// ���������� ������ ��
				START_DTR_FLP,								// ������ �������� ������ � ���
				START_DTR_F1,									// ������ �������� ������ � f1
				START_DTR_F2,									// ������ �������� ������ � f2
				START_DTR_IN,									// ������ �������� ������ � inp
				STOP_DTR,											// ��������� �������� ������
				GET_SETUP,										// �������� ��������� � �����������
				GET_F1,												// �������� ������� f1
				GET_F2,												// �������� ������� f2
				GET_BR,												// �������� ��������
				GET_DET_THRES,								// �������� ������ �����������
				GET_NAME_HPT,									// �������� ������
				GET_SN,												// �������� �������� �����
				GET_FW,												// �������� ������ ��
				GET_LAST_CALL,								// �������� ��������� �����
				GET_LAST_CALL_TYPE,						// �������� ��� ���������� ������
				SEND_CALL_NAME,								// �������� ����������� ������
				SEND_CALL_TYPE,								// �������� ���� ������
				SEND_DTR_FLP,									// �������� ������ � ���
				SEND_DTR_F1,									// �������� ������ � f1
				SEND_DTR_F2,									// �������� ������ � f2
				SEND_DTR_IN,									// �������� ������ � inp
				DEF_SETT,											// ����� �� ��������� ���������
				CONNECT,											// ������ �����������
				OK,														// ����� ��
				ERR														// ����� ERR
				
				
			 } Cmd_Type;


// ��� ����������� ������
typedef enum {UART2_RECV_CMD, UART2_RECV_VALUE} UART2Recv_t;

// ��������� � �����������
typedef struct SettingParametrs {

	uint16_t hpt_name;			// ����� �����
	uint16_t samplenum;			// ����� ������� �� ���� ��� (BR*3200)
	uint8_t	 repeatnum;			// ���������� ��������
	uint16_t f1;						// ������������� ������� f1 � ��
	uint16_t f2;						// ������������� ������� f2 � ��
	uint8_t  BR;						// ������������� ������� � 10*���/�
	uint32_t serailnum;			// �������� �����
	uint16_t firmware;		// ������ ��
	
	float cf1s1[5];			// ������� 1 ������ 1
	float cf1s2[5];			// ������� 1 ������ 2
	float cf2s1[5];			// ������� 2 ������ 1
	float cf2s2[5];			// ������� 2 ������ 2
	float cflp[5];			// ������� ���
	
	float ratio;				// ����� �����������
	
} SettingParametrs_t, *pSettingParametrs_t;

// ���������� uart
typedef struct SoftUART_15Baud	{
	uint8_t	tim_en;						// ������ �������
	uint8_t	tim_cnt;					// �������
	uint8_t	rx_cnt;						// ������� �������� ���
	uint8_t err_cnt;					// ������� ��������� ����
	uint8_t rx_buff[30];			// �������� ����� ��� ���
	uint8_t	rx_data[3];				// �������� ����� ��� ����
	uint8_t rx_data_cnt;		  // ������� �������� ����
	uint16_t rx_tmp;					// ��������� ����� ��� ������


}SoftUART_15Baud_t, *pSoftUART_15Baud_t;


float IIR_SOS(float in, float *coef, float *his);													// ����������� ������ ������� �������


void blink(char mode);				// ������ ��������
void DefaultSettings(void);		// ����� �������� �� ��������� 

void SaveSetting(SettingParametrs_t *Settings);		// ���������� ��������
void LoadSetting(SettingParametrs_t *Settings);		// �������� ��������

void HPT_Transmite(uint8_t type);			// ����� HPT �����
void S_UART(void);										// ���������� ����������� UART

void StartSUART(void);								// ������ ����������� UART


uint16_t dataBuff(uint8_t data);			// ��������� �����


