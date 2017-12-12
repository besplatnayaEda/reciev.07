

#include "stm32l0xx_hal.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"

#define NAME 206
#define START			0
#define ALARM 		1
#define PERSONAL	2
#define OK_SET		3


#define ADR_START 0x08080000

#define FLASH_WORD_SIZE 4UL

#define SETTING_RECORD_SIZE ((sizeof(SettingParametrs_t) + FLASH_WORD_SIZE - 1) & (~(FLASH_WORD_SIZE - 1)))
// ������ ��������� �������� � ������
#define SETTING_WORDS (SETTING_RECORD_SIZE / FLASH_WORD_SIZE)


	//	������ ������ ��� UART
typedef enum {	UNDEFINED = -1, 
				NUMBER,							// ����� �����
				BODRATE,						// �������
				REPEATNUM,					// ���������� ��������
				F1S1,								// ������� 1 ������ 1
				F1S2,								// ������� 1 ������ 2
				F2S1,								// ������� 2 ������ 1
				F2S2,								// ������� 2 ������ 2
				FLP,								// ������� ���
				DETECTION_THRESHOLD,					// ����� �����������
				START_DATA_TRANSMIT_FLP,			// ������ �������� ������ � ���
				STOP_DATA_TRANSMIT,						// ��������� �������� ������
				START_DATA_TRANSMIT_F1,				// ������ �������� ������ � f1
				START_DATA_TRANSMIT_F2,				// ������ �������� ������ � f2
				START_DATA_TRANSMIT_IN,				// ������ �������� ������ � inp
				GET_SETUP,						// �������� ��������� � �����������
				GET_F1,								// �������� ������� f1
				GET_F2,								// �������� ������� f2
				GET_BR,								// �������� ��������
				GET_NAME,							// �������� ������
				CONNECT,							// ������ �����������
				OK,										// ����� ��
				ERR,									// ����� ERR
				F1,										// ��������� �������� f1
				F2,										// ��������� �������� f2
				SAMPLENUM							// ��������� �������� ������� �� ���
			 } Cmd_Type;


typedef enum {UART2_RECV_CMD, UART2_RECV_VALUE} UART2Recv_t;

typedef struct SettingParametrs {

	uint16_t name;			// ����� �����
	uint16_t samplenum;		// ����� ������� �� ���� ��� (BR*3200)
	uint8_t	 repeatnum;		// ���������� ��������
	uint16_t f1;			// ������������� ������� f1 � ��
	uint16_t f2;			// ������������� ������� f2 � ��
	uint8_t  BR;			// ������������� ������� � 10*���/�
	
	float cf1s1[5];			// ������� 1 ������ 1
	float cf1s2[5];			// ������� 1 ������ 2
	float cf2s1[5];			// ������� 2 ������ 1
	float cf2s2[5];			// ������� 2 ������ 2
	float cflp[5];			// ������� ���
	
	float ratio;				// ����� �����������
	
} SettingParametrs_t, *pSettingParametrs_t;

float iir_test(float in, float *coef, float *his);
unsigned int crc_calculating(unsigned char *puchMsg, unsigned short usDataLen);

void blink(char mode);
void SetCoefficiens(uint16_t freq1, uint16_t freq2, uint16_t boudrate);
void DefaultSettings(void);

void SaveSetting(SettingParametrs_t *Settings);
void LoadSetting(SettingParametrs_t *Settings);




/*

typedef struct rxData {
	// ��������� ���������
	uint16_t name;
	uint8_t br;
	uint16_t f1;
	uint16_t f2;
	uint8_t res;
} rxData_t, *prxData_t;

typedef struct txData {
	// ����� ���������
	uint8_t start;
	uint8_t det;
	uint8_t nop;
	uint16_t name;
	uint16_t crc;
	uint8_t stop;
	
} txData_t, *ptxData_t;
*/
