

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
// размер структуры настроек в словах
#define SETTING_WORDS (SETTING_RECORD_SIZE / FLASH_WORD_SIZE)


	//	список команд для UART
typedef enum {	UNDEFINED = -1, 
				NUMBER,							// номер метки
				BODRATE,						// бодрейт
				REPEATNUM,					// количество повторов
				F1S1,								// частота 1 секция 1
				F1S2,								// частота 1 секция 2
				F2S1,								// частота 2 секция 1
				F2S2,								// частота 2 секция 2
				FLP,								// частота ФНЧ
				DETECTION_THRESHOLD,					// порог обнаружения
				START_DATA_TRANSMIT_FLP,			// запуск передачи данных с фнч
				STOP_DATA_TRANSMIT,						// остановка передачи данных
				START_DATA_TRANSMIT_F1,				// запуск передачи данных с f1
				START_DATA_TRANSMIT_F2,				// запуск передачи данных с f2
				START_DATA_TRANSMIT_IN,				// запуск передачи данных с inp
				GET_SETUP,						// передача структуры с настройками
				GET_F1,								// передача частоты f1
				GET_F2,								// передача частоты f2
				GET_BR,								// передача бодрейта
				GET_NAME,							// передача номера
				CONNECT,							// запрос подключения
				OK,										// ответ ОК
				ERR,									// ответ ERR
				F1,										// установка значения f1
				F2,										// установка значения f2
				SAMPLENUM							// установка значения выборок на бит
			 } Cmd_Type;


typedef enum {UART2_RECV_CMD, UART2_RECV_VALUE} UART2Recv_t;

typedef struct SettingParametrs {

	uint16_t name;			// номер метки
	uint16_t samplenum;		// число выборок на один бит (BR*3200)
	uint8_t	 repeatnum;		// количество повторов
	uint16_t f1;			// установленная частота f1 в гц
	uint16_t f2;			// установленная частота f2 в гц
	uint8_t  BR;			// установленный бодрейт в 10*бит/с
	
	float cf1s1[5];			// частота 1 секция 1
	float cf1s2[5];			// частота 1 секция 2
	float cf2s1[5];			// частота 2 секция 1
	float cf2s2[5];			// частота 2 секция 2
	float cflp[5];			// частота ФНЧ
	
	float ratio;				// порог обнаружения
	
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
	// настройки приемника
	uint16_t name;
	uint8_t br;
	uint16_t f1;
	uint16_t f2;
	uint8_t res;
} rxData_t, *prxData_t;

typedef struct txData {
	// ответ приемника
	uint8_t start;
	uint8_t det;
	uint8_t nop;
	uint16_t name;
	uint16_t crc;
	uint8_t stop;
	
} txData_t, *ptxData_t;
*/
