

#include "stm32l0xx_hal.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"

// режим моргания фонаря
#define START			0			// стартовый 
#define ALARM 		1			// авария
#define PERSONAL	2			// персональный вызов
#define OK_SET		3			// настройка 

// тип запроса для НРТ метки
#define CONFIRM		0		//	195 ms
#define REQUEST		1		//	395 ms
#define	TEST			2		//	590 ms
#define ENABLE		3		// включение HPT

// стартовый адрес для сохранения
#define ADR_START 0x08080000

#define FLASH_WORD_SIZE 4UL		// размер сохраняемого слова

// расчет размера структуры
#define SETTING_RECORD_SIZE ((sizeof(SettingParametrs_t) + FLASH_WORD_SIZE - 1) & (~(FLASH_WORD_SIZE - 1)))
// размер структуры настроек в словах
#define SETTING_WORDS (SETTING_RECORD_SIZE / FLASH_WORD_SIZE)


	//	список команд для UART
typedef enum {	UNDEFINED = -1, 
				SET_NAME_HPT,									// номер метки
				SET_BODRATE,									// бодрейт
				SET_REPEATNUM,								// количество повторов
				SET_SAMPLENUM,								// установка значения выборок на бит
				SET_F1S1,											// частота 1 секция 1
				SET_F1S2,											// частота 1 секция 2
				SET_F2S1,											// частота 2 секция 1
				SET_F2S2,											// частота 2 секция 2
				SET_FLP,											// частота ФНЧ
				SET_DET_THRES,								// порог обнаружения
				SET_F1,												// установка значения f1
				SET_F2,												// установка значения f2
				SET_SN,												// установка серийного номера
				SET_FW,												// установить версью ПО
				START_DTR_FLP,								// запуск передачи данных с фнч
				START_DTR_F1,									// запуск передачи данных с f1
				START_DTR_F2,									// запуск передачи данных с f2
				START_DTR_IN,									// запуск передачи данных с inp
				STOP_DTR,											// остановка передачи данных
				GET_SETUP,										// передача структуры с настройками
				GET_F1,												// передача частоты f1
				GET_F2,												// передача частоты f2
				GET_BR,												// передача бодрейта
				GET_DET_THRES,								// передача порога обнаружения
				GET_NAME_HPT,									// передача номера
				GET_SN,												// получить серийный номер
				GET_FW,												// получить версию ПО
				GET_LAST_CALL,								// получить последний номер
				GET_LAST_CALL_TYPE,						// получить тип последнего вызова
				SEND_CALL_NAME,								// передача полученного номера
				SEND_CALL_TYPE,								// передача типа вызова
				SEND_DTR_FLP,									// передача данных с фнч
				SEND_DTR_F1,									// передача данных с f1
				SEND_DTR_F2,									// передача данных с f2
				SEND_DTR_IN,									// передача данных с inp
				DEF_SETT,											// сброс на заводские настройки
				CONNECT,											// запрос подключения
				OK,														// ответ ОК
				ERR														// ответ ERR
				
				
			 } Cmd_Type;


// тип принимаемых данных
typedef enum {UART2_RECV_CMD, UART2_RECV_VALUE} UART2Recv_t;

// структура с настройками
typedef struct SettingParametrs {

	uint16_t hpt_name;			// номер метки
	uint16_t samplenum;			// число выборок на один бит (BR*3200)
	uint8_t	 repeatnum;			// количество повторов
	uint16_t f1;						// установленная частота f1 в гц
	uint16_t f2;						// установленная частота f2 в гц
	uint8_t  BR;						// установленный бодрейт в 10*бит/с
	uint32_t serailnum;			// серийный номер
	uint16_t firmware;		// версия по
	
	float cf1s1[5];			// частота 1 секция 1
	float cf1s2[5];			// частота 1 секция 2
	float cf2s1[5];			// частота 2 секция 1
	float cf2s2[5];			// частота 2 секция 2
	float cflp[5];			// частота ФНЧ
	
	float ratio;				// порог обнаружения
	
} SettingParametrs_t, *pSettingParametrs_t;

// програмный uart
typedef struct SoftUART_15Baud	{
	uint8_t	tim_en;						// запуск таймера
	uint8_t	tim_cnt;					// счетчик
	uint8_t	rx_cnt;						// счетчик принятых бит
	uint8_t err_cnt;					// счетчик ошибочных байт
	uint8_t rx_buff[30];			// приемный буфер для бит
	uint8_t	rx_data[3];				// приемный буфер для байт
	uint8_t rx_data_cnt;		  // счетчик принятых байт
	uint16_t rx_tmp;					// временный буфер для данных


}SoftUART_15Baud_t, *pSoftUART_15Baud_t;


float IIR_SOS(float in, float *coef, float *his);													// рекурсивный фильтр второго порядка


void blink(char mode);				// запуск моргания
void DefaultSettings(void);		// сброс настроек на заводские 

void SaveSetting(SettingParametrs_t *Settings);		// сохранение настроек
void LoadSetting(SettingParametrs_t *Settings);		// загрузка настроек

void HPT_Transmite(uint8_t type);			// ответ HPT метке
void S_UART(void);										// обработчик програмного UART

void StartSUART(void);								// запуск програмного UART


uint16_t dataBuff(uint8_t data);			// кольцевой буфер


