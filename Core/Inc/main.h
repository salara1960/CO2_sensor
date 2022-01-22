/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <malloc.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdarg.h>

#include "hdr.h"

#ifdef SET_SPI_DISPLAY
	#include "font.h"
	#include "gc9a01.h"
	#include "dispcolor.h"
#endif

#ifdef SET_BME280
#include "bmx280.h"
#endif

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

enum {
	devUART = 1,
	devADC,
	devFifo,
	devQue,
	devMem,
	devTimeout,
	devBreak,
	devProg,
	devRead,
	devCmd,
	devDma,
	devFlash,
	devAck,
	devLCD,
	devI2C
#ifdef SET_SI7021
	,devCRC
#endif
};

enum {
	rdNone = 0,
	rdApi,
	rdHdr
};

#pragma pack(push,1)
typedef struct q_rec_t {
	uint32_t mark;
	uint32_t adr;
	uint32_t len;
	uint32_t crc;
} api_hdr_t;
#pragma pack(pop)


#ifdef BOOT_LOADER

	#include "stm32f4xx.h"

	#include "bootloader.h"

	#ifdef SET_LOGGER
		UART_HandleTypeDef *logPort;
	#endif

	enum {
		apiNone = -1,
		apiProg,
		apiRead,
		apiStop,
		apiSwitch,
		apiDone,
		blkReady
	};

#else

	#include "stm32f4xx_hal.h"

	#ifdef SET_MQ135
		#include "mq135.h"
	#endif

	#if defined(SET_BME280)
		#include "bmp280.h"
	#elif defined(SET_SI7021)
		#include "si7021.h"
	#endif

	typedef enum {
		evt_empty = 0,
		evt_rst,
		evt_sec,
		evt_porogMQ,
		evt_getMQ,
		evt_intrMQ,
		evt_readFW,
		evt_errInput,
		evt_dmaMem,
		evt_dmaHdr,
		evt_getVer,
		evt_getHdr,
		evt_bmpBegin,
		evt_bmpNext,
		evt_bmpEnd,
		evt_bmpPrn,
		evt_bmpNone,
		evt_siReset,
		evt_siReadT,
		evt_siReadH,
		evt_siPrn,
		evt_errCmd,
		evt_none
	} evt_t;

#endif


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BOOT_Pin GPIO_PIN_0
#define BOOT_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_1
#define LCD_CS_GPIO_Port GPIOB
#define digMQ_Pin GPIO_PIN_2
#define digMQ_GPIO_Port GPIOB
#define digMQ_EXTI_IRQn EXTI2_IRQn
#define ERR_Pin GPIO_PIN_10
#define ERR_GPIO_Port GPIOB
#define MQ_ALARM_Pin GPIO_PIN_12
#define MQ_ALARM_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_14
#define LCD_DC_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_15
#define LCD_MOSI_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_8
#define LCD_RST_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

#define LOOP_FOREVER() while(1) { HAL_Delay(1); }

#define MAX_VREM_BUF 1024
#define FLASH_ADDR 0x8000000   // Базовый адрес внутренней flash-памяти МК STM32F411
#define BOOT_LEN 0x8000        // Максимальный размер внутренней flash-памяти для загрузчика - (сектора 0..1 - 32k)
#define FLASH_LEN (512 * 1024) // Максимальный размер внутренней flash-памяти МК STM32F411 - 512k = 0x80000
#define SPEC_AREA_LEN 0x10     // Размер области заголовка API - 16 байт
#define API_START_ADR (FLASH_ADDR + BOOT_LEN) // Начальный адрес для размещения API // 32k
#define MAX_API_SIZE (FLASH_LEN - BOOT_LEN - SPEC_AREA_LEN) // Максимально допустимый размер API
#define SPEC_AREA_ADR (FLASH_ADDR + FLASH_LEN - SPEC_AREA_LEN) // Начальный адрес для размещения заголовка API - 0x807ff00
#define API_MARKER 0x5aa5f00f // Константа маркера из заголовка API

#define OTP_AREA_BEGIN 0x1FFF7800
#define OTP_AREA_END   0x1FFF7A0F
#define OTP_AREA_SIZE  512
#define OTP_LOCK_BEGIN 0x1FFFC000
#define OTP_LOCK_END   0x1FFFC00F
#define OTP_LOCK_SIZE  16


#define MAX_UART_BUF 1024
#define BACK_SPACE 8
#define MAX_FIFO_SIZE 64
#define PAGE_SIZE 256

#define MAX_ADC_BUF 8

#define _10ms 1
#define _20ms (_10ms * 2)
#define _30ms (_10ms * 3)
#define _40ms (_10ms * 4)
#define _50ms (_10ms * 5)
#define _60ms (_10ms * 6)
#define _70ms (_10ms * 7)
#define _80ms (_10ms * 8)
#define _90ms (_10ms * 9)
#define _100ms (_10ms * 10)
#define _130ms (_10ms * 13)
#define _150ms (_10ms * 15)
#define _160ms (_10ms * 16)
#define _170ms (_10ms * 17)
#define _200ms (_10ms * 20)
#define _250ms (_10ms * 25)
#define _300ms (_10ms * 30)
#define _350ms (_10ms * 35)
#define _400ms (_10ms * 40)
#define _450ms (_10ms * 45)
#define _500ms (_10ms * 50)
#define _600ms (_10ms * 60)
#define _700ms (_10ms * 70)
#define _800ms (_10ms * 80)
#define _900ms (_10ms * 90)
#define _1s (_10ms * 100)
#define _1s25 ((_1s * 1) + _250ms)
#define _1s3 ((_1s * 1) + _300ms)
#define _1s5 (_10ms * 150)
#define _2s (_1s * 2)
#define _2s3 ((_1s * 2) + _300ms)
#define _3s (_1s * 3)
#define _4s (_1s * 4)
#define _4s3 ((_1s * 4) + _300ms)
#define _5s (_1s * 5)
#define _6s (_1s * 6)
#define _7s (_1s * 7)
#define _8s (_1s * 8)
#define _9s (_1s * 9)
#define _10s (_1s * 10)
#define _15s (_1s * 15)
#define _20s (_1s * 20)
#define _30s (_1s * 30)

//--------------------------------------------------------------------

UART_HandleTypeDef *uartPort;
DMA_HandleTypeDef *dmaMem;

#ifdef SET_SPI_DISPLAY
	#define LCD_CS_ON()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,  LCD_CS_Pin,  GPIO_PIN_RESET)
	#define LCD_CS_OFF()  HAL_GPIO_WritePin(LCD_CS_GPIO_Port,  LCD_CS_Pin,  GPIO_PIN_SET)
	#define LCD_RST_ON()  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
	#define LCD_RST_OFF() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)
	#define LCD_CMD()     HAL_GPIO_WritePin(LCD_DC_GPIO_Port,  LCD_DC_Pin,  GPIO_PIN_RESET)
	#define LCD_DATA()    HAL_GPIO_WritePin(LCD_DC_GPIO_Port,  LCD_DC_Pin,  GPIO_PIN_SET)

	SPI_HandleTypeDef *lcdPort;
	volatile uint8_t lcdRdy;
#endif

bool bootMode;

const char *eol;
char stx[MAX_UART_BUF];
uint8_t rxByte;
uint16_t rxInd;
char rxBuf[MAX_UART_BUF];
uint32_t devError;
char txBuf[MAX_UART_BUF];
volatile bool startAPI;
uint8_t blkRdy;
bool uartReady;

#ifdef SET_MQ135
	uint16_t valMQ;
#endif

#if defined(SET_BME280) || defined(SET_SI7021)
	const uint32_t min_wait_ms;
	const uint32_t max_wait_ms;
	I2C_HandleTypeDef *portI2C;
	uint8_t i2c_txReady, i2c_rxReady;
#endif


api_hdr_t hdr;
uint8_t rdMem;

//--------------------------------------------------------------------

uint32_t get_tmr10(uint32_t ms);
bool check_tmr10(uint32_t ms);
int sec2str(char *stx);
void errLedOn(bool on);
uint8_t Report(const char *tag, bool addTime, const char *fmt, ...);

//--------------------------------------------------------------------

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
