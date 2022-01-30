/*
 * bootloader.h
 *
 *  Created on: Dec 23, 2021
 *      Author: alarm
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#include "main.h"

#ifdef BOOT_LOADER


#define SET_FW_PROG

//#define SET_LOGGER

//-------------------------------------------------------------------

#define GPIO_NUMBER 16U
#define MAX_FLASH_SECTOR 8
#define MAX_ACTIONS 3
#define BOOT_PIN BOOT_Pin
#define BOOT_PORT_PIN BOOT_GPIO_Port

enum {
	do_none = 0,
	do_prog,
	do_read
};

#pragma pack(push,1)
typedef struct {
	uint8_t sector;
	uint32_t adr;
	uint32_t len;
} sector_t;
#pragma pack(pop)



//-------------------------------------------------------------------

#define _writePin(port, pin, val) ({ port->BSRR = (val > 0) ? pin : ((uint32_t)pin) << 16U; })

#define _togglePin(port, pin) ({ uint32_t odr = port->ODR; \
			port->BSRR = ((odr & pin) << GPIO_NUMBER) | (~odr & pin); })

//-------------------------------------------------------------------

const char *version;

const char *name;
const char *_progfw;
const char *_apiStop;
const char *_switch;
const char *_ready;
const char *_getfw;
const char *_next;
const char *_done;
const char *allAction[MAX_ACTIONS];

uint8_t action;
uint8_t *vrem;
uint32_t file_size;
int8_t apiCmd;
uint32_t api_crc, file_crc;
uint32_t apiAddr;
uint32_t hdrAddr;
uint32_t apiAddrCur;
api_hdr_t apiHdr;
uint32_t fLen;

bool dmaDone;
bool progDone;
const sector_t allFlash[MAX_FLASH_SECTOR];
uint8_t needErase[MAX_FLASH_SECTOR];
const uint32_t crc32_tab[256];

//----------------------------------------------------------------------

#ifdef SET_LOGGER
	uint8_t logByte;
	bool logReady;
	extern UART_HandleTypeDef *logPort;
#endif

void Logger(const char *tag, bool addTime, const char *fmt, ...);
uint32_t crc32(const uint32_t orig, const uint8_t *buf, uint32_t size);
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void dmaCB(DMA_HandleTypeDef *dmem);
//void checkSectors();
void flash_erase(uint8_t sector);
uint8_t flash_write(volatile uint32_t *address, uint32_t *data, uint16_t size);
uint8_t progBlk(uint32_t faddr, const uint8_t *buf, int len);
uint32_t doAPI(uint32_t *aRet);

//----------------------------------------------------------------------

#endif

#endif /* INC_BOOTLOADER_H_ */
