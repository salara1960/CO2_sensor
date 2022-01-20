/*
 * bootloader.c
 *
 *  Created on: Dec 23, 2021
 *      Author: alarm
 */
#ifdef BOOT_LOADER

#include "bootloader.h"

//const char *version = "ver.0.4 16.12.2021 BOOT";
//const char *version = "ver.0.5 17.12.2021 BOOT";
//const char *version = "ver.0.6 19.12.2021 BOOT";
//const char *version = "ver.0.7 22.12.2021 BOOT";
//const char *version = "ver.0.8 23.12.2021 BOOT";
//const char *version = "ver.0.9 25.12.2021 BOOT";
//const char *version = "ver.1.0 26.12.2021 BOOT";
//const char *version = "ver.1.1 28.12.2021 BOOT";
//const char *version = "ver.1.2 29.12.2021 BOOT";
//const char *version = "ver.1.3 03.01.2022 BOOT";
const char *version = "ver.1.4 13.01.2022 BOOT";


const char *name = "boot";
const char *_progfw = "prog:";
const char *_apiStop = "stop";
const char *_switch = "switch";
const char *_getfw = "read";
const char *_ready = "ready\r\n";
const char *_next = "next\r\n";
const char *_done = "done\r\n";
const char *allAction[MAX_ACTIONS] = {
	"none",
	"prog",
	"read"
};


uint8_t action = do_none;
uint8_t *vrem = NULL;
uint32_t file_size = 0;
int8_t apiCmd = apiNone;
uint32_t api_crc = 0, file_crc = 0, apiLen = 0, apiCrc = 0;
uint32_t apiAddr = API_START_ADR;// sector 2...
uint32_t hdrAddr = SPEC_AREA_ADR;// sector 7
uint32_t apiAddrCur = API_START_ADR;
api_hdr_t apiHdr = {0};
uint32_t fLen = 0;
uint32_t blk_cnt = 0;
char tmp[64] = {0};
bool dmaDone = false;
bool progDone = false;
#ifdef SET_LOGGER
	char logBuf[MAX_UART_BUF] = {0};
	bool logReady = true;
	uint8_t logByte;
#endif

const sector_t allFlash[MAX_FLASH_SECTOR] = {
	{0, 0x8000000, 16384},
	{1, 0x8004000, 16384},
	{2, 0x8008000, 16384},
	{3, 0x800C000, 16384},
	{4, 0x8010000, 65536},
	{5, 0x8020000, 131072},
	{6, 0x8030000, 131072},
	{7, 0x8060000, 131072}//20000
};

uint8_t needErase[MAX_FLASH_SECTOR] = {0};


const uint32_t crc32_tab[256] =
{
	0x00000000UL, 0x77073096UL, 0xee0e612cUL, 0x990951baUL,
	0x076dc419UL, 0x706af48fUL, 0xe963a535UL, 0x9e6495a3UL,
	0x0edb8832UL, 0x79dcb8a4UL, 0xe0d5e91eUL, 0x97d2d988UL,
	0x09b64c2bUL, 0x7eb17cbdUL, 0xe7b82d07UL, 0x90bf1d91UL,
	0x1db71064UL, 0x6ab020f2UL, 0xf3b97148UL, 0x84be41deUL,
	0x1adad47dUL, 0x6ddde4ebUL, 0xf4d4b551UL, 0x83d385c7UL,
	0x136c9856UL, 0x646ba8c0UL, 0xfd62f97aUL, 0x8a65c9ecUL,
	0x14015c4fUL, 0x63066cd9UL, 0xfa0f3d63UL, 0x8d080df5UL,
	0x3b6e20c8UL, 0x4c69105eUL, 0xd56041e4UL, 0xa2677172UL,
	0x3c03e4d1UL, 0x4b04d447UL, 0xd20d85fdUL, 0xa50ab56bUL,
	0x35b5a8faUL, 0x42b2986cUL, 0xdbbbc9d6UL, 0xacbcf940UL,
	0x32d86ce3UL, 0x45df5c75UL, 0xdcd60dcfUL, 0xabd13d59UL,
	0x26d930acUL, 0x51de003aUL, 0xc8d75180UL, 0xbfd06116UL,
	0x21b4f4b5UL, 0x56b3c423UL, 0xcfba9599UL, 0xb8bda50fUL,
	0x2802b89eUL, 0x5f058808UL, 0xc60cd9b2UL, 0xb10be924UL,
	0x2f6f7c87UL, 0x58684c11UL, 0xc1611dabUL, 0xb6662d3dUL,
	0x76dc4190UL, 0x01db7106UL, 0x98d220bcUL, 0xefd5102aUL,
	0x71b18589UL, 0x06b6b51fUL, 0x9fbfe4a5UL, 0xe8b8d433UL,
	0x7807c9a2UL, 0x0f00f934UL, 0x9609a88eUL, 0xe10e9818UL,
	0x7f6a0dbbUL, 0x086d3d2dUL, 0x91646c97UL, 0xe6635c01UL,
	0x6b6b51f4UL, 0x1c6c6162UL, 0x856530d8UL, 0xf262004eUL,
	0x6c0695edUL, 0x1b01a57bUL, 0x8208f4c1UL, 0xf50fc457UL,
	0x65b0d9c6UL, 0x12b7e950UL, 0x8bbeb8eaUL, 0xfcb9887cUL,
	0x62dd1ddfUL, 0x15da2d49UL, 0x8cd37cf3UL, 0xfbd44c65UL,
	0x4db26158UL, 0x3ab551ceUL, 0xa3bc0074UL, 0xd4bb30e2UL,
	0x4adfa541UL, 0x3dd895d7UL, 0xa4d1c46dUL, 0xd3d6f4fbUL,
	0x4369e96aUL, 0x346ed9fcUL, 0xad678846UL, 0xda60b8d0UL,
	0x44042d73UL, 0x33031de5UL, 0xaa0a4c5fUL, 0xdd0d7cc9UL,
	0x5005713cUL, 0x270241aaUL, 0xbe0b1010UL, 0xc90c2086UL,
	0x5768b525UL, 0x206f85b3UL, 0xb966d409UL, 0xce61e49fUL,
	0x5edef90eUL, 0x29d9c998UL, 0xb0d09822UL, 0xc7d7a8b4UL,
	0x59b33d17UL, 0x2eb40d81UL, 0xb7bd5c3bUL, 0xc0ba6cadUL,
	0xedb88320UL, 0x9abfb3b6UL, 0x03b6e20cUL, 0x74b1d29aUL,
	0xead54739UL, 0x9dd277afUL, 0x04db2615UL, 0x73dc1683UL,
	0xe3630b12UL, 0x94643b84UL, 0x0d6d6a3eUL, 0x7a6a5aa8UL,
	0xe40ecf0bUL, 0x9309ff9dUL, 0x0a00ae27UL, 0x7d079eb1UL,
	0xf00f9344UL, 0x8708a3d2UL, 0x1e01f268UL, 0x6906c2feUL,
	0xf762575dUL, 0x806567cbUL, 0x196c3671UL, 0x6e6b06e7UL,
	0xfed41b76UL, 0x89d32be0UL, 0x10da7a5aUL, 0x67dd4accUL,
	0xf9b9df6fUL, 0x8ebeeff9UL, 0x17b7be43UL, 0x60b08ed5UL,
	0xd6d6a3e8UL, 0xa1d1937eUL, 0x38d8c2c4UL, 0x4fdff252UL,
	0xd1bb67f1UL, 0xa6bc5767UL, 0x3fb506ddUL, 0x48b2364bUL,
	0xd80d2bdaUL, 0xaf0a1b4cUL, 0x36034af6UL, 0x41047a60UL,
	0xdf60efc3UL, 0xa867df55UL, 0x316e8eefUL, 0x4669be79UL,
	0xcb61b38cUL, 0xbc66831aUL, 0x256fd2a0UL, 0x5268e236UL,
	0xcc0c7795UL, 0xbb0b4703UL, 0x220216b9UL, 0x5505262fUL,
	0xc5ba3bbeUL, 0xb2bd0b28UL, 0x2bb45a92UL, 0x5cb36a04UL,
	0xc2d7ffa7UL, 0xb5d0cf31UL, 0x2cd99e8bUL, 0x5bdeae1dUL,
	0x9b64c2b0UL, 0xec63f226UL, 0x756aa39cUL, 0x026d930aUL,
	0x9c0906a9UL, 0xeb0e363fUL, 0x72076785UL, 0x05005713UL,
	0x95bf4a82UL, 0xe2b87a14UL, 0x7bb12baeUL, 0x0cb61b38UL,
	0x92d28e9bUL, 0xe5d5be0dUL, 0x7cdcefb7UL, 0x0bdbdf21UL,
	0x86d3d2d4UL, 0xf1d4e242UL, 0x68ddb3f8UL, 0x1fda836eUL,
	0x81be16cdUL, 0xf6b9265bUL, 0x6fb077e1UL, 0x18b74777UL,
	0x88085ae6UL, 0xff0f6a70UL, 0x66063bcaUL, 0x11010b5cUL,
	0x8f659effUL, 0xf862ae69UL, 0x616bffd3UL, 0x166ccf45UL,
	0xa00ae278UL, 0xd70dd2eeUL, 0x4e048354UL, 0x3903b3c2UL,
	0xa7672661UL, 0xd06016f7UL, 0x4969474dUL, 0x3e6e77dbUL,
	0xaed16a4aUL, 0xd9d65adcUL, 0x40df0b66UL, 0x37d83bf0UL,
	0xa9bcae53UL, 0xdebb9ec5UL, 0x47b2cf7fUL, 0x30b5ffe9UL,
	0xbdbdf21cUL, 0xcabac28aUL, 0x53b39330UL, 0x24b4a3a6UL,
	0xbad03605UL, 0xcdd70693UL, 0x54de5729UL, 0x23d967bfUL,
	0xb3667a2eUL, 0xc4614ab8UL, 0x5d681b02UL, 0x2a6f2b94UL,
	0xb40bbe37UL, 0xc30c8ea1UL, 0x5a05df1bUL, 0x2d02ef8dUL
};

#define CRC32(cr, bt) (cr = (cr >> 8) ^ crc32_tab[(cr ^ (bt)) & 0xff])

//----------------------------------------------------------------------
uint32_t crc32(const uint32_t orig, const uint8_t *buf, uint32_t size)
{
uint32_t cr = ~orig;
const uint8_t *p;
uint32_t n;
uint8_t i;

  	for (n = size, p = buf; n != 0;) {
    	if (n >= 8) {
    		for (i = 0; i < 8; i++) CRC32(cr, *(p + i));
    		p += 8;
    		n -= 8;
    	} else {
    		CRC32(cr, *p);
    		p++;
    		n--;
    	}
  	}

  	return ~cr;
}
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
#ifdef SET_LOGGER

void Logger(const char *tag, bool addTime, const char *fmt, ...)
{
#ifdef SET_LOGGER

	va_list args;
	size_t len = MAX_UART_BUF;
	int dl = 0;
	char *buff = &logBuf[0];

	*buff = '\0';
	if (addTime) dl = sec_to_str(buff);

	if (tag) dl += sprintf(buff+strlen(buff), "[%s] ", tag);
	va_start(args, fmt);
	vsnprintf(buff + dl, len - dl, fmt, args);
	logReady = false;
	if (HAL_UART_Transmit_DMA(logPort, (uint8_t *)buff, strlen(buff)) != HAL_OK) {
		devError |= devUART;
		logReady = true;
	} else {
		while (HAL_UART_GetState(logPort) != HAL_UART_STATE_READY) {
			if (HAL_UART_GetState(logPort) == HAL_UART_STATE_BUSY_RX) break;
			HAL_Delay(1);
		}
		//while (!logReady);
	}

	va_end(args);

#endif
}

#endif
//----------------------------------------------------------------------
static void checkSectors()
{
	if (!file_size || !apiAddr) return;

	uint32_t sadr = apiAddr;
	uint32_t eadr = sadr + file_size - 1;
	uint32_t len = 0;

	for (uint8_t i = 0; i < MAX_FLASH_SECTOR; i++) {
		if (allFlash[i].adr >= sadr) {
			if ((allFlash[i].adr + allFlash[i].len - 1) <= eadr) {
				needErase[i] = 1;
				len += allFlash[i].len;
			} else {
				if (file_size < (len +allFlash[i].len)) {
					needErase[i] = 1;
					break;
				}
			}
		}
	}

	for (uint8_t i = 0; i < MAX_FLASH_SECTOR; i++) {
		if (needErase[i]) {
			Logger(__func__, true, "Erase sector #%u%s", i, eol);
#ifdef SET_FW_PROG
			flash_erase(i);
#endif
		}
	}

}
//----------------------------------------------------------------------
int8_t addr2Sector(uint32_t adr)
{
int8_t ret = -1, i;

	for (i = 0; i < MAX_FLASH_SECTOR; i++) {
		uint32_t sadr = allFlash[i].adr;
		uint32_t eadr = sadr + allFlash[i].len;
		if ( (adr >= sadr) && (adr < eadr) ) {
			ret = i;
			break;
		}
	}

	return ret;
}
//----------------------------------------------------------------------
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
	if (bootMode) progDone = true;
}
//----------------------------------------------------------------------
void dmaCB(DMA_HandleTypeDef *dmem)
{
	if (dmem == dmaMem) dmaDone = true;
}
//----------------------------------------------------------------------
void flash_erase(uint8_t sector)
{
uint32_t error = 0;

FLASH_EraseInitTypeDef FLASH_EraseInitStruct =
{
	.TypeErase = FLASH_TYPEERASE_SECTORS,
	.Sector = (uint32_t)sector,
	.NbSectors = 1,
	.VoltageRange = FLASH_VOLTAGE_RANGE_3
};

    if (HAL_FLASH_Unlock() != HAL_OK) {
    	devError |= devFlash;
        return;
    }

    if (HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &error) != HAL_OK) devError |= devFlash;

    HAL_FLASH_Lock();
}
//----------------------------------------------------------------------
uint8_t flash_write(volatile uint32_t *address, uint32_t *data, uint16_t size)
{
	if (HAL_FLASH_Unlock() != HAL_OK) {
		devError |= devFlash;
		return 1;
	}

	for (int i = 0; i < size; i++) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (*address) + (i * 4), data[i]) != HAL_OK) {
			devError |= devFlash;
			HAL_FLASH_Lock();
			return 1;
		}
	}

	HAL_FLASH_Lock();

	return 0;
}
//----------------------------------------------------------------------
uint8_t progBlk(uint32_t faddr, const uint8_t *buf, int len)
{
uint8_t ret = 1;

	int8_t sector = addr2Sector(faddr);
	Logger(__func__, true, "Addr 0x%X location in #%d sector%s", faddr, sector, eol);

	if (sector != -1) {
		uint32_t ts = HAL_GetTick();
		Logger(__func__, true, "Write %u bytes to sector #%d from addr=0x%X...",
								len, sector, (unsigned int)faddr);
#ifdef SET_FW_PROG
		if (!flash_write(&faddr, (uint32_t *)vrem, len / sizeof(uint32_t))) ret = 0;
#else
		ret = 0;
#endif
		if (!ret)
			Logger(NULL, false, "done. %u ms%s", HAL_GetTick() - ts, eol);
		else
			Logger(NULL, false, "done. ret=%u. %u ms (Err=%d)%s", ret, HAL_GetTick() - ts, devError, eol);
	}

	return ret;
}
//----------------------------------------------------------------------
uint32_t doAPI(uint32_t *aRet)
{
uint32_t ret = 0, txRet = *aRet;
uint8_t faza = 0;
uint32_t tim = 0;
bool loop = true;
bool dstop = false;
uint32_t blk_len = MAX_VREM_BUF;
char *uk = NULL;
unsigned int fsize = hdr.len;
unsigned int fcrc = hdr.crc;
unsigned int fadr = hdr.adr;

	action = do_none;

	while (loop) {
		switch (faza) {
			case 0:
				tim = get_tmr10(_30s);
				faza = 1;
	        break;
			case 1:// wait mode string
				if (startAPI) {
					if (apiCmd == apiProg) {// 'prog:1024:0x12cdc0a5'
						action = do_prog;
						if (file_size >= MAX_VREM_BUF) {
							blk_len = MAX_VREM_BUF;
						} else {
							blk_len = file_size % MAX_VREM_BUF;
						}
						if (blk_len > 0) {
							checkSectors();
							apiAddrCur = apiAddr;
							Logger(__func__, true, "FILE_SIZE: 0x%X  FILE_CRC: 0x%X%s",
												(unsigned int)file_size,
												(unsigned int)file_crc,
												eol);
							faza = 2;
							tim = get_tmr10(_5s);
						} else {
							devError |= devProg;
						}
					} else if (apiCmd == apiRead) {
						if (action == do_prog) {
							fsize = file_size;
							fcrc = file_crc;
							fadr = apiAddr;
						} else action |= do_read;
						sprintf(tmp, "ready:0x%08X:0x%08X%s", fsize, fcrc, eol);
						Report(NULL, false, "%s", tmp);
						Logger(__func__, true, "%s", tmp);
						apiAddrCur = fadr;
						apiLen = fsize;
						if (apiLen >= MAX_VREM_BUF) {
							blk_len = MAX_VREM_BUF;
						} else {
							blk_len = apiLen % MAX_VREM_BUF;
						}
						dstop = false;
						blk_cnt = 0;
						faza = 2;
						tim = get_tmr10(_5s);
					} else if (apiCmd == apiStop) {
						//loop = false;
					} else if (apiCmd == apiSwitch) {
						loop = false;
					} else if (apiCmd == apiDone) {
						loop = false;
					} else {
						devError |= devCmd;
					}
				} else if (check_tmr10(tim)) {
					devError |= devTimeout;
				}
	        break;
			case 2:
				if (apiCmd == apiProg) {
					_writePin(GPIOB, MQ_ALARM_Pin, GPIO_PIN_SET);//ON
					Report(NULL, false, "ready%s", eol);
					blkRdy = 0;
					memset(vrem, 0xff, MAX_VREM_BUF);
					if (blk_len < MAX_VREM_BUF) {
						uint32_t plus = blk_len % 4;
						blk_len += plus;
					}
					if (blk_len > MAX_VREM_BUF) blk_len = MAX_VREM_BUF;
					if (HAL_UART_Receive_DMA(uartPort, vrem, blk_len) != HAL_OK) {
						devError |= devUART;
					} else {
						tim = get_tmr10(_10s);
						faza = 3;
					}
				} else if (apiCmd == apiRead) {
					_writePin(GPIOB, MQ_ALARM_Pin, GPIO_PIN_SET);//ON
					Logger(__func__, true, "read_from_flash: addr=0x%X len=%u%s", (unsigned int)apiAddrCur, blk_len, eol);
					dmaDone = 0;
					if (HAL_DMA_Start_IT(dmaMem, apiAddrCur, (uint32_t)vrem, blk_len) != HAL_OK) {
						devError |= devDma;
					} else {
						tim = get_tmr10(_10s);
						faza = 3;
					}
				} else if (apiCmd == apiStop) {
					loop = false;
				} else if (apiCmd == apiDone) {
					loop = false;
				} else if (check_tmr10(tim)) {
					devError |= devTimeout;
				}
			break;
			case 3:
				if (apiCmd == apiProg) {
					if (blkRdy) {
						blkRdy = 0;
						ret += blk_len;
						//   program current recv. block to flash memory (if CRC is OK)
						if (!progBlk(apiAddrCur, vrem, blk_len)) {//prog is OK
							apiAddrCur += blk_len;
							api_crc = crc32(api_crc, vrem, blk_len);
							if (ret < file_size) {
								if ((ret + blk_len) > file_size) {
									blk_len = file_size - ret;
								}
								startAPI = true;
								faza = 2;
								Report(NULL, false, "ready%s", eol);
							} else {//DONE. all data recv.
								HAL_UART_DMAStop(uartPort);
								startAPI = false;
								HAL_UART_Receive_IT(uartPort, &rxByte, 1);
								dstop = true;
								tim = get_tmr10(_10s);
								faza = 1;
							}
						} else {
							devError |= devProg;
						}
						_writePin(GPIOB, MQ_ALARM_Pin, GPIO_PIN_RESET);//OFF
					} else {
						if (check_tmr10(tim)) devError |= devTimeout;
					}
				} else if (apiCmd == apiRead) {
					if (dmaDone) {
						dmaDone = 0;
						uartReady = false;
						if (HAL_UART_Transmit_DMA(uartPort, vrem, blk_len) != HAL_OK) {
							devError |= devUART;
						} else {
							blk_cnt++;
							Logger(__func__, true, "send_to_client: addr=0x%X len=%u blk=%u%s",
									(unsigned int)apiAddrCur, blk_len, blk_cnt, eol);
							tim = get_tmr10(_10s);
							faza = 4;
						}
					} else {
						if (check_tmr10(tim)) devError |= devTimeout;
					}
				} else if (apiCmd == apiStop) {
					loop = false;
				} else if (apiCmd == apiDone) {
					loop = false;
				} else {
					devError |= devCmd;
				}
			break;
			case 4:
				if (apiCmd == apiRead) {
					if (uartReady) {
						uartReady = false;
						blkRdy = 0;
						memset(tmp, 0, sizeof(tmp));
						if (HAL_UART_Receive_DMA(uartPort, (uint8_t *)tmp, (uint16_t)strlen(_next)) != HAL_OK) {
							devError |= devUART;
						} else {
							tim = get_tmr10(_10s);
							faza = 5;
						}
					} else {
						if (check_tmr10(tim)) devError |= devTimeout;
					}
				} else if (apiCmd == apiDone) {
					loop = false;
				} else devError |= devCmd;
			break;
			case 5:
				if (apiCmd == apiRead) {
					if (blkRdy) {
						blkRdy = 0;
						txRet += blk_len;
						apiAddrCur += blk_len;
						apiCrc = crc32(apiCrc, vrem, blk_len);
						//
						_writePin(GPIOB, MQ_ALARM_Pin, GPIO_PIN_RESET);
						if ((uk = strstr(tmp, "\r\n")) != NULL) *uk = '\0';
						Logger(__func__, true, "Received ack '%s' from client%s", tmp, eol);
						//
						if (txRet < hdr.len) {
							if ((txRet + blk_len) > hdr.len) {
								blk_len = hdr.len - txRet;
							}
							faza = 2;
						} else {//DONE. all data transmited
							*aRet = txRet;
							memset(tmp, 0, sizeof(tmp));
							if (HAL_UART_Receive_DMA(uartPort, (uint8_t *)tmp, (uint16_t)strlen(_done)) != HAL_OK) {
								devError |= devUART;
							} else {
								tim = get_tmr10(_6s);
								faza = 6;
							}
						}
					} else {
						if (check_tmr10(tim)) devError |= devTimeout;
					}
				} else if (apiCmd == apiDone) {
					loop = false;
				}
				if (check_tmr10(tim)) devError |= devTimeout;//devError |= devCmd;
			break;
			case 6:
				if (blkRdy) {
					blkRdy = 0;
					uartReady = false;
					HAL_UART_DMAStop(uartPort);
					startAPI = false;
					HAL_UART_Receive_IT(uartPort, &rxByte, 1);
					dstop = true;
					loop = false;
					if ((uk = strstr(tmp, "\r\n")) != NULL) *uk = '\0';
					Logger(__func__, true, "Received ack '%s' from client. Aciton '%s' done.%s",
											tmp, allAction[action], eol);
				}
				else
				if (check_tmr10(tim)) devError |= devTimeout;
			break;
		}
		if (devError) loop = false;
	}

	blkRdy = 1;
	startAPI = false;
	if (!dstop) {
		HAL_UART_DMAStop(uartPort);
		HAL_UART_Receive_IT(uartPort, &rxByte, 1);
	}

	return ret;
}
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------

#endif

