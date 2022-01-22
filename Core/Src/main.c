/* USER CODE BEGIN Header */
/*
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  Проект CO2_sensor :
  I. Реализован средствами разработки STM32CubeIDE и состоит из 3-х частей.
  	  1. BOOTLOADER - загрузчик (занимает сектора 0..1)
  	  2. RELEASE - собственно API (занимает сектора с второго и далее)
  	  3. DEBUG - отладочная версия API (занимает сектора с 0 и далее)
  	  	  Для записи API средствами загрузчика написана утилита,
  	  	  позволяющая менять версию API через последовательный интерфейс UART1 МК
  II. Аппаратные средства проекта :
  	  1. Плата микроконтроллера STM32F411 (New Black Pill board)
  	  2. Датчик уровня углекислого газа в атмосфере MQ135
  	  3. Датчик температуры и влажности SI7021 (для корректировки значения CO2 по температуре и влажности)
  	  4. Круглый TFT ЖК-дисплей GC9A01
  ******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//arm-none-eabi-objcopy -O ihex "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.hex" && arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin" && ls -la | grep "${BuildArtifactFileBaseName}.*"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
/* USER CODE BEGIN PV */


const char *eol      = "\r\n";
const char *_restart = "restart";
const char *_get     = "get";
const char *_readfw  = "read:";
const char *_getVer  = "ver";
const char *_getHdr  = "hdr";


bool bootMode;

UART_HandleTypeDef *uartPort = &huart1;
DMA_HandleTypeDef *dmaMem = &hdma_memtomem_dma2_stream0;

uint32_t mSecCounter = 0;//+10 ms
uint32_t SecCounter = 0;//+1 s
bool restart_flag = false;

uint8_t rxByte;
uint16_t rxInd = 0;
char rxBuf[MAX_UART_BUF];
bool uartReady = true;

uint32_t devError = 0;
char buf[256];

char txBuf[MAX_UART_BUF] = {0};
volatile bool startAPI = false;
uint8_t blkRdy = 1;

char stx[MAX_UART_BUF] = {0};

api_hdr_t hdr;
uint8_t rdMem = rdNone;

#ifdef BOOT_LOADER

//  see bootloader.* files
	#ifdef SET_LOGGER
		UART_HandleTypeDef *logPort = &huart2;
	#endif

#else

#ifdef SET_SPI_DISPLAY
	SPI_HandleTypeDef *lcdPort = &hspi2;
	volatile uint8_t lcdRdy = 1;

	const uint8_t max_colors = 8;
	uint8_t cidx = 0;
	uint16_t allColors[] = {
		BLUE,    //  0x001F
		RED,     //  0xF800
		GREEN,   //  0x07E0
		CYAN,    //  0x07FF
		MAGENTA, //  0xF81F
		YELLOW,  //  0xFFE0
		WHITE,   //  0xFFFF
		BLACK    //  0x0000
	};
#endif

#ifdef RELEASE
	//#pragma LOCATION(version , API_START_ADR + 2048);
	//const char *version = "ver.0.3 13.12.2021 API";
	//const char *version = "ver.0.4 16.12.2021 API";
	//const char *version = "ver.0.5 17.12.2021 API";
	//const char *version = "ver.0.6 19.12.2021 API";
	//const char *version = "ver.0.7 23.12.2021 API";
	//const char *version = "ver.0.8 25.12.2021 API";
	//const char *version = "ver.0.9 26.12.2021 API";
	//const char *version = "ver.0.9.1 29.12.2021 API";
	//const char *version = "ver.0.9.2 02.01.2022 API";
	//const char *version = "ver.0.9.3 03.01.2022 API";
	//const char *version = "ver.0.9.4 05.01.2022 API";
	//const char *version = "ver.0.9.5 06.01.2022 API";
	//const char *version = "ver.1.0 08.01.2022 API";
	//const char *version = "ver.1.1 10.01.2022 API";
	//const char *version = "ver.1.2 12.01.2022 API";// add big chars for font_24f
	//const char *version = "ver.1.3 13.01.2022 API";
	//const char *version = "ver.1.4 14.01.2022 API";// remove BME280 and add SI7021 sensor
	//const char *version = "ver.1.5 15.01.2022 API";//add 'sion' and 'sioff' commands - dump ON/OFF data from si7021
	//const char *version = "ver.1.6 18.01.2022 API";
	//const char *version = "ver.1.6.1 19.01.2022 API";
	//const char *version = "ver.1.6.2 20.01.2022 API";
	//const char *version = "ver.1.6.3 21.01.2022 API";
	const char *version = "ver.1.6.4 22.01.2022 API";
#else
	//const char *version = "ver.0.3 13.12.2021 DEBUG";
	//const char *version = "ver.0.4 16.12.2021 DEBUG";
	//const char *version = "ver.0.5 17.12.2021 DEBUG";
	//const char *version = "ver.0.6 19.12.2021 DEBUG";
	//const char *version = "ver.0.7 23.12.2021 DEBUG";
	//...
	const char *version = "ver.0.9.3 03.01.2022 DEBUG";
#endif

uint32_t fwAdr = 0;
uint16_t fwLen = 0;
uint8_t fwCmd = 0;

#ifdef SET_MQ135
	bool porogMQ = false;
	uint16_t valMQ = 0, lastMQ = 0;
	uint8_t adcValCounter = 0;
	uint16_t adcBuf[MAX_ADC_BUF] = {0};
	bool beginMQ = false;
#endif

#if defined(SET_BME280) || defined(SET_SI7021)
	const uint32_t min_wait_ms = 150;
	const uint32_t max_wait_ms = 1000;

	I2C_HandleTypeDef *portI2C = &hi2c1;
	uint8_t i2c_txReady = 1, i2c_rxReady = 1;
#endif

static evt_t evt_fifo[MAX_FIFO_SIZE] = {evt_empty};
uint8_t rd_evt_adr = 0;
uint8_t wr_evt_adr = 0;
uint8_t wr_evt_err = 0;
uint8_t cnt_evt = 0;
uint8_t max_evt = 0;
bool lock_fifo = false;
evt_t evt = evt_none;
uint8_t cntEvt = 0;

#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

uint32_t get_tmr10(uint32_t ms);
bool check_tmr10(uint32_t ms);
int sec2str(char *st);//int sec_to_str(char *stx);
void errLedOn(bool on);
static char *errName(uint32_t err);
uint8_t Report(const char *tag, bool addTime, const char *fmt, ...);


#ifndef BOOT_LOADER

	void dmaTransferDone(DMA_HandleTypeDef *dmem);
	uint32_t hex2bin(const char *buf, uint8_t len);

	uint8_t getEvtCount();
	void putEvt(evt_t evt);
	evt_t getEvt();

#endif

//--------------------------------------------------------------

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

#ifndef BOOT_LOADER

	#ifdef RELEASE
		__disable_irq();
		SCB->VTOR = API_START_ADR;//0x8008000
		__enable_irq();
	#endif

#endif


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //---------------------------------------------------------

  //---------------------------------------------------------
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(uartPort, &rxByte, 1);
  HAL_TIM_Base_Start_IT(&htim3);

#ifdef BOOT_LOADER

  	bootMode = true;
  	bool noApi = true;
  	HAL_StatusTypeDef dstat = HAL_ERROR;

  	HAL_Delay(1000);


  	Logger(NULL, false, "%s", eol);
  	int vpin = HAL_GPIO_ReadPin(BOOT_PORT_PIN, BOOT_PIN);
  	Logger(name, true, "Start %s with BOOT_PIN=%d%s", version, vpin, eol);

  	//
  	_writePin(GPIOB, ERR_Pin, GPIO_PIN_RESET);
  	_writePin(GPIOB, MQ_ALARM_Pin, GPIO_PIN_SET);
  	for (int i = 0; i < 7; i++) {
  		HAL_Delay(250);
  		_togglePin(GPIOB, ERR_Pin);
  		_togglePin(GPIOB, MQ_ALARM_Pin);
  	}
  	//
  	//------   read API spec area : marker, addr, len, crc32   -----
  	//
  	uint32_t addr = SPEC_AREA_ADR;
  	dmaDone = false;
  	dstat = HAL_DMA_RegisterCallback(dmaMem, HAL_DMA_XFER_CPLT_CB_ID, &dmaCB);
  	if (dstat == HAL_OK) {
  		dmaDone = false;
  		if (HAL_DMA_Start_IT(dmaMem, addr, (uint32_t)&hdr, sizeof(api_hdr_t)) != HAL_OK) devError |= devDma;
  		else {
  			uint8_t sch = 100;
  			while(!dmaDone && --sch) HAL_Delay(1);
  			if (!sch && !dmaDone) devError |= devDma;
  		}
    } else devError |= devDma;

    if (devError) {
    	if (dstat == HAL_OK) HAL_DMA_UnRegisterCallback(dmaMem, HAL_DMA_XFER_CPLT_CB_ID);
    	for (int i = 0; i < 7; i++) {
    		HAL_Delay(150);
    		_togglePin(GPIOB, ERR_Pin);
    		_togglePin(GPIOB, MQ_ALARM_Pin);
    	}
    	LOOP_FOREVER();
    } else {
    	Logger(name, true, "Hdr(0x%X): mark=0x%X adr=0x%X len=0x%X crc=0x%X%s",
        					(unsigned int)addr,
    						(unsigned int)hdr.mark,
    						(unsigned int)hdr.adr,
    						(unsigned int)hdr.len,
    						(unsigned int)hdr.crc,
    						eol);
    }

    if (vpin == GPIO_PIN_SET) {
    	if ( (hdr.mark == API_MARKER) && (hdr.adr == API_START_ADR) && hdr.len && hdr.crc ) {
    		noApi = false;
    	}
    }
    //
    //
    //
    if (noApi) {
    	//
    	vrem = (uint8_t *)calloc(1, MAX_VREM_BUF);//get memory for data buffer
    	if (!vrem) {
    		devError |= devMem;
    		_writePin(GPIOB, ERR_Pin, GPIO_PIN_RESET);
    		_writePin(GPIOB, MQ_ALARM_Pin, GPIO_PIN_SET);
    		LOOP_FOREVER();
    	}
    	//
    	//---------------------------------------------------------------
    	//
    	uint32_t the_len = doAPI(&fLen);

    	if (vrem) free(vrem);

    	if (the_len || fLen) {

    		if (apiCmd != apiSwitch) HAL_Delay(1500);

    		if (action == do_prog) {
    			apiHdr.mark = API_MARKER;
    			apiHdr.adr = API_START_ADR;
    			apiHdr.len = file_size;
    			apiHdr.crc = file_crc;

    			api_hdr_t *Hdr = &hdr;
    			if (dstat == HAL_OK) {
    				if (apiHdr.len && apiHdr.crc) {
    					if ( (hdr.mark != apiHdr.mark) || (hdr.adr != apiHdr.adr) ||
    								(hdr.len != apiHdr.len) || (hdr.crc != apiHdr.crc) ) {
    						Logger(name, true, "SPEC_AREA not empty -> erase sector #7...");
    						flash_erase(7);
    						if (!devError)
    							Logger(NULL, false, " done.%s", eol);
    						else
    							Logger(NULL, false, " done.(error:'%s')%s", errName(devError), eol);
    						Hdr = &apiHdr;
    						flash_write(&addr, (uint32_t *)&Hdr->mark, sizeof(api_hdr_t)/sizeof(uint32_t));
    					}
    				}
    				Logger(name, true, "Hdr(0x%X): mark=0x%X adr=0x%X len=0x%X crc=0x%X%s",
    									(unsigned int)addr,
										(unsigned int)Hdr->mark,
										(unsigned int)Hdr->adr,
										(unsigned int)Hdr->len,
										(unsigned int)Hdr->crc,
										eol);
    			}
    		}

    	}

    	errLedOn(devError ? true : false);
    	if (devError) {
    		Logger(name, true, "action '%s': Error #%lu '%s'%s", allAction[action], devError, errName(devError), eol);
    		HAL_Delay(1000);
    	} else {
    		for (int i = 0; i < 6; i++) {
    			HAL_Delay(250);
    			_togglePin(GPIOB, ERR_Pin);
    			_togglePin(GPIOB, MQ_ALARM_Pin);
    		}
    	}

  	}

    if (dstat == HAL_OK) HAL_DMA_UnRegisterCallback(dmaMem, HAL_DMA_XFER_CPLT_CB_ID);

    Logger(name, true, "Switch to application by address 0x%X%s", (unsigned int)hdr.adr, eol);


    //HAL_UART_MspDeInit(uartPort);
    //HAL_TIM_Base_MspDeInit(&htim3);
    //HAL_RCC_DeInit();
    //HAL_DeInit();


    uint32_t appJump = *((volatile uint32_t *)(API_START_ADR + 4));

    void (*GoToApp)(void);
    GoToApp = (void (*)(void))appJump;

    __disable_irq();
    __set_MSP(*((volatile uint32_t *)API_START_ADR));

    GoToApp();//switch to API


#else

    bootMode = false;

#ifdef SET_MQ135
    HAL_ADC_Start_IT(&hadc1);
#endif

    Report(NULL, true, "%s Start in fifo_event_loop mode%s", version, eol);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	HAL_StatusTypeDef dmaMemStatus = HAL_DMA_RegisterCallback(dmaMem, HAL_DMA_XFER_CPLT_CB_ID, &dmaTransferDone);

	strcpy(stx, "Register callback function 'dmaTransferDone()' ");
	if (dmaMemStatus == HAL_OK) strcat(stx, "OK");
                           else strcat(stx, "Error");
	Report(NULL, true, "%s%s", stx, eol);



#ifdef SET_SPI_DISPLAY
	char bufik[64];
	int x, y = 96;
	uint32_t lastErr = devError;

	uint16_t DW = 240, DH = 240;
	dispcolor_Init(DW, DH);

	uint16_t dw = GC9A01A_GetWidth();

	uint16_t radius = 118;

	uint8_t fh16 = font_GetCharHeight(font_GetFontStruct(FONTID_16F, '0'));
	//uint8_t fh24 = font_GetCharHeight(font_GetFontStruct(FONTID_24F, '0'));
	uint8_t fh32 = font_GetCharHeight(font_GetFontStruct(FONTID_32F, '0'));
#endif

#ifdef SET_MQ135
	float rzero = MQ135_getRZero();
	Report(__func__, true, "MQ135 RZERO first calibration value : %.2f kOhm%s", rzero, eol);
	float ppm = MQ135_getPPM();
#endif
	//
#if defined(SET_BME280)
	//
	evt_t evt_sens = evt_bmpBegin;
	const uint32_t wait_next = _50ms;
	bool sensReady = false;
	uint16_t d_size = 6;
	int32_t temp = 0, pres = 0, humi = 0, tmr_sens = 0;
	uint8_t data_rdx[DATA_LENGTH] = {0};
	char sensName[8] = {0};
	uint32_t sch = 0;
	int8_t schet = -1;
	//
	if (i2c_master_reset_sensor(&regs.id) == HAL_OK) {
		switch (regs.id) {
			case BMP280_SENSOR :
				strcpy(sensName, "BMP280");
			break;
			case BME280_SENSOR :
				d_size = 8;
				strcpy(sensName, "BME280");
			break;
		}
	}
	if (regs.id) {
		if (strlen(sensName)) Report(NULL, true, "BMx280 chip '%s'(0x%X)%s", sensName, regs.id, eol);
		tmr_sens = get_tmr10(wait_next);
	} else {
		putEvt(evt_bmpNone);
	}
	//
#elif defined(SET_SI7021)
	bool sensReady = false;
	uint32_t sch = 0;
	uint8_t schet = 0;
	const uint32_t wait_next = _50ms;
	const char *sensName = "SI7021";

	uint32_t tmr_sens = get_tmr10(wait_next);
	evt_t evt_sens = evt_siReset;
#endif



	while (!restart_flag) {

	/* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


#if defined(SET_BME280) || defined(SET_SI7021)
		if (tmr_sens) {
			if (check_tmr10(tmr_sens)) {
				tmr_sens = 0;
				beginMQ = true;
				putEvt(evt_sens);
			}
		}
#endif


		evt = getEvt();
		cntEvt = getEvtCount();

		switch ((int)evt) {
			case evt_errCmd:
				Report(NULL, true, "[que:%u] Error cmd enter !%s", cntEvt, eol);
			break;
#if defined(SET_BME280)
			case evt_bmpNone:
				Report(NULL, true, "[que:%u] BMx280 Not Present !%s", cntEvt, eol);
			break;
			case evt_bmpBegin:
				if (i2c_master_test_sensor(&regs) == HAL_OK) {
					evt_sens = evt_bmpNext;
					tmr_sens = get_tmr10(wait_next);
				} else {
					devError |= devI2C;
				}
			break;
			case evt_bmpNext:
				regs.stat &= 0x0f;
				memset(data_rdx, 0, DATA_LENGTH);
				if (i2c_master_read_sensor(BMP280_REG_PRESSURE, data_rdx, d_size) == HAL_OK) {
					if (bmx280_readCalibrationData(regs.id) == HAL_OK) {
						evt_sens = evt_bmpEnd;
						tmr_sens = get_tmr10(wait_next);
					} else devError |= devI2C;
				} else devError |= devI2C;
			break;
			case evt_bmpEnd:
				pres = temp = 0; humi = 0;
				pres = (data_rdx[0] << 12) | (data_rdx[1] << 4) | (data_rdx[2] >> 4);
				temp = (data_rdx[3] << 12) | (data_rdx[4] << 4) | (data_rdx[5] >> 4);
				if (regs.id == BME280_SENSOR) humi = (data_rdx[6] << 8) | data_rdx[7];
				bmx280_CalcAll(&sens, regs.id, temp, pres, humi);
				sensReady = true;
				evt_sens = evt_bmpPrn;
				tmr_sens = get_tmr10(wait_next);
			break;
			case evt_bmpPrn:
				sch++;
				if (sch == 8) {
					sch = 0;
					sprintf(stx, "[que:%u] press=%.2f temp=%.2f humi=%.2f ppm=%d",
							cntEvt,
							sens.pres,
							sens.temp,
							sens.humi,
							(int)ppm);
					Report(NULL, true, "%s%s", stx, eol);
				}
				if (schet == 1) {
					sprintf(bufik, "Press : %.2f mmHg ", sens.pres);
					uint8_t step = 8 + fh32;
					dispcolor_DrawString(40, y + step, FONTID_16F, bufik, MAGENTA);
					sprintf(bufik, "Humi : %.2f%% ", sens.humi);
					step += fh16 + 4;
					dispcolor_DrawString(64, y + step, FONTID_16F, bufik, YELLOW);
					sprintf(bufik, "Temp : %.2f%c ", sens.temp, 0xb0);
					step += fh16 + 4;
					dispcolor_DrawString(64, y + step, FONTID_16F, bufik, CYAN);
				}
				evt_sens = evt_bmpBegin;
				schet++;
				schet &= 1;
				tmr_sens = get_tmr10(_1s25);
			break;
#elif defined(SET_SI7021)
			case evt_siReset:
				si7021_reset();

				if (devError & devI2C) {
					evt_sens = evt_siReset;
					tmr_sens = get_tmr10(_1s25);
				} else {
					evt_sens = evt_siReadT;
					tmr_sens = get_tmr10(wait_next);
				}
			break;
			case evt_siReadT:
				si7021_read_temp();

				if (devError & devI2C) {
					evt_sens = evt_siReset;
					tmr_sens = get_tmr10(_1s25);
				} else {
					evt_sens = evt_siReadH;
					tmr_sens = get_tmr10(wait_next);
				}
			break;
			case evt_siReadH:
				si7021_read_humi();
				sensReady = true;

				if (devError & devI2C) {
					evt_sens = evt_siReset;
					tmr_sens = get_tmr10(_1s25);
				} else {
					evt_sens = evt_siPrn;
					tmr_sens = get_tmr10(wait_next);
				}
			break;
			case evt_siPrn:
				sch++;
				if (sch == 8) {
					sch = 0;
					sprintf(stx, "[que:%u] temp=%.2f humi=%.2f ppm=%d",
							cntEvt,
							sens.temp,
							sens.humi,
							(int)ppm);
					Report(NULL, true, "%s%s", stx, eol);
				}
				if (!schet) {
					uint16_t cvet = MAGENTA;
					if (sens.humi <= 0) cvet = RED;
					sprintf(bufik, "Humi : %.2f%% ", sens.humi);
					uint8_t step = 8 + fh32;
					dispcolor_DrawString(64, y + step, FONTID_16F, bufik, cvet);
					sprintf(bufik, "Temp : %.2f%c ", sens.temp, 0xb0);
					if (sens.temp > 0) cvet = CYAN;
					step += fh16 + 4;
					dispcolor_DrawString(64, y + step, FONTID_16F, bufik, cvet);
				}

				if (devError & devI2C)
					evt_sens = evt_siReset;
				else
					evt_sens = evt_siReadT;
				schet++;
				schet &= 1;
				if (si_dump) {
					Report(NULL, true,
							"humi: val=0x%02X%02X crc=0x%02X/0x%02X "
							"temp: val=0x%02X%02X crc=0x%02X/0x%02X%s",
							si_ack[0], si_ack[1], si_ack[2], si_ack[3],
							si_ack[4], si_ack[5], si_ack[6], si_ack[7],
							eol);
				}
				tmr_sens = get_tmr10(_1s25);
			break;
#endif
	  		case evt_rst:
	  			restart_flag = true;
	  		break;
	  		case evt_sec:
	  		{
#ifdef SET_SWV
	  			puts("Second...");
	  			printf("[que:%u] MQ135: %u 0x%X%s", cntEvt, valMQ, valMQ, eol);
#endif
#ifdef SET_MQ135
	  			if (valMQ != lastMQ) {
	  				lastMQ = valMQ;
	  				if (sensReady) {
	  					//rzero = MQ135_getRZero();
	  					//ppm = MQ135_getPPM();
	  					rzero = MQ135_getCorrectedRZero(sens.temp, sens.humi);
	  					ppm = MQ135_getCorrectedPPM(sens.temp, sens.humi);
	  					sprintf(bufik, "CO2 : %.3f%% ", ppm / 1000);
	  					dispcolor_DrawString(56, y - fh32, FONTID_24F, bufik, ppmColor(ppm));
	  				}
	  			}
#endif
	  			//
	  			x = (dw - (sec2str(stx) * 16));
	  			dispcolor_DrawString(x - 8, y, FONTID_32F, stx, RGB565(255, 255, 255));
	  			//
	  			dispcolor_DrawCircle((dw >> 1) - 1, (dw >> 1) - 1, radius, allColors[cidx++], 0);
	  			if (cidx >= max_colors) cidx = 0;
	  		}
	  		break;
#ifdef SET_MQ135
	  		case evt_intrMQ:
	  			HAL_ADC_Start_IT(&hadc1);
	  		break;
	  		case evt_porogMQ:
	  			if (porogMQ)
	  				Report(NULL, true, "[que:%u] MQ135: alarmMQ signal (%lu/%d)%s", cntEvt, valMQ, (int)ppm, eol);
	  			else
	  				Report(NULL, true, "[que:%u] MQ135: clearMQ signal (%lu/%d)%s", cntEvt, valMQ, (int)ppm, eol);
	  		break;
	  		case evt_getMQ:
	#if defined(SET_BME280)
	  			Report(NULL, true,  "[que:%u] MQ135: adc=%lu ppm=%d, "
	  								"%s: press=%.2f temp=%.2f humi=%.2f%s",
									cntEvt, valMQ, (int)ppm,
									sensName, sens.pres, sens.humi, sens.temp,
									eol);
	#elif defined(SET_SI7021)
	  			Report(NULL, true,  "[que:%u] MQ135: adc=%lu ppm=%d, "
	  								"%s: temp=%.2f humi=%.2f%s",
									cntEvt, valMQ, (int)ppm,
									sensName, sens.temp, sens.humi,
									eol);
	#endif
	  		break;
#endif
	  		case evt_getVer:
	  			Report(NULL, true, "[que:%u] %s%s", cntEvt, version, eol);
	  		break;
	  		case evt_getHdr:
	  			if (dmaMemStatus == HAL_OK) {
	  				rdMem = rdHdr;
	  				if (HAL_DMA_Start_IT(dmaMem, (uint32_t)SPEC_AREA_ADR, (uint32_t)&hdr, SPEC_AREA_LEN) != HAL_OK) devError |= devDma;
	  			}
	  		break;
	  		case evt_dmaHdr:
	  			Report(NULL, true, "[que:%u] Hdr(0x%X): mark=0x%X adr=0x%X len=0x%X crc=0x%X%s",
	  								cntEvt,
									(unsigned int)SPEC_AREA_ADR,
									(unsigned int)hdr.mark,
									(unsigned int)hdr.adr,
									(unsigned int)hdr.len,
									(unsigned int)hdr.crc,
									eol);
	  		break;
	  		case evt_readFW:
	  			if (dmaMemStatus == HAL_OK) {
	  				Report(NULL, true, "Read flash: adr=0x%X len=%u %s", fwAdr, fwLen, eol);
	  				rdMem = rdApi;
	  				memset(buf, 0, sizeof(buf));
	  				if (HAL_DMA_Start_IT(dmaMem, fwAdr, (uint32_t)buf, fwLen) != HAL_OK) devError |= devDma;
	  			}
	  		break;
	  		case evt_dmaMem:
	  		{
	  			uint32_t adr = fwAdr;
	  			int step = 32;
	  			uint32_t ind = 0;
	  			bool done = false;
	  			while (ind < fwLen) {
	  				stx[0] = '\0';
	  				while (1) {
	  					sprintf(stx+strlen(stx), "%08X ", (unsigned int)adr);
	  					for (int i = 0; i < step; i++) {
	  						if ((i + ind) < fwLen) sprintf(stx+strlen(stx), " %02X", buf[i + ind]);
	  						else {
	  							done = true;
	  							break;
	  						}
	  					}
	  					strcat(stx, "\n");//eol);
	  					adr += step;
	  					ind += step;
	  					if (!(ind % fwLen)) break;
	  					if (done) break;
	  				}
	  				Report(NULL, false, "%s%s", stx, eol);
	  			}
	  		}
	  		break;
	  		case evt_errInput:
	  			Report(NULL, true, "Read flash: error input !%s", eol);
	  		break;
		}
		//
		//
		//
		errLedOn(devError ? true : false);
		if (devError) {
			if (lastErr != devError) {
				/*
				x = (dw - sprintf(bufik, "'%lu'", devError)) >> 2;
				gc9a01_text(&obj, &fontik, bufik, x, y + (fontik.glyph->height << 2), RED, BLACK);
				*/
				lastErr = devError;
				Report(NULL, true, "[que:%u] err:'%s'%s", cntEvt, errName(devError), eol);
			}
		}
		//
		//
		//
		HAL_Delay(1);//10
	}

	Report(NULL, true, "[que:%u] Restart...%s", cntEvt, eol);
	HAL_Delay(1000);
	NVIC_SystemReset();

#endif

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
#ifndef BOOT_LOADER
#ifdef SET_MQ135
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
#endif
#endif
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
#if defined(SET_BME280) || defined(SET_SI7021)
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
#endif
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
#ifndef BOOT_LOADER
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
#endif
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
//#ifndef BOOT_LOADER
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
//#endif
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
//#ifndef BOOT_LOADER
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
//#endif
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
#ifdef BOOT_LOADER
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
#endif
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|MQ_ALARM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ERR_Pin|LCD_DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT_Pin */
  GPIO_InitStruct.Pin = BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOOT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : digMQ_Pin */
  GPIO_InitStruct.Pin = digMQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(digMQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ERR_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = ERR_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MQ_ALARM_Pin */
  GPIO_InitStruct.Pin = MQ_ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MQ_ALARM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
static char *errName(uint32_t err)
{
	switch (err) {
	    case 0:
	    	return "None";
		case devUART:// = 1,
			return "UART";
		case devADC:
			return "ADC";
		case devFifo:
			return "Fifo";
		case devQue:
			return "Que";
		case devMem:
			return "Mem";
		case devTimeout:
			return "Timeout";
		case devBreak:
			return "apiStop";
		case devProg:
			return "apiProg";
		case devRead:
			return "apiRead";
		case devCmd:
			return "apiCmd";
		case devDma:
			return "dmaMem";
		case devFlash:
			return "devFlash";
		case devAck:
			return "devAck";
		case devLCD:
			return "devLCD";
		case devI2C:
			return "devI2C";
#ifdef SET_SI7021
		case devCRC:
			return "devCRC";
#endif
	}
	return "???";
}
//-----------------------------------------------------------------------------
//      Функция преобразует hex-строку в бинарное число типа uint32_t
//
uint32_t hex2bin(const char *buf, uint8_t len)
{
uint8_t i, j, jk, k;
uint8_t mas[8] = {0x30}, bt[2] = {0};
uint32_t dword, ret = 0;

    if (!len || !buf) return ret;
    if (len > 8) len = 8;
    k = 8 - len;
    memcpy(&mas[k], buf, len);

    k = j = 0;
    while (k < 4) {
        jk = j + 2;
        for (i = j; i < jk; i++) {
                 if ((mas[i] >= 0x30) && (mas[i] <= 0x39)) bt[i&1] = mas[i] - 0x30;
            else if ((mas[i] >= 0x61) && (mas[i] <= 0x66)) bt[i&1] = mas[i] - 0x57;//a,b,c,d,e,f
            else if ((mas[i] >= 0x41) && (mas[i] <= 0x46)) bt[i&1] = mas[i] - 0x37;//A,B,C,D,E,F
        }
        dword = (bt[0] << 4) | (bt[1] & 0xf);
        ret |= (dword << 8*(4 - k - 1));
        k++;
        j += 2;
    }

    return ret;
}
//-------------------------------------------------------------------------------------------
uint32_t get_mSecCounter()
{
	return mSecCounter;
}
//----------------------------------------------
void inc_mSecCounter()
{
	mSecCounter++;
}
//----------------------------------------------
uint32_t get_secCounter()
{
	return SecCounter;
}
//----------------------------------------------
void set_secCounter(uint32_t sec)
{
	SecCounter = sec;
}
//----------------------------------------------
void inc_secCounter()
{
	SecCounter++;
}
//----------------------------------------------
uint32_t get_tmr10(uint32_t ms)
{
	return (get_mSecCounter() + ms);
}
//----------------------------------------------
bool check_tmr10(uint32_t ms)
{
	return (get_mSecCounter() >= ms ? true : false);
}
//-------------------------------------------------------------------------------------------
void errLedOn(bool on)
{
	HAL_GPIO_WritePin(ERR_GPIO_Port, ERR_Pin, GPIO_PIN_SET);//LED OFF
	HAL_Delay(25);
	if (on) HAL_GPIO_WritePin(ERR_GPIO_Port, ERR_Pin, GPIO_PIN_RESET);//LED ON
}
//-----------------------------------------------------------------------------------------
int sec2str(char *st)
{
	uint32_t sec = get_secCounter();

	uint32_t day = sec / (60 * 60 * 24);
	sec %= (60 * 60 * 24);
	uint32_t hour = sec / (60 * 60);
	sec %= (60 * 60);
	uint32_t min = sec / (60);
	sec %= 60;

	return (sprintf(st, "%lu.%02lu:%02lu:%02lu ", day, hour, min, sec));
}
//-----------------------------------------------------------------------------------------
/*
int sec_to_str(char *stx)
{
	uint32_t sec = get_secCounter();

	uint32_t day = sec / (60 * 60 * 24);
	sec %= (60 * 60 * 24);
	uint32_t hour = sec / (60 * 60);
	sec %= (60 * 60);
	uint32_t min = sec / (60);
	sec %= 60;

	return (sprintf(stx, "%lu.%02lu:%02lu:%02lu | ", day, hour, min, sec));
}
*/
//-------------------------------------------------------------------------------------------
uint8_t Report(const char *tag, bool addTime, const char *fmt, ...)
{
va_list args;
size_t len = MAX_UART_BUF;
int dl = 0;
char *buff = &txBuf[0];

	*buff = '\0';
	if (addTime) {
		dl = sec2str(buff);
		strcat(buff, "| ");
		dl += 2;
	}

	if (tag) dl += sprintf(buff+strlen(buff), "[%s] ", tag);
	va_start(args, fmt);
	vsnprintf(buff + dl, len - dl, fmt, args);
	uartReady = false;
	if (HAL_UART_Transmit_DMA(uartPort, (uint8_t *)buff, strlen(buff)) != HAL_OK) devError |= devUART;
	while (HAL_UART_GetState(uartPort) != HAL_UART_STATE_READY) {
		if (HAL_UART_GetState(uartPort) == HAL_UART_STATE_BUSY_RX) break;
		HAL_Delay(1);
	}
	va_end(args);


	return 0;
}

//-------------------------------------------------------------------------------------------

#ifndef BOOT_LOADER
//*******************************************************************************************

//-------------------------------------------------------------------------------------------
void dmaTransferDone(DMA_HandleTypeDef *dmem)
{
	if (dmem == dmaMem) {
		evt_t ev = evt_none;

		if (rdMem == rdApi) ev = evt_dmaMem;
		else
		if (rdMem == rdHdr) ev = evt_dmaHdr;

		if (ev != evt_none) putEvt(ev);
	}
}
//-------------------------------------------------------------------------------------------
uint8_t getEvtCount()
{
	return cnt_evt;
}
//-------------------------------------------------------------------------------------------
void putEvt(evt_t evt)
{

	//while (lock_fifo);
	//lock_fifo = true;

	if (cnt_evt > (MAX_FIFO_SIZE - 3)) {
		devError |= devFifo;
		//lock_fifo = false;
		return;
	}

	HAL_NVIC_DisableIRQ(ADC_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	HAL_NVIC_DisableIRQ(TIM3_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);

	if (cnt_evt >= MAX_FIFO_SIZE) {
		wr_evt_err++;
	} else {
		evt_fifo[wr_evt_adr] = evt;
		cnt_evt++;
		if (wr_evt_adr < (MAX_FIFO_SIZE - 1) ) {
			wr_evt_adr++;
		} else  {
			wr_evt_adr = 0;
		}
		wr_evt_err = 0;
		if (cnt_evt > max_evt) max_evt = cnt_evt;
	}

	if (wr_evt_err) devError |= devFifo;
			   else devError &= ~devFifo;

	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(ADC_IRQn);

	//lock_fifo = false;
}
//
evt_t getEvt()
{
evt_t ret = evt_empty;

	//while (lock_fifo);
	//lock_fifo = true;

	HAL_NVIC_DisableIRQ(ADC_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	HAL_NVIC_DisableIRQ(TIM3_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);

	if (cnt_evt) {
		ret = evt_fifo[rd_evt_adr];
		if (cnt_evt) cnt_evt--;
		if (rd_evt_adr < (MAX_FIFO_SIZE - 1) ) {
			rd_evt_adr++;
		} else {
			rd_evt_adr = 0;
		}
	}

	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(ADC_IRQn);

	//lock_fifo = false;

	return ret;
}
//----------------------------------------------------------------------------------------

#ifdef SET_MQ135

//----------------------------------------------------------------------------------------
//   Функция добавляет в кольцевой буфер очередной замер,
//    полученный при оцифровке данных с аналогового выхода MQ135
//
uint8_t adcAddVal(uint16_t value)
{
int8_t i;

	if (!adcValCounter) {
		for (i = 0; i < MAX_ADC_BUF; i++) adcBuf[i] = value;
		adcValCounter = MAX_ADC_BUF;
	} else {
		for (i = MAX_ADC_BUF - 2; i >= 0; i--) adcBuf[i + 1] = adcBuf[i];
		adcBuf[0] = value;
		if (adcValCounter < MAX_ADC_BUF) adcValCounter++;
	}
    return adcValCounter;
}
//-----------------------------------------------------------------------------------------
//   CallBack функция, вызывается по завершении прерывания
//    по изменению уровня на цифровом выходе MQ135
//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if ((GPIO_Pin != digMQ_Pin) || !beginMQ) return;

	GPIO_PinState led;

	if (HAL_GPIO_ReadPin(digMQ_GPIO_Port, digMQ_Pin) == GPIO_PIN_RESET) {
		led = GPIO_PIN_SET;//ON
		porogMQ = true;
	} else {
		led = GPIO_PIN_RESET;//OFF
		porogMQ = false;
	}
	HAL_GPIO_WritePin(MQ_ALARM_GPIO_Port, MQ_ALARM_Pin, led);

	putEvt(evt_porogMQ);
}
//-----------------------------------------------------------------------------------------
//   Функция вызывается по готовности очередного оцифрованного значения замеров
//    с аналогового выхода MQ135. Здесь же выполняется фильтрация полученных
//    замеров в скользящем окне 'шириной' в MAX_ADC_BUF замеров.
//    Результат фильтрации помещается в глобальную переменную valMQ.
//
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1) {
		uint16_t val = (uint16_t)HAL_ADC_GetValue(hadc);
		//
		if (adcAddVal(val) == MAX_ADC_BUF) {//в окне накоплено MAX_ADC_BUF выборок -> фильтрация !
			uint32_t sum = 0;
			for (int8_t j = 0; j < MAX_ADC_BUF; j++) sum += adcBuf[j];
			valMQ = sum / MAX_ADC_BUF;
		}
		//
		putEvt(evt_intrMQ);
	}
}
//-----------------------------------------------------------------------------------------
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1) devError |= devADC;
}
//-----------------------------------------------------------------------------------------

#endif

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

#ifdef SET_SPI_DISPLAY
//-----------------------------------------------------------------------
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2) lcdRdy = 1;
}
//-----------------------------------------------------------------------
#endif

//-----------------------------------------------------------------------

//*******************************************************************************************
#endif

//-----------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart->Instance == USART1) {
		//
		if (!startAPI) {
			if (rxByte != BACK_SPACE) {
				rxBuf[rxInd & 0x3ff] = (char)rxByte;
			} else {
				if (rxInd) rxInd--;
			}
		}
#ifdef BOOT_LOADER
		else {//DMA mode
			blkRdy = 1;
			return;
		}
#endif
		//
		if (rxByte == 0x0a) {//end of line
			rxBuf[rxInd] = '\0';
			if (rxInd > 0) if (rxBuf[rxInd - 1] == '\r') rxBuf[rxInd - 1] = '\0';
#ifdef BOOT_LOADER
			char *ui = NULL;
			if ((ui = strstr(rxBuf, _progfw))) {//const char *_progfw = "prog:17308:0x4549ABBB";
				if (ui == rxBuf) {
					ui += strlen(_progfw);
					char *uki = strchr(ui, ':');
					if (uki) {
						char *ucrc = strstr(uki, "0x");
						if (ucrc) {
							ucrc += 2;
							file_crc = hex2bin(ucrc, strlen(ucrc));
						} else {
							file_crc = atol(ucrc);
						}
						*uki = '\0';
						if (strstr(ui, "0x")) {
							ui += 2;
							file_size = hex2bin(ui, strlen(ui));
						} else {
							file_size = atoi(ui);
						}
						if (file_size < MAX_API_SIZE) {
							if (!startAPI) {
								startAPI = true;
								apiCmd = apiProg;
							}
						} else file_size = 0;
					}
				}
			} else if ((ui = strstr(rxBuf, _getfw))) {//const char *_getfw = "read\r\n";
				if (ui == rxBuf) {
					if (!startAPI) {
						startAPI = true;
						apiCmd = apiRead;
					}
				}
			} else if ((ui = strstr(rxBuf, _apiStop))) {
				if (ui == rxBuf) {
					if (!startAPI) {
						startAPI = true;
						apiCmd = apiStop;
					}
				}
			} else if ((ui = strstr(rxBuf, _switch))) {
				if (ui == rxBuf) {
					if (!startAPI) {
						startAPI = true;
						apiCmd = apiSwitch;
					}
				}
			} else if ((ui = strstr(rxBuf, _next))) {
				if (ui == rxBuf) {
					if (!startAPI) startAPI = true;
				}
			} else if ((ui = strstr(rxBuf, _done))) {
				if (ui == rxBuf) {
					startAPI = true;
					apiCmd = apiDone;
				}
			}
#else
			//    parse input string
			if (!strncmp(rxBuf, _restart, strlen(_restart))) {//const char *_restart = "restart";
				if (!restart_flag) {
					restart_flag = 1;
					return;
				}
			}
#ifdef SET_MQ135
			else if (!strncmp(rxBuf, _get, strlen(_get))) {//const char *_get = "get";
				putEvt(evt_getMQ);
			}
#endif
#ifdef SET_SI7021
			else if (!strncmp(rxBuf, _siON, strlen(_siON))) {//const char *_siON = "sion";
				si_dump = true;
			} else if (!strncmp(rxBuf, _siOFF, strlen(_siOFF))) {//const char *_siOFF = "sioff";
				si_dump = false;
			}
#endif
			else if (!strncmp(rxBuf, _getHdr, strlen(_getHdr))) {//const char *_getHdr = "hdr";
				putEvt(evt_getHdr);
			} else if (!strncmp(rxBuf, _getVer, strlen(_getVer))) {//const char *_getVer = "ver";
				putEvt(evt_getVer);
			} else if (!strncmp(rxBuf, _readfw, strlen(_readfw))) {//const char *_readfw = "read:adr:len";
				evt_t ev = evt_errInput;
				char *ua = rxBuf + strlen(_readfw);
				char *ul = strchr(ua, ':');
				if (ul) {
					fwLen = atol(ul + 1);
					if (fwLen > PAGE_SIZE) fwLen = PAGE_SIZE;
				    *ul = '\0';
				} else {
					fwLen = PAGE_SIZE;
				}
				if (ua) {
					fwAdr = 0;
					if (strstr(ua, "0x")) {
					    ua += 2;
					    fwAdr = hex2bin(ua, strlen(ua));
					} else {
					    fwAdr = atol(ua);
					}
					if (fwLen) {
						if ( ((fwAdr >= FLASH_ADDR) && (fwAdr < (FLASH_ADDR + FLASH_LEN))) ) {
							if ((fwAdr + fwLen) > (FLASH_ADDR + FLASH_LEN))
								fwLen = (FLASH_ADDR + FLASH_LEN) - fwAdr;
							ev = evt_readFW;
						} else if ( (fwAdr >= OTP_AREA_BEGIN) && (fwAdr <= OTP_AREA_END) ) {
							if ((fwAdr + fwLen) > OTP_AREA_END) fwLen = OTP_AREA_END - fwAdr;
							ev = evt_readFW;
						} else if ( (fwAdr >= OTP_LOCK_BEGIN) && (fwAdr <= OTP_LOCK_END) ) {
							fwLen = OTP_LOCK_SIZE;
							ev = evt_readFW;
						}
					}
				}
				putEvt(ev);
			} else {
				putEvt(evt_errCmd);
			}

#endif
			rxInd = 0;
			memset(rxBuf, 0, sizeof(rxBuf));
		} else rxInd++;
		//
		if (!startAPI) HAL_UART_Receive_IT(huart, &rxByte, 1);
		//
	}
}
//-----------------------------------------------------------------------
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if ((huart->Instance == USART1) || (huart->Instance == USART2)) devError |= devUART;
}
//-----------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) uartReady = true;
#ifdef SET_LOGGER
	else
	if (huart->Instance == USART2) logReady = true;
#endif
}

//-----------------------------------------------------------------------------

#if defined(SET_BME280) || defined(SET_SI7021)

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) devError |= devI2C;
}
//-----------------------------------------------------------------------------
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) {
		i2c_txReady = 1;
	}
}
//-----------------------------------------------------------------------------
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) {
		i2c_rxReady = 1;
	}
}

#endif

//-----------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3) {// период срабатывания 10 мсек.
		inc_mSecCounter();//mSecCounter++;//+10 ms

		if (!(get_mSecCounter() % _1s)) {// 1 seconda
			inc_secCounter();//SecCounter++;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);//set ON/OFF
#ifndef BOOT_LOADER
			putEvt(evt_sec);
#endif
		}
	}
}
//-----------------------------------------------------------------------

#ifdef SET_SWV
/*
#include "stdio.h"

#define ITM_Port8(n)  (*((volatile unsigned char *)(0xE0000000 + 4*n)))
#define ITM_Port16(n) (*((volatile unsigned short *)(0xE0000000 + 4*n)))
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000 + 4*n)))

#define DEMCR (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA 0x1000000

struct __FILE { int handle; }
FILE __stdout;
FILE __stin;


int fputc(int ch, FILE *f)
{
	if (DEMCR & TRCENA) {
		while (ITM_Port32(0) == 0);
		ITM_Port8(0) = ch;
	}
	return ch;
}
*/
int __io_putchar(int ch)
{
	ITM_SendChar(ch);

	return ch;
}
/*
int _write(int file, char *buf, int len)
{
	for (int i = 0; i < len; i++) ITM_SendChar((*buf++));

	return len;
}
*/
#endif


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

