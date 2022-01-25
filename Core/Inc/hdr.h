/*
 * hdr.h
 *
 *  Created on: Dec 13, 2021
 *      Author: alarm
 */

#ifndef __HDR_H_
#define __HDR_H_


#define SET_SPI_DISPLAY

#define SET_MQ135
//#define SET_BME280
#ifndef SET_BME280
	#define SET_SI7021
	#define SET_SI7021_DUMP
#endif

#define SET_BLE
#ifdef SET_BLE
	#define SET_QUEUE
#endif

//#define SET_SWV


#endif /* __HDR_H_ */
