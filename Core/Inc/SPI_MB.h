/*
 * SPI_MB.h
 *
 *  Created on: Mar 20, 2024
 *      Author: schweja3
 */

#ifndef SRC_SPI_MB_H_
#define SRC_SPI_MB_H_

#include "main.h"

// ADBMS Register Address
#define WRCFGA   0x0001
#define WRCFGB   0x0024
#define RDCFGA   0x0002
#define RDCFGB   0x0026

#define RDCVA    0x0004
#define RDCVB    0x0006
#define RDCVC    0x0008
#define RDCVD    0x000A
#define RDCVE    0x0009
#define RDCVF    0x000B

#define RDAUXA   0x000C
#define RDAUXB   0x000E
#define RDAUXC   0x000D
#define RDAUXD   0x000F

#define RDSTATA  0x0010
#define RDSTATB  0x0012

#define WRSCTRL  0x0014
#define WRPWM    0x0020
#define WRPSB    0x001C
#define RDSCTRL  0x0016
#define RDPWM    0x0022
#define RDPSB    0x001E
#define STSCTRL  0x0019
#define CLRSCTRL 0x0018

#define CLRCELL  0x0711
#define CLRAUX   0x0712
#define CLRSTAT  0x0713
#define PLADC    0x0714
#define DIAGN    0x0715
#define WRCOMM   0x0721
#define RDCOMM   0x0722
#define STCOMM   0x0723
#define MUTE     0x0028
#define UNMUTE   0x0029

#define ADCV	 0x0360		// 3kHz  Cell, all Cells
#define ADAX	 0x04E0		// 14kHz GPIO
#define ADSTAT	 0x04EA		// 14kHz internal Temp

// Settings
#define NUM_OF_CLIENTS 8
#define DUMMY 0xAA

// HAL Handle
extern SPI_HandleTypeDef hspi1;

// SPI MB Functions
HAL_StatusTypeDef Read_Voltages(uint16_t *data_buffer);
HAL_StatusTypeDef Read_Temp(uint16_t *data_buffer);
uint16_t read_ADBMS_Temp();
HAL_StatusTypeDef set_DCCx(uint32_t* cells_to_balance);
HAL_StatusTypeDef ADBMS_HW_Init();

#endif /* SRC_SPI_MB_H_ */
