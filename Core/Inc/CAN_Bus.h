/*
 * CAN_Bus.h
 *
 *  Created on: 12.04.2024
 *      Author: jansc
 */

#ifndef INC_CAN_BUS_H_
#define INC_CAN_BUS_H_

#include "main.h"
#include "battery.h"

#define local_addr_ECU 0x410		// addres used ECU => CB
#define ext_addr_ECU   0x310		// addres used CB => ECU
#define local_addr_IVS 0x12345		// addres used from ISA IVS-T TBD

// CAN send/receive Flags in 8 bit format, can byte 0 LSB
#define AIR_positive 		(1<<0)
#define AIR_negative 		(1<<1)
#define Precharge_Relay 	(1<<2)
#define Battery_SW_reset	(1<<3)	// receive only
#define Battery_charging	(1<<4)	// send only

// can error codes in 8 bit format, can byte 1
#define SPI_error 			(1<<0)
#define Voltage_error		(1<<1)
#define Temperature_error	(1<<2)
#define Battery_error		(1<<3)
#define CAN_buffer_ovf		(1<<4)

// 16 bit voltage in can byte 2/3
// 16 bit temperature in can byte 4/5
// 16 bit reserve in can byte 6/7 MSB

extern CAN_HandleTypeDef hcan1;

uint64_t CAN_convert(uint8_t *data);
HAL_StatusTypeDef send_data2ECU(uint32_t GPIO_Input, uint8_t error_codes);
uint16_t read_CAN(uint8_t *RxData);
uint8_t FIFO_ovf();

#endif /* INC_CAN_BUS_H_ */
