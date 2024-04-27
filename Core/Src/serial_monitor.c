/*
 * serial_monitor.c
 *
 *  Created on: 12.04.2024
 *      Author: schweja3
 */

#include "serial_monitor.h"

void switch_HByte_LByte(uint8_t *buffer, uint16_t size){
	uint8_t low_byte;
	for(uint16_t i=0; i<size>>1; i++){	// switch high and low byte per 16 bit value
		low_byte = buffer[i*2];
		buffer[i*2] = buffer[i*2+1];
		buffer[i*2+1] = low_byte;
	}
}

void SerialMonitor(Serial_Commmand_type command, uint8_t* data, uint16_t size){
	if((command&0xF0)==0xA0){
		uint8_t send_buffer[size];
		for(uint16_t i=0; i<size>>1; i++){	// switch high and low byte per 16 bit value
			send_buffer[i*2] = data[i*2+1];
			send_buffer[i*2+1] = data[i*2];
		}
		uint8_t start[] = {0xFF, command};
		uint8_t stop[] = {0xFF, command|0xB0};
		HAL_UART_Transmit(&huart2, start, 2, 100);
		HAL_UART_Transmit(&huart2, send_buffer, size, 100);
		HAL_UART_Transmit(&huart2, stop, 2, 100);
	}else{
		uint8_t code[] = {0xFF, command};
		HAL_UART_Transmit(&huart2, code, 2, 100);
	}
}
