/*
 * serial_monitor.c
 *
 *  Created on: 12.04.2024
 *      Author: schweja3
 */

#include "serial_monitor.h"

void SerialMonitor(uint8_t* data, uint16_t size){
	static uint8_t DMA_buffer[520];
	static uint8_t start[] = {0xFF, 0xA3};
	static uint8_t stop[] = {0xFF, 0xB3};
	DMA_buffer[0] = start[0];
	DMA_buffer[1] = start[1];
	for(uint16_t i=0; i<size; i++){
		DMA_buffer[i+2] = data[i];
	}
	DMA_buffer[size+2] = stop[0];
	DMA_buffer[size+3] = stop[1];

	//HAL_UART_Transmit(&huart2, DMA_buffer, size+4, 100);

	HAL_UART_Transmit_DMA(&huart2, DMA_buffer, size+4);
}
