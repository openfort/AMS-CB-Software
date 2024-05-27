/*
 * serial_monitor.c
 *
 *  Created on: 12.04.2024
 *      Author: schweja3
 */

#include "serial_monitor.h"

void SerialMonitor(uint8_t* data, uint16_t size){
	uint8_t start[] = {0xFF, 0xA3};
	uint8_t stop[] = {0xFF, 0xB3};
	HAL_UART_Transmit(&huart2, start, 2, 100);
	HAL_UART_Transmit(&huart2, data, size, 100);
	HAL_UART_Transmit(&huart2, stop, 2, 100);
}
