/*
 * serial_monitor.h
 *
 *  Created on: 12.04.2024
 *      Author: jansc
 */

#ifndef INC_SERIAL_MONITOR_H_
#define INC_SERIAL_MONITOR_H_

#include "main.h"

// HAL Handle
extern UART_HandleTypeDef huart2;

// Function prototypes
void SerialMonitor(uint8_t* data, uint16_t size);

#endif /* INC_SERIAL_MONITOR_H_ */
