/*
 * CAN_Bus.h
 *
 *  Created on: 12.04.2024
 *      Author: jansc
 */

#ifndef INC_CAN_BUS_H_
#define INC_CAN_BUS_H_

#include "main.h"

#define addr1 0x234
#define addr2 0x12345

extern CAN_HandleTypeDef hcan1;

uint64_t CAN_convert(uint8_t *data);
HAL_StatusTypeDef send_CAN(uint8_t *TxData);
uint16_t read_CAN(uint8_t *RxData);

#endif /* INC_CAN_BUS_H_ */
