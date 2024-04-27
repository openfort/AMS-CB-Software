/*
 * CAN_Bus.c
 *
 *  Created on: 12.04.2024
 *      Author: jansc
 */

#include "CAN_Bus.h"

uint64_t CAN_convert(uint8_t *data){
	uint64_t data64 = 0;
	for(uint8_t i=0; i<8; i++){
		data64 |= data[i]<<(i*8);
	}
	return data64;
}

HAL_StatusTypeDef send_CAN(uint8_t *TxData){		// 8*Bytes for TxData, LSB first
	static uint32_t TxMailbox[20];
	static CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = 0x123;   // Standard ID of the message
	TxHeader.DLC = 8;         // Data Length Code (number of bytes in data field)
	TxHeader.IDE = CAN_ID_STD; // Standard ID type
	TxHeader.RTR = CAN_RTR_DATA; // Data frame, not remote frame
	TxHeader.TransmitGlobalTime = DISABLE; // Disable time stamp
    // Transmit CAN message
	//HAL_CAN_WakeUp(&hcan1);
	//HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, (uint32_t *)CAN_TX_MAILBOX0);
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, TxMailbox);
    return status;
}

uint16_t read_CAN(uint8_t *RxData){			// LSB first
	CAN_RxHeaderTypeDef RxHeader;
	// Check if a message is received in CAN RX FIFO 0
	if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
	  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		// Process the received message
	    return (uint16_t)(RxHeader.ExtId);
	  }
	}
	// Check if a message is received in CAN RX FIFO 1
	if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1) > 0) {
	  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		// Process the received message
	    return (uint16_t)(RxHeader.ExtId);
	  }
	}
	return 0;
}
