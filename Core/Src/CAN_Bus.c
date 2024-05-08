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

HAL_StatusTypeDef send_CAN(uint32_t addres, uint8_t *TxBuffer){
	static uint32_t TxMailbox[20];
	static CAN_TxHeaderTypeDef TxHeader;
	TxHeader.ExtId = addres;   			// ID of the message
	TxHeader.DLC = 8;         			// Data Length Code (number of bytes in data field)
	TxHeader.IDE = CAN_ID_EXT; 			// Extended ID type
	TxHeader.RTR = CAN_RTR_DATA; 		// Data frame, not remote frame
	TxHeader.TransmitGlobalTime = DISABLE; // Disable time stamp
    // Transmit CAN message
	//HAL_CAN_WakeUp(&hcan1);
	//HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, (uint32_t *)CAN_TX_MAILBOX0);
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxBuffer, TxMailbox);
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

uint8_t FIFO_ovf(){
	// check if FIFO is full
	if ((HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 3) || (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1) >= 3)) {
		return 1;
	}else{
		return 0;
	}
}

HAL_StatusTypeDef send_data2ECU(uint32_t GPIO_Input, uint8_t error_codes){		// 8*Bytes for TxData, LSB first
	uint8_t can_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};		// LSB first
	can_data[0] |= (GPIO_Input&V_FB_AIR_positive_Pin) >> (3-0);
	can_data[0] |= (GPIO_Input&V_FB_AIR_negative_Pin) >> (1-1);
	can_data[0] |= (GPIO_Input&V_FB_PC_Relay_Pin)	  >> (4-2);
	can_data[0] |= (GPIO_Input&Charger_Con_Pin)       >> (10-4);
	can_data[1] |= error_codes;
	uint16_t total_volt = battery_values.totalVoltage;
	can_data[2] = total_volt&0xFF;
	can_data[3] = total_volt>>8;
	uint16_t max_temp = battery_values.highestCellTemp;
	can_data[4] = max_temp&0xFF;
	can_data[5] = max_temp>>8;

	return send_CAN(ext_addr_ECU, can_data);
}
