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

HAL_StatusTypeDef send_CAN(uint32_t addres, uint8_t *TxBuffer){		// send 8 Bytes
	static uint32_t TxMailbox[20];
	static CAN_TxHeaderTypeDef TxHeader;
	TxHeader.ExtId = addres;   			// ID of the message
	TxHeader.DLC = 8;         			// Data Length Code (number of bytes in data field)
	TxHeader.IDE = CAN_ID_EXT; 			// Extended ID type
	TxHeader.RTR = CAN_RTR_DATA; 		// Data frame, not remote frame
	TxHeader.TransmitGlobalTime = DISABLE; // Disable time stamp
    // Transmit CAN message
	//HAL_CAN_WakeUp(&hcan1);
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxBuffer, TxMailbox);
    return status;
}

HAL_StatusTypeDef send_CAN_IVT_nbytes(uint32_t addres, uint8_t *TxBuffer, uint8_t length){
	static uint32_t TxMailbox[20];
	static CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = addres;   			// ID of the message
	TxHeader.DLC = length;         		// Data Length Code (number of bytes in data field)
	TxHeader.IDE = CAN_ID_STD; 			// Extended ID type
	TxHeader.RTR = CAN_RTR_DATA; 		// Data frame, not remote frame
	TxHeader.TransmitGlobalTime = DISABLE; // Disable time stamp
    // Transmit CAN message
	//HAL_CAN_WakeUp(&hcan1);
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxBuffer, TxMailbox);
    return status;
}

uint32_t read_CAN(uint8_t *RxData){
	CAN_RxHeaderTypeDef RxHeader;
	// Check if a message is received in CAN RX FIFO 0
	if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
	  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		// Process the received message
	    return RxHeader.ExtId;
	  }
	}
	// Check if a message is received in CAN RX FIFO 1
	if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1) > 0) {
	  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK) {
		// Process the received message
	    return RxHeader.StdId;
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

HAL_StatusTypeDef send_data2ECU(uint16_t GPIO_Input){		// 8 Bytes for TxData, LSB first
	uint8_t can_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};		// LSB first
	can_data[0] |= get_battery_status_code(GPIO_Input);
	can_data[1] |= get_battery_error_code();
	uint16_t total_volt = battery_values.totalVoltage;
	can_data[2] = total_volt&0xFF;
	can_data[3] = total_volt>>8;
	uint16_t actualCurrent = battery_values.actualCurrent;
	can_data[4] = (uint8_t)(actualCurrent/1000);
	can_data[5] = volt2celsius(battery_values.highestCellTemp);
	can_data[6] = 100;		// SoC TBD

	return send_CAN(ADDR_ECU_TX, can_data);
}

HAL_StatusTypeDef ISA_IVT_Init(){
	HAL_StatusTypeDef status = HAL_OK;

	// set sensor mode to STOP
	uint8_t can_data0[] = {SET_MODE, 0x00, 0x01, 0x00, 0x00};
	status |= send_CAN_IVT_nbytes(IVT_MSG_COMMAND, can_data0, 5);
	HAL_Delay(5);

	// set current measurement to CYCLIC 100 Hz
	uint8_t can_data1[] = {(MUX_SETCONFIG|IVT_NCURRENT), CYCLIC, (CYCLETIME>>8)&0xFF, CYCLETIME&0xFF};
	status |= send_CAN_IVT_nbytes(IVT_MSG_COMMAND, can_data1, 4);
	HAL_Delay(5);

	// disable Voltage Measurement
	uint8_t can_data2[] = {(MUX_SETCONFIG|IVT_NU1), DISABLED, 0x00, 0x00};
	status |= send_CAN_IVT_nbytes(IVT_MSG_COMMAND, can_data2, 4);
	HAL_Delay(5);
	uint8_t can_data3[] = {(MUX_SETCONFIG|IVT_NU2), DISABLED, 0x00, 0x00};
	status |= send_CAN_IVT_nbytes(IVT_MSG_COMMAND, can_data3, 4);
	HAL_Delay(5);
	uint8_t can_data4[] = {(MUX_SETCONFIG|IVT_NU3), DISABLED, 0x00, 0x00};
	status |= send_CAN_IVT_nbytes(IVT_MSG_COMMAND, can_data4, 4);
	HAL_Delay(5);

	// set current counter
	uint8_t can_data5[] = {(MUX_SETCONFIG|IVT_NQ), CYCLIC, (CYCLETIME>>8)&0xFF, CYCLETIME&0xFF};
	status |= send_CAN_IVT_nbytes(IVT_MSG_COMMAND, can_data5, 4);
	HAL_Delay(5);

	// store configuration
	//uint8_t can_data6[] = {0x32};
	//status |= send_CAN_IVT_nbytes(IVT_MSG_COMMAND, can_data6, 1);
	//HAL_Delay(1000);

	// set sensor mode to RUN
	HAL_Delay(100);
	uint8_t can_datan[] = {SET_MODE, 0x01, 0x01, 0x00, 0x00};
	status |= send_CAN_IVT_nbytes(IVT_MSG_COMMAND, can_datan, 5);

	return status;
}

void CAN_receive_packet(){
	uint8_t RxData[8];
	uint32_t addres = 0;
	addres = read_CAN(RxData);
	if(addres == ADDR_ECU_TX){
		set_relays(RxData[0]);
		if(RxData[0] & BATTERY_SW_RESET){
			SDC_reset();
		}
	}else if(addres == IVT_MSG_RESPONSE){
		return;
	}else if(addres == IVT_MSG_RESULT_I){
		if(RxData[0] == IVT_NCURRENT){
			battery_values.actualCurrent = RxData[5] | RxData[4]<<8 | RxData[3]<<16 | RxData[2]<<24;
		}
	}else if(addres == IVT_MSG_RESULT_T){
		return;
	}else if(addres == IVT_MSG_RESULT_AS){
		if(RxData[0] == IVT_NQ){
			battery_values.CurrentCounter = RxData[5] | RxData[4]<<8 | RxData[3]<<16 | RxData[2]<<24;
		}
	}
}
