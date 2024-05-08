/*
 * SPI_MB.c
 *
 *  Created on: Mar 20, 2024
 *      Author: schweja3
 */

#include "SPI_MB.h"

// Default Configuration Register
static const uint8_t CFGAR[] = {0xF9, 0x00, 0xF0, 0xFF, 0x00, 0x00}; // data for CRFA, ADCOPT = 1, REFON = 1, GPIOx = 1
static const uint8_t CFGBR[] = {0x0F, 0x00, 0x00, 0x00, 0x00, 0x00}; // data for CRFB
static const uint8_t RDCV[] = {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF};	// Read Voltages Register
static const uint8_t RDAUX[] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD};

void delay_1us(){	// delay 960ns + pin delay 45ns = 1050ns
	for(volatile uint32_t i=0; i<5; i++);	// 100ns per cycle
}

void wake_up(){
	for(uint8_t i=0; i<num_of_clients+2; i++){
		GPIOB->BSRR = ISO_SPI_CS1_Pin<<16;	// CS low
		delay_1us();
		GPIOB->BSRR = ISO_SPI_CS1_Pin;	// CS high
		delay_1us();
	}
	for(volatile uint32_t i=0; i<100; i++);	// 100ns per cycle, 10us, communication ready time
}

HAL_StatusTypeDef SPI_Transceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
	GPIOB->BSRR = ISO_SPI_CS1_Pin<<16;	// CS low
	delay_1us();
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size, 200);
	GPIOB->BSRR = ISO_SPI_CS1_Pin;	// CS high
	return status;
}

/*
uint16_t Calculate_CRC(uint8_t* data, uint16_t size) {
	uint16_t dat[] = {0x0001};
    return (uint16_t)(HAL_CRC_Calculate(&hcrc, (uint32_t *)dat, 1)) & 0xFE;
}
*/
// Function to generate 15-bit packet error code
uint16_t generatePEC(uint8_t data[], size_t length) {
    // Initial value of PEC
    uint16_t pec = 0x0010;
    // Characteristic polynomial: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
    for (size_t i = 0; i < length; ++i) {
            for (int bit = 7; bit >= 0; --bit) {
                uint16_t in0 = ((data[i] >> bit) & 0x01) ^ ((pec >> 14) & 0x01);
                uint16_t in3 = in0 ^ ((pec >> 2) & 0x01);
                uint16_t in4 = in0 ^ ((pec >> 3) & 0x01);
                uint16_t in7 = in0 ^ ((pec >> 6) & 0x01);
                uint16_t in8 = in0 ^ ((pec >> 7) & 0x01);
                uint16_t in10 = in0 ^ ((pec >> 9) & 0x01);
                uint16_t in14 = in0 ^ ((pec >> 13) & 0x01);
                pec <<= 1;
                pec = (pec & 0x3FFF) | (in14 << 14);
                pec = (pec & 0xFBFF) | (in10 << 10);
                pec = (pec & 0xFEFF) | (in8 << 8);
                pec = (pec & 0xFF7F) | (in7 << 7);
                pec = (pec & 0xFFEF) | (in4 << 4);
                pec = (pec & 0xFFF7) | (in3 << 3);
                pec = (pec & 0xFFFE) | (in0 << 0);
            }
        }
    pec <<=1;
    return pec;
}


HAL_StatusTypeDef Command(uint16_t command){	// checked
	uint8_t tx_data[4];
	uint8_t crc_data[2];
	crc_data[0] = command>>8;
	crc_data[1] = command&0xFF;
	uint16_t crc = generatePEC(crc_data, 2);
	tx_data[0] = command>>8;
	tx_data[1] = command&0xFF;
	tx_data[2] = crc>>8;
	tx_data[3] = crc&0xFF;
	uint8_t rx_data[4];
	return SPI_Transceive(tx_data, rx_data, 4);
}

HAL_StatusTypeDef Write_Registergroup(uint16_t command, uint8_t *data){		// write data to every single client, data length: 6*num_of_clients
	uint8_t tx_data[4+num_of_clients*8];
	uint8_t crc_data[2];
	crc_data[0] = command>>8;
	crc_data[1] = command&0xFF;
	uint16_t crc = generatePEC(crc_data, 2);
	tx_data[0] = command>>8;
	tx_data[1] = command&0xFF;
	tx_data[2] = crc>>8;
	tx_data[3] = crc&0xFF;
	for(uint16_t i=0; i<num_of_clients; i++){
		for(uint16_t j=0; j<6; j++){
			tx_data[4+i*8+j] = data[i*6+j];
		}
		uint16_t crc = generatePEC(&data[i*6], 6);
		tx_data[4+i*8+6] = crc>>8;
		tx_data[4+i*8+7] = crc&0xFF;
	}
	uint8_t rx_data[sizeof(tx_data)];
	return SPI_Transceive(tx_data, rx_data, sizeof(tx_data));
}

HAL_StatusTypeDef Read_Registergroup(uint16_t command, uint8_t *buffer){		// checked, read data for every single client, data length: 6*num_of_clients
	uint8_t tx_data[4+num_of_clients*8];
	for(uint16_t i=0; i<sizeof(tx_data)-4; i++){
		tx_data[i+4] = dummy;
	}
	uint8_t crc_data[2];
	crc_data[0] = command>>8;
	crc_data[1] = command&0xFF;
	uint16_t crc = generatePEC(crc_data, 2);
	tx_data[0] = command>>8;
	tx_data[1] = command&0xFF;
	tx_data[2] = crc>>8;
	tx_data[3] = crc&0xFF;
	uint8_t rx_data[sizeof(tx_data)];
	HAL_StatusTypeDef status = SPI_Transceive(tx_data, rx_data, sizeof(tx_data));		// read data
	uint16_t not_valid;
	for(uint16_t i=0; i<num_of_clients; i++){
		crc = generatePEC(&rx_data[4+i*8], 6);
		not_valid = (rx_data[10+i*8]<<8 | rx_data[11+i*8])-crc;		// check crc
		for(uint16_t j=0; j<6; j++){			// write to buffer
			buffer[i*6+j] = rx_data[4+i*8+j];
		}
		if(not_valid){
			return HAL_ERROR;
		}
	}
	return status;
}

HAL_StatusTypeDef Read_Voltages(uint8_t *buffer){		// checked, num_of_clients * 36
	HAL_StatusTypeDef status;
	uint8_t sbuffer[num_of_clients*6];		// short buffer for a single transmission
	wake_up();
	Command(ADCV);
	HAL_Delay(3);
	for(uint16_t i=0; i<6; i++){
		//wake_up();		// used for debug
		status = Read_Registergroup(RDCV[i], sbuffer);
		if(status==HAL_OK){
			for(uint16_t j=0; j<num_of_clients; j++){
				for(uint16_t k=0;k<6; k++){
					buffer[j*36+i*6+k] = sbuffer[j*6+k];
				}
			}
		}else{
			for(uint16_t j=0; j<(num_of_clients*36); j++){
				buffer[j] = 0;
			}
			return status;
		}
	}
	return status;
}

HAL_StatusTypeDef Read_Temp(uint8_t *buffer){		// buffer num_of_clients * 20
	HAL_StatusTypeDef status;
	uint8_t sbuffer[num_of_clients*6];		// short buffer for a single transmission
	wake_up();
	Command(ADAX);
	HAL_Delay(3);
	for(uint16_t i=0; i<4; i++){
		//wake_up();		// used for debug
		status = Read_Registergroup(RDAUX[i], sbuffer);
		if(status==HAL_OK){
			if(i<3){		// Register AUXA - AUXC
				for(uint16_t j=0; j<num_of_clients; j++){		// read 6 Bytes
					for(uint16_t k=0;k<6; k++){
						buffer[j*20+i*6+k] = sbuffer[j*6+k];
					}
				}
			}else{			// Register AUXD
				for(uint16_t j=0; j<num_of_clients; j++){		// read 2 Bytes
					for(uint16_t k=0;k<2; k++){
						buffer[j*20+i*6+k] = sbuffer[j*6+k];
					}
				}
			}
		}else{
			for(uint16_t j=0; j<(num_of_clients*20); j++){
				buffer[j] = 0;
			}
			return status;
		}
	}
	// delete reference voltage and gpio9 => values 5 and 9
	uint16_t j=12;
	for(uint16_t i=10; i<16*num_of_clients; i++){
		buffer[i] = buffer[j];
		if((j%20==9) || (j%20==17)){
			j+=3;
		}else{
			j++;
		}
	}
	return status;
}

HAL_StatusTypeDef ADBMS_HW_Init(){
	uint8_t config_data_A[num_of_clients*6];
	uint8_t config_data_B[num_of_clients*6];
	for(uint16_t i=0; i<num_of_clients; i++){
		for(uint16_t j=0; j<6; j++){
			config_data_A[i*6+j] = CFGAR[j];
			config_data_B[i*6+j] = CFGBR[j];
		}
	}
	wake_up();
	HAL_Delay(1);		// timeout for stability
	HAL_StatusTypeDef status = HAL_OK;
	status |= Write_Registergroup(WRCFGA, config_data_A);
	status |= Write_Registergroup(WRCFGB, config_data_B);
	uint8_t read_data_A[num_of_clients*6];
	uint8_t read_data_B[num_of_clients*6];
	status |= Read_Registergroup(RDCFGA, read_data_A);
	status |= Read_Registergroup(RDCFGB, read_data_B);

	if(status != HAL_OK){
		return status;
	}
	uint8_t not_valid = 0;
	for(uint16_t i=0; i<num_of_clients; i++){
		for(uint16_t j=1; j<6; j++){
			not_valid += (config_data_A[i*6+j] - read_data_A[i*6+j]);
			not_valid += (config_data_B[i*6+j] - read_data_B[i*6+j]);
		}
	}
	if(not_valid){
		return HAL_ERROR;
	}else{
		return HAL_OK;
	}
}
