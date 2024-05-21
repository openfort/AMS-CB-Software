/*
 * Status_Error_flags.h
 *
 *  Created on: May 20, 2024
 *      Author: jansc
  ******************************************************************************
  * @file           : CB_status_error_flags.h
  * @brief          : Header for status and error flags from the CB to ECU and
  *                   serial monitor.
 */

#ifndef INC_CB_STATUS_ERROR_FLAGS_H_
#define INC_CB_STATUS_ERROR_FLAGS_H_

// Flags from CB to ECU and serial monitor


// CAN Data frame LSB to MSB: status flag byte, error flag byte, total voltage low, total voltage high, actual current low, actual current high, cell temp low, cell temp high

//>> Status Flags 1 Byte
#define STATUS_BATTERY_OK	(1<<0)		// set to 1 when accumulator system is ok, otherwise 0
#define STATUS_CHARGING		(1<<1)		// set to 1 when system is in charging mode, otherwise 0
#define STATUS_MB_TEMP_OK	(1<<2)		// set to 1 when MB temperature is ok, set to 0 if one or more MB has overtemperature due to balancing
#define STATUS_AIR_POSITIVE (1<<3)		// set to 1 when AIR positive is closed
#define STATUS_AIR_NEGATIVE (1<<4)		// set to 1 when AIR negative is closed
#define STATUS_PRECHARGE 	(1<<5)		// set to 1 when AIR precharge is closed
#define STATUS_RESERVE2		(1<<6)		// unused, set to 0
#define STATUS_RESERVE1		(1<<7)		// unused, set to 0

//>> Error Flags 1 Byte
#define ERROR_BATTERY		(1<<0)		// set to 1 if any error with the battery cells occur
#define ERROR_TEMP			(1<<1)		// set to 1 if any cell has overtemperature
#define ERROR_VOLT			(1<<2)		// set to 1 if any cell has over-/undervoltage
#define ERROR_SDC			(1<<3)		// set to 1 if SDC is in error state
#define ERROR_SPI			(1<<4)		// set to 1 if an error related to IsoSPI occurs
#define ERROR_CAN			(1<<5)		// set to 1 if CAN buffer overflow occurs
#define ERROR_RESERVE2		(1<<6)		// unused, set to 0
#define ERROR_RESERVE1		(1<<7)		// unused, set to 0

#endif /* INC_CB_STATUS_ERROR_FLAGS_H_ */
