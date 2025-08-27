/*
 * communication_serial_protocol.h
 *
 *  Created on: Aug 27, 2025
 *      Author: Matheus Markies
 */

#ifndef INC_COMMUNICATION_SERIAL_PROTOCOL_H_
#define INC_COMMUNICATION_SERIAL_PROTOCOL_H_

#include "main.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>

typedef enum {
	//STARTING
	FREE = 0,
	WAITING = 1,
	RX = 2,
	PROCESSING = 3,
} PROTOCOLStatus;

uint16_t send_tick = 0;

uint8_t PROTOCOL_RX_Buffer[500];
uint8_t PROTOCOL_RX_Stream_Data = 0;
int PROTOCOL_Stream_Index = 0;

uint16_t last_rx_tick = 0;

PROTOCOLStatus protocol_status = FREE;

UART_HandleTypeDef huart_instance;

const uint32_t PROTOCOL_TIMEOUT = 1 * 1000;
const uint32_t SEND_ATTEMPT_INTERVAL_MS = 500;
const uint32_t READ_SERIAL_INTERVAL_MS = 20;
const uint32_t SEND_AT_INTERVAL_MS = 50;

volatile uint32_t last_read_serial_timestamp = 0;
volatile uint32_t last_send_at_timestamp = 0;

void PROTOCOL_RX_Callback(void);
void PROTOCOL_TX_Callback(void);
void resetBuffers(void);
uint8_t sendCommand(char command[], char answer[], uint32_t timeout);
void directTransmit(char *cmd);

#endif /* INC_COMMUNICATION_SERIAL_PROTOCOL_H_ */
