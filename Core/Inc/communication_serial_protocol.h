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

extern uint16_t send_tick;

extern uint8_t PROTOCOL_RX_Buffer[500];
extern uint8_t PROTOCOL_RX_Stream_Data;
extern int PROTOCOL_Stream_Index;

extern uint16_t last_rx_tick;

extern PROTOCOLStatus protocol_status;

extern UART_HandleTypeDef huart_instance;

extern const uint32_t PROTOCOL_TIMEOUT;
extern const uint32_t SEND_ATTEMPT_INTERVAL_MS;
extern const uint32_t READ_SERIAL_INTERVAL_MS;
extern const uint32_t SEND_AT_INTERVAL_MS;

extern volatile uint32_t last_read_serial_timestamp;
extern volatile uint32_t last_send_at_timestamp;

extern volatile uint8_t is_connected;

// ============================================================================
// PROTÓTIPOS DE FUNÇÕES
// ============================================================================

void PROTOCOL_RX_Callback(void);
void PROTOCOL_TX_Callback(void);
void SERIAL_ResetBuffers(void);
uint8_t SERIAL_SendCommand(char command[], char answer[], uint32_t timeout);
uint8_t SERIAL_CheckConnection(void);
void SERIAL_DirectTransmit(char *cmd);

#endif /* INC_COMMUNICATION_SERIAL_PROTOCOL_H_ */
