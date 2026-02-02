/*
 * communication_serial_protocol.h
 *
 *  Created on: Aug 27, 2025
 *      Author: Matheus Markies
 */

#ifndef INC_COMMUNICATION_SERIAL_PROTOCOL_H_
#define INC_COMMUNICATION_SERIAL_PROTOCOL_H_

#include "stm32h7xx_hal.h"
#include "main.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef enum {
	//STARTING
	FREE = 0,
	WAITING = 1,
	RX = 2,
	PROCESSING = 3,
} Protocol_Status_t;

extern uint16_t send_tick;

extern uint8_t PROTOCOL_RX_Buffer[500];
extern uint8_t PROTOCOL_RX_Stream_Data;
extern int PROTOCOL_Stream_Index;

extern uint16_t last_rx_tick;

extern Protocol_Status_t protocol_status;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef *huart_instance;

extern const uint32_t PROTOCOL_TIMEOUT;
extern const uint32_t SEND_ATTEMPT_INTERVAL_MS;
extern const uint32_t READ_SERIAL_INTERVAL_MS;
extern const uint32_t SEND_AT_INTERVAL_MS;

extern volatile uint32_t last_read_serial_timestamp;
extern volatile uint32_t last_send_at_timestamp;

extern volatile uint8_t is_connected;

typedef enum {
	CMD_STATUS_WAITING = 0,
	CMD_STATUS_PROCESSING = 1,
	CMD_STATUS_PROCESSED = 2,
} Command_State_t;

typedef enum {
    CMD_RESULT_PENDING,
    CMD_RESULT_SUCCESS,
    CMD_RESULT_TIMEOUT,
    CMD_RESULT_ERROR
} Command_Result_t;

typedef void (*CommandCallback_t)(Command_Result_t result, char* response);

typedef struct {
    char* cmd;
    char* expected_answer;
    uint32_t timeout;
    uint32_t start_tick;
    uint32_t send_attempt_tick;
    CommandCallback_t callback;
    Command_Result_t result;
    uint8_t active;
} Command_Context_t;

// ============================================================================
// PROTÓTIPOS DE FUNÇÕES
// ============================================================================

void PROTOCOL_RX_Callback(void);
void PROTOCOL_TX_Callback(void);
void SERIAL_ResetBuffers(void);
void SERIAL_SendCommand(char command[], char answer[], uint32_t timeout, CommandCallback_t callback);
void SERIAL_CheckRXCommand(void);
void SERIAL_CheckConnection(Command_Result_t result, char* response);
void SERIAL_DirectTransmit(char *cmd);

#endif /* INC_COMMUNICATION_SERIAL_PROTOCOL_H_ */
