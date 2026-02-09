/*
 * communication_serial_protocol.h
 *
 *  Created on: Aug 27, 2025
 *      Author: Matheus Markies
 */

#ifndef INC_COMMUNICATION_SERIAL_PROTOCOL_H_
#define INC_COMMUNICATION_SERIAL_PROTOCOL_H_

#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CMD_QUEUE_SIZE 12
#define MAX_CMD_LENGTH 1536
#define MAX_ANSWER_LENGTH 64
#define PROTOCOL_RX_BUFFER_SIZE 3072

//#define DEBUG_SERIAL_PROTOCOL

extern const uint32_t PROTOCOL_TIMEOUT;
extern const uint32_t SEND_ATTEMPT_INTERVAL_MS;
extern const uint32_t READ_SERIAL_INTERVAL_MS;
extern const uint32_t SEND_AT_INTERVAL_MS;

typedef enum {
    FREE = 0,
    WAITING = 1,
    BUSY = 2
} Protocol_Status_t;

typedef enum {
    CMD_RESULT_PENDING = 0,
    CMD_RESULT_SUCCESS = 1,
    CMD_RESULT_TIMEOUT = 2,
    CMD_RESULT_ERROR = 3
} Command_Result_t;

typedef enum {
    CMD_STATUS_WAITING = 0,
    CMD_STATUS_PROCESSING = 1
} Command_State_t;

typedef void (*CommandCallback_t)(Command_Result_t result, char *response);

typedef struct {
    char cmd[MAX_CMD_LENGTH];
    char expected_answer[MAX_ANSWER_LENGTH];
    uint32_t timeout;
    CommandCallback_t callback;
    uint8_t active;
} Command_Item_t;

typedef struct {
    Command_Item_t buffer[CMD_QUEUE_SIZE];
    volatile uint8_t head;      // Próxima posição para escrever
    volatile uint8_t tail;      // Próxima posição para ler
    volatile uint8_t count;     // Número de itens na fila
} Command_Queue_t;

typedef struct {
    char cmd[MAX_CMD_LENGTH];
    char expected_answer[MAX_ANSWER_LENGTH];
    uint32_t timeout;
    uint32_t start_tick;
    uint32_t send_attempt_tick;
    CommandCallback_t callback;
    Command_Result_t result;
    uint8_t active;
} Command_Context_t;

extern volatile uint8_t is_connected;
extern Protocol_Status_t protocol_status;
extern UART_HandleTypeDef *huart_instance;
extern Command_Context_t current_command;
extern Command_Queue_t command_queue;

extern uint8_t PROTOCOL_RX_Buffer[PROTOCOL_RX_BUFFER_SIZE];
extern uint8_t PROTOCOL_RX_Stream_Data;
extern volatile int PROTOCOL_Stream_Index;
extern volatile uint16_t last_rx_tick;

void SERIAL_Init(UART_HandleTypeDef *huart);

uint8_t SERIAL_QueueCommand(const char *command, const char *answer,
                             uint32_t timeout, CommandCallback_t callback);
uint8_t SERIAL_IsQueueFull(void);
uint8_t SERIAL_GetQueueCount(void);
void SERIAL_ClearQueue(void);

void SERIAL_SendCommand(char command[], char answer[], uint32_t timeout,
                        CommandCallback_t callback);
void SERIAL_SendJSON(char *json_string, char answer[], uint32_t timeout,
                     CommandCallback_t callback);
void SERIAL_DirectTransmit(char *cmd);

void SERIAL_ProcessQueue(void);
void SERIAL_CheckRXCommand(void);
void SERIAL_CheckConnection(Command_Result_t result, char *answer);

void SERIAL_ResetBuffers(void);

void PROTOCOL_RX_Callback(void);
void PROTOCOL_TX_Callback(void);

#endif /* INC_COMMUNICATION_SERIAL_PROTOCOL_H_ */
