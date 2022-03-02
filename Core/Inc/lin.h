/*

The MIT License (MIT)

Copyright (c) 2022 R. Edwards

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

/**
******************************************************************************
* @file           : lin.h
* @brief          : LIN driver to interface with the STM32G4
******************************************************************************
*/
#ifndef __LIN_H
#define __LIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Exported defines -----------------------------------------------------------*/
#define LIN_CHANNEL_1 0U
#define LIN_MAX_DATA_BYTES 8U
#define LIN_CONFIG_MSG_ID_CMD       0x1FFFFE80
#define LIN_CONFIG_MSG_ID_DATA      0x1FFFFE81
#define LIN_GATEWAY_MSG_ID          0x1FFFFE82
#define IS_LIN_FRAME(can_id)    (can_id == LIN_CONFIG_MSG_ID_CMD \
                                  || can_id == LIN_CONFIG_MSG_ID_DATA)

/* Exported types ------------------------------------------------------------*/
typedef enum {
  LIN_SLAVE,
  LIN_MONITOR,
  LIN_MASTER
} lin_type_t;

typedef enum {
  LIN_IDLE_AWAIT_BREAK,
  LIN_IDLE_AWAIT_SYNC,
  LIN_PID_RX,
  LIN_MASTER_TX_DATA,
  LIN_MASTER_RX_DATA,
  LIN_SLAVE_RX_DATA,
  LIN_SLAVE_TX_DATA,
  LIN_MONITOR_RX_DATA
} lin_state_t;

typedef enum {
  LIN_CMD_LOAD_MSG,
  LIN_CMD_READ_MSG,
  LIN_CMD_ENABLE_MSG,
  LIN_CMD_DISABLE_MSG,
  LIN_CMD_DISABLE_ALL_MSGS,
  LIN_CMD_ERASE_ALL_MSGS,
  LIN_CMD_ENABLE_LIN,
  LIN_CMD_DISABLE_LIN
} lin_command_type_t;

typedef union
{
  struct __attribute__((packed)) 
  {
    uint8_t PID;
    uint8_t len;
    uint8_t data[8];
  } lin_msg_data;
  uint32_t lin_baud_rate;
  uint8_t checksum_type;  
} lin_data_t;

typedef struct
{
  uint8_t lin_rx_data_available    : 1;
} lin_flags_t;

typedef struct
{
  uint8_t pid;
  uint8_t data_length;
  uint8_t lin_data_buffer[9];
  uint8_t checksum;
} lin_data_frame_t;

typedef struct __attribute__((packed))
{
  uint8_t is_active : 1;
  uint8_t lin_node_action : 2;
} lin_msg_flags_t;

typedef struct __attribute__((packed))
{
  lin_msg_flags_t lin_msg_flags;
  uint8_t PID;
  uint8_t len;
  uint8_t data[LIN_MAX_DATA_BYTES];
} lin_msg_data_t;

typedef struct {

  uint8_t lin_cmd_type;
  uint8_t lin_channel;
  uint8_t lin_table_index;
  lin_msg_data_t lin_data;
} lin_config_t;


typedef struct {
  uint8_t lin_instance;
  uint8_t lin_msg_table_index;
  UART_HandleTypeDef* huart;
  lin_flags_t lin_flags;
  uint8_t UartRxBuffer[1];
  uint8_t lin_state;
  lin_type_t lin_type;
  lin_data_frame_t lin_data_frame;
  uint32_t lin_rx_timeout;
  uint8_t lin_classicChecksum;
  uint32_t lin_baud_rate; 
} LIN_HandleTypeDef;


/* Exported functions --------------------------------------------------------*/
void lin_init(LIN_HandleTypeDef* hlin);
void lin_handler_task(LIN_HandleTypeDef* hlin);
void lin_handle_uart_rx_IRQ(LIN_HandleTypeDef* hlin);
uint8_t lin_config(uint32_t msg_id, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __LIN_H */
