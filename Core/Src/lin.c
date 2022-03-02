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
* @file           : lin.c
* @brief          : LIN driver to interface with the STM32G4
******************************************************************************
*/

#include <string.h>
#include "lin.h"
#include "main.h"
#include "can.h"
#include "gs_usb.h"

#define LIN_SYNC_BYTE 0x55u
#define LIN_GET_PID_BIT(x,y) (((x) >> (y)) & 0x01u)
#define LIN_ID_MASK 0x3Fu
#define LIN_P0_FLAG 6
#define LIN_P1_FLAG 7
#define LIN_RX_TIMEOUT_VALUE 3
#define LIN_RX_MAX_DATA_BYTES 9

#define LIN_MAX_USART_CHAN      1U
#define LIN_MAX_MESSAGE_ITEMS   10U

static lin_msg_data_t lin_message_table[LIN_MAX_USART_CHAN][LIN_MAX_MESSAGE_ITEMS];
static lin_config_t lin_config_data;

extern UART_HandleTypeDef huart4;
extern LIN_HandleTypeDef hlin1;
extern FDCAN_HandleTypeDef hfdcan1;

static FlagStatus lin_hw_check_for_break(LIN_HandleTypeDef* hlin);
static uint8_t lin_data_layer_checksum(uint8_t pid, uint8_t length, const uint8_t* data_ptr);
static void lin_erase_message_table(void);
static void lin_disable_all_messages(void);
static void lin_can_gateway_tx(LIN_HandleTypeDef* hlin);

extern void MX_UART4_Init(void);

void lin_init(LIN_HandleTypeDef* hlin)
{
  /* erase the message table to ensure no random data */
  lin_erase_message_table();  

  MX_UART4_Init();
  HAL_UART_Receive_IT(&huart4, hlin1.UartRxBuffer, 1);
  
  /* Turn on the LIN drivers */
  HAL_GPIO_WritePin(LIN1_NSLP_GPIO_Port, LIN1_NSLP_Pin, GPIO_PIN_SET);

  /* init the LIN handles */
  hlin->huart = &huart4;
  hlin->lin_state = LIN_IDLE_AWAIT_BREAK;
  hlin->lin_instance = LIN_CHANNEL_1;
}

void lin_handle_uart_rx_IRQ(LIN_HandleTypeDef* hlin)
{
  /* create a local copy for readability */
  uint8_t rbyte = hlin->UartRxBuffer[0];

  /* LIN can be reset and state flags set outside of this routine as needed */
  if (hlin->lin_state == LIN_IDLE_AWAIT_BREAK) {
    hlin->lin_data_frame.data_length = 0;
    /* if flag is set to wait for sync and we get sync set flag to get ID */
    if (lin_hw_check_for_break(hlin) == SET  && rbyte == LIN_SYNC_BYTE) {
      hlin->lin_state = LIN_PID_RX;
    }
    else {
      /* do nothing - wait for break */
    }
  }
  else {
    switch (hlin->lin_state) {
      case LIN_PID_RX:
        /* is this PID in the config table - loop to find out */
        /* based on the table this is either a master, slave, or monitor */
        hlin->lin_rx_timeout = HAL_GetTick() + LIN_RX_TIMEOUT_VALUE;
        hlin->lin_data_frame.pid = rbyte;
        for (uint8_t lin_message_index = 0; lin_message_index < LIN_MAX_MESSAGE_ITEMS; lin_message_index++) {
          if ((hlin->lin_data_frame.pid & LIN_ID_MASK) == lin_message_table[hlin->lin_instance][lin_message_index].PID  &&
                (lin_message_table[hlin->lin_instance][lin_message_index].lin_msg_flags.is_active) &&
                hlin->lin_data_frame.pid != 0x00) {
            /* there is an instance in the table for this PID - handle based on the flags */
            hlin->lin_msg_table_index = lin_message_index;
            if (lin_message_table[hlin->lin_instance][lin_message_index].lin_msg_flags.lin_node_action == LIN_MONITOR) {
              hlin->lin_state = LIN_MONITOR_RX_DATA;
            }
            else if (lin_message_table[hlin->lin_instance][lin_message_index].lin_msg_flags.lin_node_action == LIN_SLAVE){
              hlin->lin_state = LIN_SLAVE_TX_DATA;
            }
            else {
              /* there was a match but no matching action so reset */
              hlin->lin_state = LIN_IDLE_AWAIT_BREAK;
            }
            break; /* exit the for loop as we have found a matching item */
          }
        }       
        break;
      case LIN_MONITOR_RX_DATA:
        /* reset the timeout for this reception state */
        /* this is handled in the interrupt since data is read byte by byte */
        hlin->lin_rx_timeout = HAL_GetTick() + LIN_RX_TIMEOUT_VALUE;
        hlin->lin_data_frame.lin_data_buffer[hlin->lin_data_frame.data_length] = rbyte;
        if (hlin->lin_data_frame.data_length > LIN_RX_MAX_DATA_BYTES) {
          /* we've received the max # of bytes, flag it */
          hlin->lin_flags.lin_rx_data_available = 1;
        }
        else {
          hlin->lin_data_frame.data_length++;
        }
        break;

      default:
        break;
    }
  }
}

void lin_handler_task(LIN_HandleTypeDef* hlin)
{
  uint8_t data_index = 0;
  uint8_t checksum;
  uint8_t tx_buffer[9];
  uint8_t data_length;

  switch (hlin->lin_state) {
    case LIN_MONITOR_RX_DATA:
      if (HAL_GetTick() >= hlin->lin_rx_timeout || hlin->lin_flags.lin_rx_data_available) {
        /* either the max number of bytes were received or the state machine timed out waiting for more data */
        hlin->lin_data_frame.data_length--; /* subtract the checksum from the data length */
        hlin->lin_data_frame.checksum = hlin->lin_data_frame.lin_data_buffer[hlin->lin_data_frame.data_length];
        if (hlin->lin_data_frame.checksum == lin_data_layer_checksum(hlin->lin_data_frame.pid,
          hlin->lin_data_frame.data_length, /* add the PID to the data length */
          hlin->lin_data_frame.lin_data_buffer)) {
          /* We have good data, take action on it */
          /* send this data on CAN */
          lin_can_gateway_tx(hlin);
        }
        else {
          /* checksum doesn't match - TODO: throw an error eventually */
        }

        /* Reset the LIN state machine */
        hlin->lin_flags.lin_rx_data_available = 0;
        hlin->lin_data_frame.data_length = 0;
        hlin->lin_state = LIN_IDLE_AWAIT_BREAK;
      }
      break;
    case LIN_SLAVE_TX_DATA:
      /* we have RX'd a PID that we now need to respond to */
      /* need the table index that we used to trigger the PID */
      /* create a copy of the data length for readability */
      data_length = lin_message_table[hlin->lin_instance][hlin->lin_msg_table_index].len;

      /* grab the data from the table and load it into the buffer */
      for (data_index=0; data_index <  data_length; data_index++) {
        tx_buffer[data_index] = lin_message_table[hlin->lin_instance][hlin->lin_msg_table_index].data[data_index];
      }

      /* calculate the checksum */
      checksum = lin_data_layer_checksum(hlin->lin_data_frame.pid,
        data_length,
        tx_buffer);
      
      /* add the checksum to the end of the buffer */
      tx_buffer[data_length] = checksum;

      /* TX the data over the UART */
      HAL_UART_Transmit(hlin->huart,tx_buffer,data_length + 1,1000);

      /* Reset the LIN state machine */
      hlin->lin_flags.lin_rx_data_available = 0;
      hlin->lin_data_frame.data_length = 0;
      hlin->lin_state = LIN_IDLE_AWAIT_BREAK;

      break;
    default:
      /* nothing to do for other states */
      /* TODO: currently LIN master is not supported */
      break;
  }
}

uint8_t lin_config(uint32_t msg_id, uint8_t *data)
{
  if (msg_id == LIN_CONFIG_MSG_ID_DATA) {
    /* store the data into the config data store */
    memcpy(lin_config_data.lin_data.data, data, 8);
    return 0;
  }

  lin_config_data.lin_cmd_type = data[0];
  lin_config_data.lin_channel = data[1];
  lin_config_data.lin_table_index = data[2];
  lin_config_data.lin_data.lin_msg_flags.is_active = data[3];
  lin_config_data.lin_data.lin_msg_flags.lin_node_action = data[4];
  lin_config_data.lin_data.PID = data[5];
  lin_config_data.lin_data.len = data[6];

  switch (lin_config_data.lin_cmd_type) {
    case LIN_CMD_LOAD_MSG:
      /* load the message table with the passed data */ 	
      lin_message_table[lin_config_data.lin_channel][lin_config_data.lin_table_index] = lin_config_data.lin_data;
      break;
      
    case LIN_CMD_READ_MSG: 
      /* read the data out of the message table and send it back to the host */
      /* TODO: Figure out a good way to do this if it's really needed */
      break;
    
    case LIN_CMD_ENABLE_MSG: 
      /* set the flag to enable one message */
      lin_message_table[lin_config_data.lin_channel][lin_config_data.lin_table_index].lin_msg_flags.is_active = 1;
      break;

    case LIN_CMD_DISABLE_MSG: 
      /* clear the flag to disable one message */
      lin_message_table[lin_config_data.lin_channel][lin_config_data.lin_table_index].lin_msg_flags.is_active = 0;
      break;

    case LIN_CMD_DISABLE_ALL_MSGS:
      lin_disable_all_messages();
      break;

    case LIN_CMD_ERASE_ALL_MSGS:
      lin_erase_message_table();
      break;   

    case LIN_CMD_ENABLE_LIN:
      /* to do, need a lookup table to get the baud info */
      //static void lin_usart_enable(LIN_USART_HW[lin_ctrl_message->LIN_Channel], NEED BAUD LOOKUP ETC.);
      break;  
    
    case LIN_CMD_DISABLE_LIN:
      /* to do, need a lookup table to get the pin info */
      //lin_usart_disable(LIN_USART_HW[lin_ctrl_message->LIN_Channel], NEED PIN LOOKUP);
      break;
 
    default:
      break;
  }

  return 0;
}

static void lin_erase_message_table(void)
{
  /* routine to zero out all data in the message table */
  for (uint8_t usart_chan = 0; usart_chan < LIN_MAX_USART_CHAN; usart_chan++) {
    for (uint8_t lin_message_index = 0; lin_message_index < LIN_MAX_MESSAGE_ITEMS; lin_message_index++) {
      lin_message_table[usart_chan][lin_message_index].lin_msg_flags.is_active = 0;
      //lin_message_table[usart_chan][lin_message_index].lin_msg_flags.lin_node_action = US_LINMR_NACT_IGNORE;
      lin_message_table[usart_chan][lin_message_index].PID = 0x00;
      lin_message_table[usart_chan][lin_message_index].len = 0x00;
      for (uint8_t data_index = 0; data_index <  LIN_MAX_DATA_BYTES; data_index++) {
        lin_message_table[usart_chan][lin_message_index].data[data_index] = 0x00;
      }
    }
  }
}

static void lin_disable_all_messages(void)
{
  /* routine to disable all messages in the message table */
  for (uint8_t usart_chan = 0; usart_chan < LIN_MAX_USART_CHAN; usart_chan++) {
    for (uint8_t lin_message_index = 0; lin_message_index < LIN_MAX_MESSAGE_ITEMS; lin_message_index++) {
      lin_message_table[usart_chan][lin_message_index].lin_msg_flags.is_active = 0;
    }
  }
}

static FlagStatus lin_hw_check_for_break(LIN_HandleTypeDef* hlin)
{
  FlagStatus result = __HAL_UART_GET_FLAG(hlin->huart, UART_FLAG_LBDF);
  if (result == SET) {
    __HAL_UART_CLEAR_FLAG(hlin->huart, UART_CLEAR_LBDF);
  }

  return result;
}

static uint8_t lin_data_layer_checksum(uint8_t pid, uint8_t length, const uint8_t* data_ptr)
{
  uint16_t chk = pid;
  // loop over data bytes
  for (uint8_t i = 0; i < length; i++)
  {
    chk += (uint16_t) (data_ptr[i]);
    if (chk>255)
      chk -= 255;
  }
  chk = (uint8_t)(0xFF - ((uint8_t) chk));   // bitwise invert
  // return frame checksum
  return (uint8_t)chk;
}

static void lin_can_gateway_tx(LIN_HandleTypeDef* hlin)
{
  struct gs_host_frame frame;

  /* zero out the buffer data for good measure */
  for(uint8_t i = 0 ; i < 8 ; i++) {
    frame.classic_can.data[i]  = 0x00;
  }

  frame.can_id = (LIN_GATEWAY_MSG_ID | CAN_EFF_FLAG);
  frame.can_dlc = 8;

  frame.classic_can.data[0] = hlin->lin_data_frame.pid & LIN_ID_MASK;
  frame.classic_can.data[1] = hlin->lin_data_frame.data_length;

  /* can only use the first 6 bytes of data in the LIN frame due to CAN data limits */
  uint8_t lin_gateway_tx_len = hlin->lin_data_frame.data_length;
  if (lin_gateway_tx_len > 6) {
    lin_gateway_tx_len = 6;
  }

  for (uint8_t i = 0; i < lin_gateway_tx_len; i++) {
    frame.classic_can.data[i + 2] = hlin->lin_data_frame.lin_data_buffer[i];
  }
   
  /* Set this frame just like a regular CAN frame - reuse the can_send routine */
  can_send(&hfdcan1, &frame);
}
