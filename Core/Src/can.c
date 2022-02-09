/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair
Copyright (c) 2022 Ryan Edwards (changes for STM32G4 and CAN-FD)

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

#include <string.h>
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "can.h"
#include "led.h"
#include "usbd_gs_can.h"

static uint8_t can_rx_data_buff[64];
static uint8_t can_tx_data_buff[64];

extern LED_HandleTypeDef hled1;
extern LED_HandleTypeDef hled2;
extern LED_HandleTypeDef hled3;

extern TIM_HandleTypeDef htim6;
extern QueueHandle_t queue_to_hostHandle;
extern USBD_HandleTypeDef hUSB;

static void can_parse_error_status(FDCAN_ProtocolStatusTypeDef *status, struct gs_host_frame *frame);

/** @brief Function to initialize the FDCAN handle
 *  @param FDCAN_HandleTypeDef *hcan - The pointer to the handle.
 *  @param FDCAN_GlobalTypeDef *instance - pointer to the instance name.
 *  @retval None
 */
void can_init(FDCAN_HandleTypeDef *hcan, FDCAN_GlobalTypeDef *instance)
{
  if (instance == FDCAN1) {
    MX_FDCAN1_Init();
  }
  else if(instance == FDCAN2) {
    MX_FDCAN2_Init();
  }
  else if(instance == FDCAN3) {
    MX_FDCAN3_Init();
  }
  else {
    /* invalid channel */
  }
}

/** @brief Function to set the CAN bit timing registers - will not set until can_enable() is executed
 *  @param FDCAN_HandleTypeDef *hcan - The pointer to the handle.
 *  @param uint16_t brp - The prescale value
 *  @param uint16_t phase_seg1 - The time phase segment 1 value
 *  @param uint16_t phase_seg2 - The time phase segment 2 value
 *  @param uint8_t sjw - The sync jump width value
 *  @retval None
 */
void can_set_bittiming(FDCAN_HandleTypeDef *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
  if ((brp > 0) && (brp <= 1024)
    && (phase_seg1 > 0) && (phase_seg1 <= 16)
    && (phase_seg2 > 0) && (phase_seg2 <= 8)
    && (sjw > 0) && (sjw <= 4))
  {
    hcan->Init.NominalPrescaler = brp;
    hcan->Init.NominalTimeSeg1 = phase_seg1;
    hcan->Init.NominalTimeSeg2 = phase_seg2;
    hcan->Init.NominalSyncJumpWidth = sjw;
  }
}

/** @brief Function to set the CAN-FD (data) bit timing registers - will not set until can_enable() is executed
 *  @param FDCAN_HandleTypeDef *hcan - The pointer to the handle.
 *  @param uint16_t brp - The prescale value (data)
 *  @param uint16_t phase_seg1 - The time phase segment 1 value (data)
 *  @param uint16_t phase_seg2 - The time phase segment 2 value (data)
 *  @param uint8_t sjw - The sync jump width value (data)
 *  @retval None
 */
void can_set_data_bittiming(FDCAN_HandleTypeDef *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
  if ((brp > 0) && (brp <= 1024)
      && (phase_seg1 > 0) && (phase_seg1 <= 16)
      && (phase_seg2 > 0) && (phase_seg2 <= 8)
      && (sjw > 0) && (sjw <= 4))
  {
    hcan->Init.DataPrescaler = brp;
    hcan->Init.DataTimeSeg1 = phase_seg1;
    hcan->Init.DataTimeSeg2 = phase_seg2;
    hcan->Init.DataSyncJumpWidth = sjw;
  }
}

/** @brief Function to enable the CAN channel
 *  @param FDCAN_HandleTypeDef *hcan - The pointer to the handle.
 *  @param bool loop_back - flag to indicate CAN mode should be in loopback.
 *  @param bool listen_only - flag to indicate CAN mode should be in monitor.
 *  @param bool one_shot - flag to indicate CAN mode should be in one shot.
 *  @param bool can_mode_fd - flag to indicate CAN supports CAN-FD.
 *  @retval None
 */
void can_enable(FDCAN_HandleTypeDef *hcan, bool loop_back, bool listen_only, bool one_shot, bool can_mode_fd)
{
  FDCAN_FilterTypeDef sFilterConfig;

  if (one_shot) {
    hcan->Init.AutoRetransmission = DISABLE;
  }
  else {
    hcan->Init.AutoRetransmission = ENABLE;
  }

  hcan->Init.Mode = FDCAN_MODE_NORMAL;

  if (loop_back && listen_only)
  {
    hcan->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
  }
  else if (loop_back)
  {
    hcan->Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
  }
  else if (listen_only)
  {
    hcan->Init.Mode = FDCAN_MODE_BUS_MONITORING;
  }
  else
  {
    //normal is good
  }

  if (can_mode_fd) {
    hcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  }
  else {
    hcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  }

  HAL_FDCAN_Init(hcan);

  /* Configure reception filter to Rx FIFO 0 on both FDCAN instances */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x7FF;

  HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig);

  /* Configure global filter on both FDCAN instances:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  // Start CAN using HAL
  HAL_FDCAN_Start(hcan);

  HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE
                                 | FDCAN_IT_LIST_BIT_LINE_ERROR
                                 | FDCAN_IT_LIST_PROTOCOL_ERROR, 0);

  led_set_active(&hled3);
}

/** @brief Function to isable the CAN channel
 *  @param FDCAN_HandleTypeDef *hcan - The pointer to the handle.
 *  @retval None
 */
void can_disable(FDCAN_HandleTypeDef *hcan)
{
  //Stop can using HAL
  HAL_FDCAN_Stop(hcan);
  HAL_FDCAN_DeactivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE
                                   | FDCAN_IT_LIST_BIT_LINE_ERROR
                                   | FDCAN_IT_LIST_PROTOCOL_ERROR);

  led_set_inactive(&hled3);
}

bool can_is_enabled(FDCAN_HandleTypeDef *hcan)
{
  if(hcan->State == HAL_FDCAN_STATE_BUSY) {
    return true;
  }
  else {
    return false;
  }
}

/** @brief Function to disable the CAN channel
 *  @param FDCAN_HandleTypeDef *hcan - The pointer to the handle.
 *  @param struct gs_host_frame *frame - The pointer to the host frame containing message data.
 *  @retval true if TX was successful, false if no successful.
 */
bool can_send(FDCAN_HandleTypeDef *hcan, struct gs_host_frame *frame)
{
  FDCAN_TxHeaderTypeDef TxHeader;

  TxHeader.DataLength = frame->can_dlc << 16;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  if (frame->can_id & CAN_RTR_FLAG) {
      TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
  }
  else {
      TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  }

  if (frame->can_id & CAN_EFF_FLAG) {
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.Identifier = frame->can_id & 0x1FFFFFFF;
  }
  else {
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.Identifier = frame->can_id & 0x7FF;
  }

  if (frame->flags & GS_CAN_FLAG_FD) {
    TxHeader.FDFormat = FDCAN_FD_CAN;
    if (frame->flags & GS_CAN_FLAG_BRS) {
      TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    }
    else {
      TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    }
    memcpy(can_tx_data_buff, frame->canfd.data, 64);
  }
  else {
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    memcpy(can_tx_data_buff, frame->classic_can.data, 8);
  }

  /* blink LED to indicate a CAN TX */
  led_indicate_rxtx(&hled1);

  if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, can_tx_data_buff) != HAL_OK) {
      return false;
  }
  else {
      return true;
  }
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef RxHeader;
  struct gs_host_frame frame;

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, can_rx_data_buff) != HAL_OK) {
    //Error_Handler();
  }

  frame.channel = USBD_GS_CAN_GetChannelNumber(&hUSB, hfdcan);
  frame.can_id = RxHeader.Identifier;

  if (RxHeader.IdType == FDCAN_EXTENDED_ID) {
    frame.can_id |= CAN_EFF_FLAG;
  }

  if (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME) {
    frame.can_id |= CAN_RTR_FLAG;
  }

  frame.can_dlc = (RxHeader.DataLength & 0x000F0000) >> 16;

  frame.echo_id = 0xFFFFFFFF; // not a echo frame
  frame.reserved = 0;
  frame.flags = 0;

  if (RxHeader.FDFormat == FDCAN_FD_CAN) {
    /* this is a CAN-FD frame */
    frame.flags = GS_CAN_FLAG_FD;
    if (RxHeader.BitRateSwitch == FDCAN_BRS_ON) {
      frame.flags |= GS_CAN_FLAG_BRS;
    }
    memcpy(frame.canfd.data, can_rx_data_buff, 64);
  }
  else {
    memcpy(frame.classic_can.data, can_rx_data_buff, 8);
  }

  /* put this CAN message into the queue to send to host */
  xQueueSendToBackFromISR(queue_to_hostHandle, &frame, NULL);
  /* blink LED to indicate a CAN RX */
  led_indicate_rxtx(&hled2);
}

/**
  * @brief  Error status callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  ErrorStatusITs indicates which Error Status interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Error_Status_Interrupts.
  * @retval None
  */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  FDCAN_ProtocolStatusTypeDef status;
  struct gs_host_frame frame;

  frame.channel = USBD_GS_CAN_GetChannelNumber(&hUSB, hfdcan);

  HAL_FDCAN_GetProtocolStatus(hfdcan, &status);
  can_parse_error_status(&status, &frame);

  /* QueuePut will only grab the bytes that match the size defined at initialization */
  xQueueSendToBackFromISR(queue_to_hostHandle, &frame, NULL);
}

/** @brief Function parse the error data returned from the error callback
 *  @param FDCAN_ProtocolStatusTypeDef *status - pointer to the status data
 *  @param struct gs_host_frame *frame - The pointer to the host frame containing message data.
 *  @retval None
 */
static void can_parse_error_status(FDCAN_ProtocolStatusTypeDef *status, struct gs_host_frame *frame)
{
  frame->echo_id = 0xFFFFFFFF;
  frame->can_id  = CAN_ERR_FLAG | CAN_ERR_CRTL;
  frame->can_dlc = CAN_ERR_DLC;
  frame->classic_can.data[0] = CAN_ERR_LOSTARB_UNSPEC;
  frame->classic_can.data[1] = CAN_ERR_CRTL_UNSPEC;
  frame->classic_can.data[2] = CAN_ERR_PROT_UNSPEC;
  frame->classic_can.data[3] = CAN_ERR_PROT_LOC_UNSPEC;
  frame->classic_can.data[4] = CAN_ERR_TRX_UNSPEC;
  frame->classic_can.data[5] = 0;
  frame->classic_can.data[6] = 0;
  frame->classic_can.data[7] = 0;

  if (status->BusOff == 1) {
    frame->can_id |= CAN_ERR_BUSOFF;
  }

  if (status->ErrorPassive == 1) {
    frame->classic_can.data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
  }
  else if (status->Warning) {
    frame->classic_can.data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
  }

  uint8_t lec = status->LastErrorCode;
  if (lec != 0) { /* protocol error */
    switch (lec) {
      case 0x01: /* stuff error */
        frame->can_id |= CAN_ERR_PROT;
        frame->classic_can.data[2] |= CAN_ERR_PROT_STUFF;
        break;
      case 0x02: /* form error */
        frame->can_id |= CAN_ERR_PROT;
        frame->classic_can.data[2] |= CAN_ERR_PROT_FORM;
        break;
      case 0x03: /* ack error */
        frame->can_id |= CAN_ERR_ACK;
        break;
      case 0x04: /* bit recessive error */
        frame->can_id |= CAN_ERR_PROT;
        frame->classic_can.data[2] |= CAN_ERR_PROT_BIT1;
        break;
      case 0x05: /* bit dominant error */
        frame->can_id |= CAN_ERR_PROT;
        frame->classic_can.data[2] |= CAN_ERR_PROT_BIT0;
        break;
      case 0x06: /* CRC error */
        frame->can_id |= CAN_ERR_PROT;
        frame->classic_can.data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
        break;
      default:
        break;
    }
  }
}
