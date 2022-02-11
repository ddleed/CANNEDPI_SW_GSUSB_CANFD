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
  * @file    rtc_ds3231.c
  * @author  R. Edwards
  * @brief   This code allows for any master device on I2C to see this slave device
  *          as a DS3231 RTC device.  It uses the STM32 RTC registers to accurately
  *          keep time and transfer it back and forth to the master.
  *          The master sees this code as if it were a DS3231 so any code written to
  *          interface with the DS3231 will work.  This has been tested with the "hwclock"
  *          utility running on a Raspberry Pi.
*/

#include "rtc_ds3231.h"
#include "main.h"

/* This structure aligns to the register map of the DS3231 starting with register 0x00 */
/* https://datasheets.maximintegrated.com/en/ds/DS3231.pdf - Page 11 */
typedef struct {
  uint8_t seconds_bcd;
  uint8_t minutes_bcd;
  uint8_t hours_bcd;
  uint8_t week_day_bcd;
  uint8_t date_bcd;
  uint8_t month_bcd;
  uint8_t year_bcd;
  uint8_t alarm_1_sec_bcd;
  uint8_t alarm_1_min_bcd;
  uint8_t alarm_1_hrs_bcd;
  uint8_t alarm_1_day_bcd;
  uint8_t alarm_1_date_bcd;
  uint8_t alarm_2_sec_bcd;
  uint8_t alarm_2_min_bcd;
  uint8_t alarm_2_hrs_bcd;
  uint8_t alarm_2_day_bcd;
  uint8_t alarm_2_date_bcd;
  uint8_t control;
  uint8_t control_status;
  uint8_t aging_offset;
  uint8_t temp_msb;
  uint8_t temp_lsb;
} rtc_ds3231_reg_data_t;

extern RTC_HandleTypeDef hrtc;

static uint8_t rtc_ds3231_reg_buff_index = 0;
static uint8_t rtc_ds3231_reg_buff[sizeof(rtc_ds3231_reg_data_t)];
static uint8_t rtc_ds3231_rx_buff_index = 0;
static uint8_t rtc_ds3231_rx_buff[sizeof(rtc_ds3231_reg_data_t)+1];
static uint8_t rtc_ds3231_pending_write = 0;

static void rtc_ds3231_read_rtc_to_regs(rtc_ds3231_reg_data_t* rtc_time);
static void rtc_ds3231_write_regs_to_rtc(rtc_ds3231_reg_data_t* rtc_time);


/** @brief Function to initialize the I2C callback routine
 *  @param I2C_HandleTypeDef *hi2c - Pointer to the i2c handle
 *  @retval None
 */
void rtc_ds3231_init(I2C_HandleTypeDef *hi2c)
{
  HAL_I2C_EnableListen_IT(hi2c);
}

/** @brief Function to handle data available once listen is complete on i2c
 *  @param I2C_HandleTypeDef *hi2c - Pointer to the i2c handle
 *  @retval None
 */
void rtc_ds3231_listen_cplt_callback(I2C_HandleTypeDef *hi2c)
{
  /* TODO: Technically we have the ability to do two alarms and use the functionality of the DS3231 registers
   *       to manage the RTC alarm registers but need to understand the value
   *       By design the DS3231 registers are already written we just need to port to the HAL RTC alarm then decide
   *       what do to when the alarm triggers
   */
  rtc_ds3231_reg_data_t* rtc_time;

  if (rtc_ds3231_pending_write == 1) {
    /* If this was a data write then update the "registers" */
    /* copy of RX buffer into the register buffer starting with the given offset and length */
    /* always refresh the RTC against the register buffer - shadow copy */
    /* update the RTC buffer */
    rtc_time =  (rtc_ds3231_reg_data_t*)&rtc_ds3231_reg_buff[0];
    rtc_ds3231_read_rtc_to_regs(rtc_time);

    /* using the first rx byte as the start register write to each "register" */
    /* rollover of the index is handled with the modulo check */
    for (uint8_t index=0; index < rtc_ds3231_rx_buff_index-1; index++) {
    	rtc_ds3231_reg_buff[(rtc_ds3231_rx_buff[0] + index) % sizeof(rtc_ds3231_reg_data_t)] = rtc_ds3231_rx_buff[index+1];
    }

    /* update the uC RTC registers */
    rtc_ds3231_write_regs_to_rtc(rtc_time);
  }

  rtc_ds3231_pending_write = 0;

  HAL_I2C_EnableListen_IT(hi2c); // Restart
}

/** @brief Function to handle data available once the slave address is matched on i2c
 *  @param I2C_HandleTypeDef *hi2c - Pointer to the i2c handle
 *  @retval None
 */
void rtc_ds3231_addr_callback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  UNUSED(AddrMatchCode);
  rtc_ds3231_reg_data_t* rtc_time;

  if(TransferDirection == I2C_DIRECTION_TRANSMIT) {
	  rtc_ds3231_rx_buff_index = 0;
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rtc_ds3231_rx_buff[rtc_ds3231_rx_buff_index], 1, I2C_FIRST_FRAME);
    rtc_ds3231_pending_write = 1;
  }
  else {
    /* the MASTER node is requesting data - the first byte of the RX buffer has the start index */
	  rtc_ds3231_pending_write = 0;

    /* update the RTC buffer */
    rtc_time =  (rtc_ds3231_reg_data_t*)&rtc_ds3231_reg_buff[0];
    rtc_ds3231_read_rtc_to_regs(rtc_time);

    /* dummy config for now - make it intelligent later */
    rtc_ds3231_reg_buff[14] = 0x1C;
    rtc_ds3231_reg_buff[15] = 0x08;

    rtc_ds3231_reg_buff_index = rtc_ds3231_rx_buff[0];
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &rtc_ds3231_reg_buff[rtc_ds3231_reg_buff_index], 1, I2C_FIRST_FRAME);
  }
}

/** @brief Function to handle the process once the slave data has been transmitted
 *  @param I2C_HandleTypeDef *hi2c - Pointer to the i2c handle
 *  @retval None
 */
void rtc_ds3231_slave_tx_cplt_callback(I2C_HandleTypeDef *hi2c)
{
  /* handle the circular readback style of the i2c method of the ds3231 */
  if (rtc_ds3231_reg_buff_index++ >= sizeof(rtc_ds3231_reg_data_t)) {
	  rtc_ds3231_reg_buff_index = 0;
  }
  HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &rtc_ds3231_reg_buff[rtc_ds3231_reg_buff_index], 1, I2C_NEXT_FRAME);
}

/** @brief Function to handle the process once the slave data has been received
 *  @param I2C_HandleTypeDef *hi2c - Pointer to the i2c handle
 *  @retval None
 */
void rtc_ds3231_slave_rx_cplt_callback(I2C_HandleTypeDef *hi2c)
{
  /* only continue reading data if we haven't overflowed the buffer */
  if (rtc_ds3231_rx_buff_index++ < sizeof(rtc_ds3231_reg_data_t)) {
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rtc_ds3231_rx_buff[rtc_ds3231_rx_buff_index], 1, I2C_NEXT_FRAME);
  }

}

/** @brief Helper function to read the RTC data out of the STM32 into the ds3231 mock registers
 *  @param rtc_ds3231_reg_data_t* rtc_time - Pointer to the mock registers
 *  @retval None
 */
static void rtc_ds3231_read_rtc_to_regs(rtc_ds3231_reg_data_t* rtc_time)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BCD);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BCD);

  rtc_time->seconds_bcd = stimestructureget.Seconds;
  rtc_time->minutes_bcd = stimestructureget.Minutes;
  rtc_time->hours_bcd = stimestructureget.Hours;

  rtc_time->week_day_bcd = sdatestructureget.WeekDay;
  rtc_time->month_bcd = sdatestructureget.Month;
  rtc_time->date_bcd = sdatestructureget.Date;
  rtc_time->year_bcd = sdatestructureget.Year;
}

/** @brief Helper function to read the ds3231 mock registers into the STM32 RTC structures
 *  @param rtc_ds3231_reg_data_t* rtc_time - Pointer to the mock registers
 *  @retval None
 */
static void rtc_ds3231_write_regs_to_rtc(rtc_ds3231_reg_data_t* rtc_time)
{
  RTC_DateTypeDef sdatestructureset;
  RTC_TimeTypeDef stimestructureset;

  stimestructureset.Hours = rtc_time->hours_bcd;
  stimestructureset.Minutes = rtc_time->minutes_bcd;
  stimestructureset.Seconds = rtc_time->seconds_bcd;
  stimestructureset.SubSeconds = 0x0;
  stimestructureset.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructureset.StoreOperation = RTC_STOREOPERATION_SET;
  if (HAL_RTC_SetTime(&hrtc, &stimestructureset, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sdatestructureset.WeekDay = rtc_time->week_day_bcd;
  sdatestructureset.Month = rtc_time->month_bcd;
  sdatestructureset.Date = rtc_time->date_bcd;
  sdatestructureset.Year = rtc_time->year_bcd;

  if (HAL_RTC_SetDate(&hrtc, &sdatestructureset, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
}
