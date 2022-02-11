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
#ifndef __RTC_DS3231_H
#define __RTC_DS3231_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Exported defines -----------------------------------------------------------*/
#define RTC_DS3231_I2C_INSTANCE     I2C3

/* Exported functions --------------------------------------------------------*/
void rtc_ds3231_init(I2C_HandleTypeDef *hi2c);
void rtc_ds3231_listen_cplt_callback(I2C_HandleTypeDef *hi2c);
void rtc_ds3231_addr_callback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void rtc_ds3231_slave_tx_cplt_callback(I2C_HandleTypeDef *hi2c);
void rtc_ds3231_slave_rx_cplt_callback(I2C_HandleTypeDef *hi2c);


#ifdef __cplusplus
}
#endif

#endif /* __RTC_DS3231_H */
