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

#pragma once

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "gs_usb.h"


void can_init(FDCAN_HandleTypeDef *hcan, FDCAN_GlobalTypeDef *instance);
void can_set_bittiming(FDCAN_HandleTypeDef *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw);
void can_set_data_bittiming(FDCAN_HandleTypeDef *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw);
void can_enable(FDCAN_HandleTypeDef *hcan, bool loop_back, bool listen_only, bool one_shot, bool can_mode_fd);
void can_disable(FDCAN_HandleTypeDef *hcan);
bool can_is_enabled(FDCAN_HandleTypeDef *hcan);

bool can_send(FDCAN_HandleTypeDef *hcan, struct gs_host_frame *frame);

