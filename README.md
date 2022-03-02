# CANNEDPI_SW_GSUSB_CANFD
 A CAN-FD port for the STM32G4 using the gs_usb driver
## Description
This is an STM32 firmware which supports an interface to the gs_usb CAN driver.  The USB device code was ported from candleLight (https://github.com/candle-usb/candleLight_fw). This firmware is tailored for use on the CannedPI custom hardware (https://github.com/ryedwards/CANNEDPIHAT-Hardware). I currently do NOT have any plans to make this hardware available for sale due to multiple factors. It's an open design made in KiCad so the world is your oyster.

Since the code was developed around the CubeMX HAL from STM you should be able to port to most STM32 devices without much tear up.  I was able to port this code to an STM32H7 with very little modification to make it work (see the branch).

## Features
A few new enhancements this has over existing firmwares for gs_usb:
- Uses FreeRTOS.  May be overkill but I like using the queue design and using an RTOS allows for future scalability. 
- Full CAN-FD implementation with support for gs_usb CAN-FD when it becomes available.  Until then this firmware works fine in classic mode with the current driver.
- Support for multiple CAN channels.  The gs_usb driver currently only supports 2 channels. This firmware supports up 3 channels which can be changed via a compiler constant as the gs_usb driver is updated (default is 3).
- Since the CannedPI has an onboard LSE and coin cell it has an accurate RTC. Rather than add another RTC chip onto the board I've created an I2C library to emulate the popular DS3231 RTC chip. The Raspberry PI simply sees the code running on the STM32 as an I2C ds3231 chip and communicates using the existing Linux drivers. ~~(not yet integrated)~~ It's there now!
- Support for LIN. Use dummy CAN messages to configure and monitor the LIN bus. ~~(not yet integrated)~~ It's in there now!
- USART passthrough to that I can use the Raspberry Pi UART console via the STLINK virtual com port.

## Developer Notes
The code is written with the free STM32 Cube IDE. To use this code the only prerequisite is to download and install the IDE.  The CubeMX .ioc file is also included in case you need to regenerate code for your own custom hardware or a different chip.

You can modify the number of support channels by changing the #NUM_CAN_CHANNELS compiler option.  Any value 1->3 can be used.

Post any questions or comments in the issues list.
