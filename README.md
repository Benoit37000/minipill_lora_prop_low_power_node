# MiniPill LoRa Hardware version 1.x

Please look at https://www.iot-lab.org/blog/370/ for general information on the starting project. In this this file I will share some software specific information.

## Special software version: Proprietary LoRAWan lib
In an earlier project I used a ATTiny84 with a RFM95W LoRa module https://www.iot-lab.org/blog/101/. I used and build special code to fit the 8k flash memory.

Now I have ported this code/library to the STM32L051C8T6 chip. Without BME280 code it uses about 47% of the 64k flash memory and uses 1.3uA in sleep mode.

This version only supports ABP, but does not use the any type of RTOS/timeticks for timing. It is quite straightforward code.

## Update 2020-11-26
All hardware versions 1.x are supported by this software

## other information
Look for more information on the MiniPill LoRa and programming it on the original project page/gitlab page.
