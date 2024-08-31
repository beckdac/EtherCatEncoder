# EtherCatEncoder
Open-source EtherCat Spindle Encoder using GDS32F407 and LAN9252

This code was developed for this board from AliExpress:
![GD32+LAN9252 Development Board](doc/gd32_lan9252.jpg?raw=true "GD32+LAN9252 Development Board")

This board has 8 LEDs and 8 switches connected to a GD32F407ZG. The GD32F407ZG has SPI and an FSMC bus to a LAN9252 which has the PHY and EEPROM for doing EtherCat.  The GD32F407ZG has to do some configuration oof the LAN9252 over SPI and then the LAN9252 can do some basic responding and the GD32F407ZG can handle inputs and as fast as possible update the LAN9252 state over SPI.  Nothing fancy.  The board is capable of supporting an IRQ and SYNC0 / SYNC1 connections with the GD32F407ZG.  The code in this repo sets up those ISRs, but does not use them.

Switch 3 and switch 4 are encoder inputs to TIMER1.  Switch 5 has a rising edge external interrupt for the rotational index.  The other switches are ignored.  This selection is largely based on the TIMER1 CH0 and CH1 assignments as well as the IRQ and SYNC0/SYNC1 isr lines that are accomodated but not used. Switch 1 and 2 can be used for low speed things as can 4, 7 and 8.

The LEDs have the following meaning:
* LED1: on when GPIO has been initialized
* LED2: on when TIMER1 has been initialized
* LED3: on when the LAN9252 has been initialized and responded
* LED4: toggles every 100 update cycles with the LAN9252

### Attribution:
This code was based on many sources, but instrumental was [MetalMusings](https://www.youtube.com/watch?v=wOtMrlHCCic). They had a working encoder and this uses that same PDO format and much of the encoder counting structure was derived from that video.

The `systick` pieces are lifted from the GD32 Arduino core and I think they probably originally come from stm32 cores.  I tried to leave those headers intact, even if I modified the content, e.g. see `systick.cpp` in the `soes` library.

On the SOES note, yes, the entire `SOES` library is in here.  It is "forked" from [ecat_serv](https://github.com/kubabuda/ecat_servo/tree/main/examples/SOES_LAN9252). The `hal` and some of the `esi` is rewritten for the GD32 instead of the STM32.
