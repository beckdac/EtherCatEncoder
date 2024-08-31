# EtherCatEncoder
Open-source EtherCat Spindle Encoder using GDS32F407 and LAN9252

This code was developed for this board from AliExpress:
![GD32+LAN9252 Development Board](doc/gd32_lan9252.jpg?raw=true "GD32+LAN9252 Development Board")

### Attribution:
This code was based on many sources, but instrumental was [MetalMusings](https://www.youtube.com/watch?v=wOtMrlHCCic). They had a working encoder and this uses that same PDO format and much of the encoder counting structure was derived from that video.

The `systick` pieces are lifted from the GD32 Arduino core and I think they probably originally come from stm32 cores.  I tried to leave those headers intact, even if I modified the content, e.g. see `systick.cpp` in the `soes` library.

On the SOES note, yes, the entire `SOES` library is in here.  It is "forked" from [ecat_serv](https://github.com/kubabuda/ecat_servo/tree/main/examples/SOES_LAN9252). The `hal` and some of the `esi` is rewritten for the GD32 instead of the STM32.
