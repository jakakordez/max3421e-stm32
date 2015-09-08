# MAX3421E project for STM32

> The MAX3421E adds USB host or peripheral capability to any system with an SPI interface.
>
> -- <cite>Maxim Integrated</cite>

This repository contains a Keil project for STM32F429 Discovery board interfacing with MAX3421 USB controller. The code is mostly ported from [original example for MCB2130 evaluation board](http://www.maximintegrated.com/en/app-notes/index.mvp/id/3936).

Configuration is generated with STM32Cube, so it is based on HAL drivers

# Interface pinout

##SPI5
- PF6 - CS
- PF7 - SCK
- PF8 - MISO
- PF9 - MOSI
