Drive WS2812 neopixels using the SMI of the raspberry pi (in progress).

Based on github.com/jbentham/rpi

Pre-requisites
==============

## SMI kernel module loaded

In `/boot/config.txt`:
```
dtoverlay=smi
dtoverlay=smi-dev
```

## Update base address for peripheral physical memory to match RPi variant
In `rpi_dma_utils.h`:
```
// Location of peripheral registers in physical memory
#define PHYS_REG_BASE   PI_01_REG_BASE
#define PI_01_REG_BASE  0x20000000  // Pi Zero or 1
#define PI_23_REG_BASE  0x3F000000  // Pi 2 or 3
#define PI_4_REG_BASE   0xFE000000  // Pi 4
```
