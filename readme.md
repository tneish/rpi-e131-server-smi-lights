Drive WS2812 neopixels using the SMI of the raspberry pi (in progress).

Based on github.com/jbentham/rpi

Note (!!) RPi 5 does not have SMI.

Pre-requisites
==============




## Update base address for peripheral physical memory to match RPi variant
In `rpi_dma_utils.h`:
```
// Location of peripheral registers in physical memory
#define PHYS_REG_BASE   PI_4_REG_BASE
#define PI_01_REG_BASE  0x20000000  // Pi Zero or 1
#define PI_23_REG_BASE  0x3F000000  // Pi 2 or 3
#define PI_4_REG_BASE   0xFE000000  // Pi 4
```


Troubleshooting
===============

Check ARM is in "Low Peripheral mode" (default).
/boot/firmware/config.txt should not have arm_peri_high=1
