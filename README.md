# ST17H66_blinky
Minimal code to blink the LED on Lenze st17h66 based tags, based on
- https://github.com/AloyseTech/PHY6202_GCC
- https://github.com/biemster/st17h66_RF/

It will start as a Keil project, but with the aim to move to GCC
- [x] Copy biemster/st17h66_RF and rename to Lenze_blinky
- [x] Reduce to just a single file, which turns on the LED
- [x] Convert to GCC what we have so far
- [x] Add a timer using IRQ to blink the LED
- [x] Activate deep sleep when the LED is off
- [X] Convert project to single source file, GCC only
- [x] Compilation instructions

# How to compile and flash
The compiled firmware is available in the `build/` directory, but can be compiled easily.
You will need an ARM GCC compiler and utils, like `arm-none-eabi-` on Linux, and the CMSIS headers:
```bash
$ git clone https://github.com/ARM-software/CMSIS_5
$ git clone https://github.com/biemster/st17h66_blinky
$ cd st17h66_blinky
$ make
```
To flash to the ST17h66, connect an UART to P9 and P10 (pin 5 and 6), and run
```bash
$ ./flash_st17h66.py
```
It has an USB UART hardcoded on `/dev/ttyUSB0`, so if your UART lives somewhere else you'll need to change this.

This repository is now considered finished, and will not likely recieve updates besides bug fixes. Further developtments will be done on https://github.com/biemster/st17h66_RF/.
