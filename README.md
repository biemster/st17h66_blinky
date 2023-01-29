# ST17H66_blinky
Minimal code to blink the LED on Lenze st17h66 based tags

It will start as a Keil project, but with the aim to move to GCC
- [x] Copy https://github.com/biemster/st17h66_RF/ and rename to Lenze_blinky
- [x] Reduce to just a single file, which turns on the LED
- [x] Convert to GCC what we have so far
- [ ] Add a timer using IRQ to blink the LED
- [ ] Activate deep sleep when the LED is off, without SRAM retention
- [ ] Convert project to GCC
