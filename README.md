# Lenze_blinky
Minimal code to blink the LED on Lenze st17h66 based tags

It will start as a Keil project, but with the aim to move to GCC
- [ ] Copy https://github.com/biemster/st17h66_RF/ and renove to Lenze_blinky
- [ ] Reduce to just a single file, which turns on the LED
- [ ] Add a timer using IRQ to blink the LED
- [ ] Activate deep sleep when the LED is off, without SRAM retention
- [ ] Convert project to GCC
