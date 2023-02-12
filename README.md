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
- [ ] Convert project to single source file, GCC only
