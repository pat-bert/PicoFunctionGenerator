# Pico Function Generator

This project aims to create an entry-level function generator based on the Raspberry Pico (RP2040).

## Road map

- [x] Basic PWM based on GPIO (frequency and duty cycle can be set)
- [x] Integrated library to set analog voltage of two MCP4725 12-Bit DACs via I²C using i2c0 and i2c1
- [x] Integrated suitable library for SH1106 monochrome 1.3" display via I²C using i2c1
- [ ] Update display via PIO based separate I²C to not interfere with tight DAC loops
- [ ] Make DAC waveform non-blocking
- [ ] Create mock-up UI to show current waveform, frequency and enabled for each channel
- [ ] Add enable buttons for each channel
- [ ] Connect UI to actual channel data
- [ ] Add inputs to change waveform and frequency 
