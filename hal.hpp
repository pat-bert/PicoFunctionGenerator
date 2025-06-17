#ifndef HAL_HPP_INCLUDED
#define HAL_HPP_INCLUDED

#include "i2c/i2c_pico.hpp"

#include "drivers/mcp4725.hpp"
#include "drivers/sh1106.hpp"

using DisplayDriverType = Lcd::SH1106_128x64<I2C::I2CPicoPIO>;
using DacInterfaceDriverType = I2C::I2CPicoHw;
using DacDriverType = Dac::MCP4725<DacInterfaceDriverType>;

constexpr uint sdaLcd{14U};
constexpr uint sclLcd{15U};
constexpr uint adcPins[]{26U, 27U};
constexpr uint enablePins[]{2U, 3U};
constexpr uint pioStatemachineDisplay{0U};

constexpr uint8_t numberOfChannels{2U};
constexpr uint sdaDacs[]{16U, 18U};
constexpr uint sclDacs[]{17U, 19U};
constexpr uint pwmPins[]{0U, 1U};

constexpr uint16_t i2cSpeedKHz{1000U};

constexpr uint8_t bitsPerVoltageCommand{18U}; // 16 Bits for 12-Bit voltage and mode + 2x ACK, adress becomes neglibile in continuous transfer

#endif