#ifndef HAL_HPP_INCLUDED
#define HAL_HPP_INCLUDED

#include "i2c/i2c_pico.hpp"

#include "mcp4725.hpp"
#include "sh1106.hpp"

using DisplayDriverType = Lcd::SH1106_128x64<I2C::I2CPicoPIO>;
using DacDriverType = Dac::MCP4725<I2C::I2CPicoHw>;

constexpr uint8_t sdaDac0{16U};
constexpr uint8_t sclDac0{17U};
constexpr uint8_t sdaDac1{18U};
constexpr uint8_t sclDac1{19U};
constexpr uint8_t sdaLcd{14U};
constexpr uint8_t sclLcd{15U};
constexpr uint8_t pwm0{0U};
constexpr uint8_t pwm1{2U};

constexpr uint16_t i2cSpeedKHz{400U};

#endif