#ifndef HAL_HPP_INCLUDED
#define HAL_HPP_INCLUDED

#include "i2c/i2c_pico.hpp"

#include "mcp4725.hpp"
#include "sh1106.hpp"

using DisplayDriverType = Lcd::SH1106_128x64<I2C::I2CPicoPIO>;
using DacInterfaceDriverType = I2C::I2CPicoHw;
using DacDriverType = Dac::MCP4725<DacInterfaceDriverType>;

constexpr uint pwm0{0U};
constexpr uint pwm1{2U};
constexpr uint sdaLcd{14U};
constexpr uint sclLcd{15U};
constexpr uint sdaDac0{16U};
constexpr uint sclDac0{17U};
constexpr uint sdaDac1{18U};
constexpr uint sclDac1{19U};
constexpr uint adc0{26U};
constexpr uint adc1{27U};

constexpr uint pioStatemachineDisplay{0U};

constexpr uint16_t i2cSpeedKHz{1000U};

#endif