#include "waveform_task.hpp"

// Own includes
#include "hal.hpp"
#include "waveform_data.hpp"

// Pico SDK includes
#include "hardware/pwm.h"

// Std C++ includes
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <variant>
#include <functional>

namespace Waveform
{
    void waveform_task()
    {
        constexpr uint8_t numberOfChannels{2U};

        std::array<uint, numberOfChannels> sdaDacs{sdaDac0, sdaDac1};
        std::array<uint, numberOfChannels> sclDacs{sclDac0, sclDac1};
        std::array<DacInterfaceDriverType, numberOfChannels> i2cDrivers{};
        std::array<DacDriverType, numberOfChannels> dacArray{};
        std::array<uint, numberOfChannels> pwmPins{pwm0, pwm1};
        std::array<uint, numberOfChannels> pwmSlices{};

        for (size_t i = 0; i < numberOfChannels; ++i)
        {
            i2cDrivers[i] = DacInterfaceDriverType{i2c_get_instance(i), i2cSpeedKHz, sdaDacs[i], sclDacs[i]};
            if (!i2cDrivers[i].init())
            {
                std::cout << "Failed to initialize I2C driver for DAC " << i << std::endl;
                return;
            }

            dacArray[i] = DacDriverType{&(i2cDrivers[i]), DacDriverType::I2CAddr::VariantA0_PinA00};
            if (!dacArray[i].isConnected())
            {
                std::cout << "Failed to connect to DAC " << i << std::endl;
                return;
            }

            dacArray[i].setInputCode(0, DacDriverType::CmdType::EEPROM_Mode, DacDriverType::PowerMode::Off_500kOhm);

            gpio_set_function(pwmPins[i], GPIO_FUNC_PWM);
            pwmSlices[i] = pwm_gpio_to_slice_num(pwmPins[i]);
        }

        std::array<ChannelData, 2> waveFormDataArray{
            SawtoothData{true, 0U, 500U, 4095U, true},
            TriangleData{true, 1U, 500U, 4095U}};

        WaveformVisitor visitor{dacArray, pwmSlices};

        while (true)
        {
            for (auto &waveFormData : waveFormDataArray)
            {
                std::visit(visitor, waveFormData);
            }
        }
    }

    bool pwm_set_freq_duty(uint32_t slice_num, uint32_t chan, uint32_t freq,
                           int duty_cycle)
    {
        // f_PWM = f_SYS / ((TOP + 1) * (DIV_INT + DIV_FRAC/16))
        // TOP is 16 bits, DIV_INT is 8 bits, DIV_FRAC is 4 bits

        uint8_t clk_divider = 0;
        uint32_t wrap = 0;
        uint32_t clock_div = 0;
        uint32_t clock = clock_get_hz(clk_sys);

        if (freq < 8 && freq > clock)
        {
            // This is the frequency range of generating a PWM in RP2040 at 125MHz
            return false;
        }

        for (clk_divider = 1; clk_divider < UINT8_MAX; clk_divider++)
        {
            // Find clock_division to fit current frequency
            clock_div = clock / clk_divider;
            wrap = clock_div / freq;
            if (clock_div / UINT16_MAX <= freq && wrap <= UINT16_MAX)
            {
                break;
            }
        }
        if (clk_divider < UINT8_MAX)
        {
            // Only considering whole number division
            pwm_set_clkdiv_int_frac(slice_num, clk_divider, 0);
            pwm_set_wrap(slice_num, (uint16_t)wrap);
            pwm_set_chan_level(slice_num, chan,
                               (uint16_t)((((uint16_t)(duty_cycle == 100 ? (wrap + 1) : wrap)) * duty_cycle) / 100));
        }
        else
        {
            return false;
        }

        return true;
    }

} // namespace Waveform