/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/divider.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

#include "hal.hpp"
#include "ui_task.hpp"
#include "waveform_data.hpp"

#include <cmath>
#include <variant>
#include <cstring>
#include <array>

template <class... Ts>
struct overloaded : Ts...
{
    using Ts::operator()...;
};
// explicit deduction guide (not needed as of C++20)
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

bool pwm_set_freq_duty(uint32_t slice_num, uint32_t chan, uint32_t freq,
                       int duty_cycle);

int main()
{
    stdio_init_all();
    sleep_ms(2000); // wait for USB serial to be ready

    printf("Raspberry Pico Function Generator\n");
    multicore_launch_core1(Ui::ui_task);

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
            printf("I2C driver %zu initialization failed\n", i);
            return -1;
        }

        dacArray[i] = DacDriverType{&(i2cDrivers[i]), DacDriverType::I2CAddr::VariantA0_PinA00};
        if (!dacArray[i].isConnected())
        {
            printf("DAC %zu is not connected\n", i);
            return -1;
        }

        dacArray[i].setInputCode(0, DacDriverType::CmdType::EEPROM_Mode, DacDriverType::PowerMode::Off_500kOhm);

        gpio_set_function(pwmPins[i], GPIO_FUNC_PWM);
        pwmSlices[i] = pwm_gpio_to_slice_num(pwmPins[i]);
    }

    std::array<ChannelData, numberOfChannels> waveFormDataArray{
        SawtoothData{true, 0U, 500U, 4095U, true},
        TriangleData{true, 1U, 500U, 4095U}};

    const overloaded setupVisitor{
        [&pwmSlices](const RectangleData &arg)
        {
            // Find out which PWM slice is connected to GPIO
            uint slice_num = pwmSlices[arg.m_channel];
            pwm_set_freq_duty(slice_num, PWM_CHAN_A, arg.m_frequency, arg.m_dutyCycle);
            pwm_set_enabled(slice_num, arg.m_enabled);
        },
        [&dacArray](const ConstantData &arg)
        {
            auto &dac = dacArray[arg.m_channel];

            if (!arg.m_enabled)
            {
                printf("Disabling channel %d\n", arg.m_channel);
                dac.setInputCode(0, DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::Off_500kOhm);
                return;
            }

            dac.setInputCode(arg.m_amplitude, DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::On);
        },
        [&dacArray](SawtoothData &arg)
        {
            auto &dac = dacArray[arg.m_channel];

            if (!arg.m_enabled)
            {
                printf("Disabling channel %d\n", arg.m_channel);
                dac.setInputCode(0, DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::Off_500kOhm);
                return;
            }

            // 29 Bits per I2C transfer, transfer speed is 400KHz, maximum 4096 steps per period
            // 400000 / (29 * 4096) = 2.4Hz
            // 29 / 400 kHz = 0.0000725 sec
            int16_t stepCount = i2cSpeedKHz * 1000 / (35 * arg.m_frequency);
            int16_t step = arg.m_amplitude / (stepCount - 1);

            if (arg.m_inc)
            {
                dac.setInputCode(arg.m_counter, DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::On);
            }
            else
            {
                dac.setInputCode(arg.m_amplitude - arg.m_counter, DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::On);
            }

            // Update counter for next step
            arg.m_counter += step;
            if (arg.m_counter >= arg.m_amplitude)
            {
                arg.m_counter = 0;
            }
        },
        [&dacArray](TriangleData &arg)
        {
            auto &dac = dacArray[arg.m_channel];

            if (!arg.m_enabled)
            {
                printf("Disabling channel %d\n", arg.m_channel);
                dac.setInputCode(0, DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::Off_500kOhm);
                return;
            }

            // 29 Bits per I2C transfer, transfer speed is 400KHz, maximum 4096 steps per period
            // 400000 / (29 * 4096) = 2.4Hz
            // 29 / 400 kHz = 0.0000725 sec
            int16_t stepCount = i2cSpeedKHz * 1000 / (35 * arg.m_frequency);
            int16_t step = 2 * arg.m_amplitude / (stepCount - 1);

            dac.setInputCode(arg.m_counter, DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::On);
            if (arg.m_inc)
            {
                arg.m_counter += step;
                if (arg.m_counter >= arg.m_amplitude)
                {
                    arg.m_inc = false;
                    arg.m_counter = arg.m_amplitude;
                }
            }
            else
            {
                arg.m_counter -= step;
                if (arg.m_counter <= 0)
                {
                    arg.m_inc = true;
                    arg.m_counter = 0;
                }
            }
        },
        [&dacArray](const SineData &arg) {}};

    while (true)
    {
        for (auto &waveFormData : waveFormDataArray)
        {
            std::visit(setupVisitor, waveFormData);
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
        clock_div = div_u32u32(clock, clk_divider);
        wrap = div_u32u32(clock_div, freq);
        if (div_u32u32(clock_div, UINT16_MAX) <= freq && wrap <= UINT16_MAX)
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
                           (uint16_t)div_u32u32((((uint16_t)(duty_cycle == 100 ? (wrap + 1) : wrap)) * duty_cycle), 100));
    }
    else
    {
        return false;
    }

    return true;
}
