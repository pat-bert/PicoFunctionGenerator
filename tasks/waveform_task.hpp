#ifndef WAVEFORM_TASK_HPP_INCLUDED
#define WAVEFORM_TASK_HPP_INCLUDED

#include "hal.hpp"
#include "waveform_data.hpp"

#include "hardware/pwm.h"

#include <array>
#include <cstdint>
#include <iostream>

namespace Waveform
{
    struct WaveformVisitor
    {
        WaveformVisitor(std::array<DacDriverType, 2> &dacArray, std::array<uint, 2> &pwmSlices)
            : m_dacArray(dacArray), m_pwmSlices(pwmSlices) {}

        void operator()(const RectangleData &arg) const;

        template <typename T>
        void operator()(AnalogWaveformData<T> &arg) const
        {
            auto &dac = m_dacArray[arg.getChannel()];
            if (!arg.isEnabled())
            {
                std::cout << "Disabling channel " << arg.getChannel() << std::endl;
                dac.setInputCode(0, DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::Off_500kOhm);
                return;
            }

            dac.setInputCode(arg.next(), DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::On);
        }

        std::array<DacDriverType, 2> &m_dacArray;
        std::array<uint, 2> &m_pwmSlices;
    };

    void waveform_task();

    bool pwm_set_freq_duty(uint32_t slice_num, uint32_t chan, uint32_t freq, int duty_cycle);
}

#endif