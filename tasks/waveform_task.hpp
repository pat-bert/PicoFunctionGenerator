#ifndef WAVEFORM_TASK_HPP_INCLUDED
#define WAVEFORM_TASK_HPP_INCLUDED

#include "cstdint"

namespace Waveform
{
    struct WaveformVisitor
    {
        WaveformVisitor(std::array<DacDriverType, 2> &dacArray, std::array<uint, 2> &pwmSlices)
            : m_dacArray(dacArray), m_pwmSlices(pwmSlices) {}

        void operator()(const RectangleData &arg) const
        {
            // Find out which PWM slice is connected to GPIO
            uint slice_num = m_pwmSlices[arg.m_channel];
            pwm_set_freq_duty(slice_num, PWM_CHAN_A, arg.m_frequency, arg.m_dutyCycle);
            pwm_set_enabled(slice_num, arg.m_enabled);
        }

        template <typename T>
        void operator()(AnalogWaveformData<T> &arg) const
        {
            auto &dac = m_dacArray[arg.m_channel];
            if (!arg.m_enabled)
            {
                std::cout << "Disabling channel " << arg.m_channel << std::endl;
                dac.setInputCode(0, DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::Off_500kOhm);
                return;
            }

            dac.setInputCode(arg.next(), DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::On);
        }

        void operator()(const SineData &arg) const
        {
            // Sine waveform implementation can be added here
            // For now, we leave it empty as a placeholder
        }

        std::array<DacDriverType, 2> &m_dacArray;
        std::array<uint, 2> &m_pwmSlices;
    };

    void waveform_task();

    bool pwm_set_freq_duty(uint32_t slice_num, uint32_t chan, uint32_t freq, int duty_cycle);
}

#endif