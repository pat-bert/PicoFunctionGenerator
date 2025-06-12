#ifndef WAVEFORM_VISITOR_HPP_INCLUDED
#define WAVEFORM_VISITOR_HPP_INCLUDED

#include "hal.hpp"
#include "waveform_data.hpp"

#include <iostream>

namespace Waveform
{
    bool pwm_set_freq_duty(uint32_t slice_num, uint32_t chan, uint32_t freq, int duty_cycle);

    class WaveformVisitor
    {
    public:
        WaveformVisitor() = default;

        WaveformVisitor(DacDriverType *dac, uint pwmSlice)
            : m_dac(dac), m_pwmSlice(pwmSlice) {}

        void operator()(const RectangleData &arg) const;

        template <typename T>
        void operator()(AnalogWaveformData<T> &arg) const
        {
            if (!arg.isEnabled())
            {
                m_dac->setInputCode(0, DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::Off_500kOhm);
                return;
            }

            m_dac->setInputCode(arg.next(), DacDriverType::CmdType::FastMode, DacDriverType::PowerMode::On);
        }

    private:
        DacDriverType *m_dac{nullptr};
        uint m_pwmSlice{0U};
    };
}

#endif