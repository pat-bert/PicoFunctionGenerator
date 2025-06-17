#ifndef UI_VISITOR_HPP_INCLUDED
#define UI_VISITOR_HPP_INCLUDED

#include "waveform/waveform_data.hpp"

namespace Ui
{
    struct EnableVisitor
    {
        EnableVisitor(bool isEnabled) : m_isEnabled(isEnabled) {}

        void operator()(Waveform::WaveformData &data) const
        {
            data.setEnabled(m_isEnabled);
        }

        bool m_isEnabled{false};
    };

    struct AmplitudeVisitor
    {
        AmplitudeVisitor(uint16_t amplitude) : m_amplitude(amplitude) {}

        template <typename T>
        void operator()(Waveform::AnalogWaveformData<T> &data) const
        {
            data.setAmplitude(m_amplitude);
        }

        void operator()(Waveform::WaveformData &data) const
        {
            // No action for digital waveforms, as they do not have amplitude
        }

        uint16_t m_amplitude{0U};
    };
}

#endif