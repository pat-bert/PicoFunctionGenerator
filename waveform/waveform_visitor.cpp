#include "waveform_visitor.hpp"

#include "hardware/pwm.h"

namespace Waveform
{
    void WaveformVisitor::operator()(const RectangleData &arg) const
    {
        pwm_set_freq_duty(m_pwmSlice, PWM_CHAN_A, arg.getFrequency(), arg.getDutyCycle());
        pwm_set_enabled(m_pwmSlice, arg.isEnabled());
    }
}