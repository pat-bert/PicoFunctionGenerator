#ifndef WAVEFORM_TASK_HPP_INCLUDED
#define WAVEFORM_TASK_HPP_INCLUDED

#include "cstdint"

namespace Waveform
{
    template <class... Ts>
    struct overloaded : Ts...
    {
        using Ts::operator()...;
    };

    // explicit deduction guide (not needed as of C++20)
    template <class... Ts>
    overloaded(Ts...) -> overloaded<Ts...>;

    void waveform_task();

    bool pwm_set_freq_duty(uint32_t slice_num, uint32_t chan, uint32_t freq, int duty_cycle);
}

#endif