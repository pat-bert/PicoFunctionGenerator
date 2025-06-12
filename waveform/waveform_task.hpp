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
    void waveform_task();
}

#endif