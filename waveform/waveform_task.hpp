#ifndef WAVEFORM_TASK_HPP_INCLUDED
#define WAVEFORM_TASK_HPP_INCLUDED

#include "hal.hpp"
#include "waveform_data.hpp"
#include "waveform_visitor.hpp"

#include "hardware/pwm.h"

#include <array>
#include <cstdint>
#include <iostream>

namespace Waveform
{
    class Task
    {
    public:
        /// @brief Function to run the waveform generation task
        /// @details This function initializes the DAC drivers, sets up the PWM pins, and enters a loop to generate waveforms.
        ///          It uses a visitor pattern to handle different waveform data types.
        /// @note This function is intended to be run in a separate task or thread.
        void run();

    protected:
        void init();

    private:
        std::array<DacInterfaceDriverType, numberOfChannels> m_i2cDrivers{};
        std::array<DacDriverType, numberOfChannels> m_dacs{};
        std::array<WaveformVisitor, numberOfChannels> m_visitors{};
    };

    static void run_task_wrapper(void *arg)
    {
        Task task{};
        task.run();
    }
}

#endif