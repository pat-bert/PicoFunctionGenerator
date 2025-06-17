#ifndef WAVEFORM_VISITOR_HPP_INCLUDED
#define WAVEFORM_VISITOR_HPP_INCLUDED

#include "hal.hpp"
#include "monostate_visitor.hpp"

#include "waveform/waveform_data.hpp"

#include <iostream>

namespace Waveform
{
    bool pwm_set_freq_duty(uint32_t slice_num, uint32_t chan, uint32_t freq, int duty_cycle);

    /// @brief Visitor class for handling different waveform data types
    /// @details This class implements the visitor pattern to process different waveform data types.
    ///          Currently, there are overrides for RectangleData and AnalogWaveformData.
    class WaveformVisitor : public MonostateVisitor
    {
    public:
        using MonostateVisitor::operator();

        WaveformVisitor() = default;

        /// @brief Constructor that initializes the visitor with a DAC driver and PWM slice
        /// @param dac Pointer to the DAC driver instance
        /// @param pwmSlice The PWM slice number to be used for PWM operations
        explicit WaveformVisitor(DacDriverType *dac, uint pwmSlice)
            : m_dac(dac), m_pwmSlice(pwmSlice) {}

        /// @brief Visit method for RectangleData
        /// @details This method sets the PWM frequency and duty cycle based on the RectangleData parameters.
        /// @param arg The RectangleData instance containing frequency and duty cycle information
        void operator()(const RectangleData &arg) const;

        /// @brief Visit method for AnalogWaveformData
        /// @details This method sets the input code for the DAC based on the next value from the AnalogWaveformData.
        ///          If the waveform is disabled, it sets the input code to 0 and turns off the DAC.
        /// @tparam T The type of the AnalogWaveformData (e.g., ConstantData, SawtoothData, TriangleData, SineData)
        /// @param arg The AnalogWaveformData instance containing the next value to be set
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