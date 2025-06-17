#ifndef WAVEFORM_DATA_HPP_INCLUDED
#define WAVEFORM_DATA_HPP_INCLUDED

#include <cmath>
#include <variant>

namespace Waveform
{
    class WaveformData
    {
    public:
        explicit WaveformData(bool enabled)
            : m_enabled(enabled)
        {
        }

        bool isEnabled() const
        {
            return m_enabled;
        }

        void setEnabled(bool enabled)
        {
            m_enabled = enabled;
        }

    private:
        bool m_enabled{false}; // true = enabled, false = disabled
    };

    class DigitalWaveformData : public WaveformData
    {
    public:
        explicit DigitalWaveformData(bool enabled)
            : WaveformData(enabled)
        {
        }
    };

    class RectangleData : public DigitalWaveformData
    {
    public:
        explicit RectangleData(bool enabled, uint32_t frequency, uint32_t dutyCycle)
            : DigitalWaveformData(enabled), m_frequency(frequency), m_dutyCycle(dutyCycle)
        {
        }

        uint32_t getFrequency() const
        {
            return m_frequency;
        }

        uint32_t getDutyCycle() const
        {
            return m_dutyCycle;
        }

    private:
        uint32_t m_frequency{1000U}; // Frequency in Hz, default 1kHz
        uint32_t m_dutyCycle{50U};   // Duty cycle in percent, default 50%
    };

    template <typename Derived>
    class AnalogWaveformData : public WaveformData
    {
    public:
        explicit AnalogWaveformData(bool enabled, uint16_t amplitude)
            : WaveformData(enabled), m_amplitude(amplitude)
        {
        }

        uint16_t next()
        {
            // This function should be overridden in derived classes to provide the next value
            return static_cast<Derived *>(this)->nextImpl();
        }

        uint16_t getAmplitude() const
        {
            return m_amplitude;
        }

        void setAmplitude(uint16_t amplitude)
        {
            m_amplitude = amplitude;
        }

    private:
        uint16_t m_amplitude{0U}; // Amplitude of the waveform, default 0
    };

    class ConstantData : public AnalogWaveformData<ConstantData>
    {
    public:
        explicit ConstantData(bool enabled, uint16_t amplitude)
            : AnalogWaveformData<ConstantData>(enabled, amplitude)
        {
        }

        uint16_t nextImpl()
        {
            // Return the constant amplitude value
            return getAmplitude();
        }
    };

    class SawtoothData : public AnalogWaveformData<SawtoothData>
    {
    public:
        explicit SawtoothData(bool enabled, uint32_t frequency, uint16_t amplitude, bool inc)
            : AnalogWaveformData<SawtoothData>(enabled, amplitude), m_frequency(frequency), m_inc{inc}
        {
            m_counter = inc ? 0 : amplitude; // Initialize counter based on increasing or decreasing
            int16_t stepCount = i2cSpeedKHz * 1000 / (bitsPerVoltageCommand * m_frequency);
            m_step = getAmplitude() / (stepCount - 1);
            assert(m_step > 0); // Ensure step is positive
        }

        uint16_t nextImpl()
        {
            if (m_inc)
            {
                m_counter += m_step;
                if (m_counter >= getAmplitude())
                {
                    m_counter = 0; // Reset to 0 when reaching amplitude
                }
            }
            else
            {
                m_counter -= m_step;
                if (m_counter <= 0)
                {
                    m_counter = getAmplitude(); // Reset to amplitude when reaching 0
                }
            }

            return m_counter;
        }

    private:
        uint16_t m_step;
        uint32_t m_frequency;
        bool m_inc; // true = increasing, false = decreasing
        int32_t m_counter;
    };

    class TriangleData : public AnalogWaveformData<TriangleData>
    {
    public:
        explicit TriangleData(bool enabled, uint32_t frequency, uint16_t amplitude)
            : AnalogWaveformData<TriangleData>(enabled, amplitude), m_frequency(frequency)
        {
            // 29 Bits per I2C transfer, transfer speed is 400KHz, maximum 4096 steps per period
            // 400000 / (29 * 4096) = 2.4Hz
            // 29 / 400 kHz = 0.0000725 sec
            int16_t stepCount = i2cSpeedKHz * 1000 / (bitsPerVoltageCommand * m_frequency);
            m_step = 2 * getAmplitude() / (stepCount - 1);
            assert(m_step > 0); // Ensure step is positive
        }

        uint16_t nextImpl()
        {
            if (m_inc)
            {
                m_counter += m_step;
                if (m_counter >= getAmplitude())
                {
                    m_inc = false;
                    m_counter = getAmplitude();
                }
            }
            else
            {
                m_counter -= m_step;
                if (m_counter <= 0)
                {
                    m_inc = true;
                    m_counter = 0;
                }
            }

            return m_counter;
        }

    private:
        uint16_t m_step;
        uint32_t m_frequency{1000U}; // Frequency in Hz, default 1kHz
        int32_t m_counter{0};
        bool m_inc{true}; // true = increasing, false = decreasing
    };

    class SineData : public AnalogWaveformData<SineData>
    {
    public:
        explicit SineData(bool enabled, uint32_t frequency, uint16_t amplitude)
            : AnalogWaveformData<SineData>(enabled, amplitude), m_frequency(frequency)
        {
            int16_t stepCount = i2cSpeedKHz * 1000 / (bitsPerVoltageCommand * m_frequency);
            m_step = 360 / (stepCount - 1);
            assert(m_step > 0); // Ensure step is positive
        }

        uint16_t nextImpl()
        {
            // Calculate the next sine value based on frequency, amplitude
            m_counter += m_step; // Increment angle based on frequency
            if (m_counter >= 360)
            {
                m_counter -= 360; // Wrap around angle
            }
            return static_cast<uint16_t>(getAmplitude() * (1.0 + sinf(static_cast<float>(m_counter) * static_cast<float>(M_PI) / 180.0))); // Sine wave formula
        }

    private:
        uint16_t m_step;
        uint32_t m_counter{0};       // Angle in degrees
        uint32_t m_frequency{1000U}; // Frequency in Hz, default 1kHz
    };

    using ChannelData = std::variant<std::monostate, RectangleData, SawtoothData, TriangleData, SineData>;
}

#endif