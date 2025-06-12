#ifndef WAVEFORM_DATA_HPP_INCLUDED
#define WAVEFORM_DATA_HPP_INCLUDED

#include <variant>

struct WaveformData
{
    explicit WaveformData(bool enabled, uint8_t channel)
        : m_enabled(enabled), m_channel(channel)
    {
    }

    bool m_enabled{false}; // true = enabled, false = disabled
    uint8_t m_channel{0U}; // Channel number
};

struct RectangleData : WaveformData
{
    explicit RectangleData(bool enabled, uint8_t channel, uint32_t frequency, uint32_t dutyCycle)
        : WaveformData(enabled, channel), m_frequency(frequency), m_dutyCycle(dutyCycle)
    {
    }

    uint32_t m_frequency{1000U}; // Frequency in Hz, default 1kHz
    uint32_t m_dutyCycle{50U};   // Duty cycle in percent, default 50%
};

template <typename Derived>
struct AnalogWaveformData : WaveformData
{
    explicit AnalogWaveformData(bool enabled, uint8_t channel)
        : WaveformData(enabled, channel)
    {
    }

    uint16_t next()
    {
        // This function should be overridden in derived classes to provide the next value
        return static_cast<Derived *>(this)->nextImpl();
    }
};

struct ConstantData : AnalogWaveformData<ConstantData>
{
    explicit ConstantData(bool enabled, uint8_t channel, uint16_t amplitude)
        : AnalogWaveformData<ConstantData>(enabled, channel), m_amplitude(amplitude)
    {
    }

    uint16_t nextImpl()
    {
        // Return the constant amplitude value
        return m_amplitude;
    }

    uint16_t m_amplitude{0U};
};

struct SawtoothData : AnalogWaveformData<SawtoothData>
{
    explicit SawtoothData(bool enabled, uint8_t channel, uint32_t frequency, uint16_t amplitude, bool inc)
        : AnalogWaveformData<SawtoothData>(enabled, channel), m_frequency(frequency), m_amplitude(amplitude), m_inc{inc}
    {
        m_counter = inc ? 0 : amplitude; // Initialize counter based on increasing or decreasing
        int16_t stepCount = i2cSpeedKHz * 1000 / (35 * m_frequency);
        m_step = m_amplitude / (stepCount - 1);
    }

    uint16_t nextImpl()
    {
        if (m_inc)
        {
            m_counter += m_step;
            if (m_counter >= m_amplitude)
            {
                m_counter = 0; // Reset to 0 when reaching amplitude
            }
        }
        else
        {
            m_counter -= m_step;
            if (m_counter <= 0)
            {
                m_counter = m_amplitude; // Reset to amplitude when reaching 0
            }
        }

        return m_counter;
    }

    uint16_t m_step;
    uint32_t m_frequency;
    uint16_t m_amplitude;
    bool m_inc; // true = increasing, false = decreasing
    int32_t m_counter;
};

struct TriangleData : AnalogWaveformData<TriangleData>
{
    explicit TriangleData(bool enabled, uint8_t channel, uint32_t frequency, uint16_t amplitude)
        : AnalogWaveformData<TriangleData>(enabled, channel), m_frequency(frequency), m_amplitude(amplitude)
    {
        // 29 Bits per I2C transfer, transfer speed is 400KHz, maximum 4096 steps per period
        // 400000 / (29 * 4096) = 2.4Hz
        // 29 / 400 kHz = 0.0000725 sec
        int16_t stepCount = i2cSpeedKHz * 1000 / (35 * m_frequency);
        m_step = 2 * m_amplitude / (stepCount - 1);
    }

    uint16_t nextImpl()
    {
        if (m_inc)
        {
            m_counter += m_step;
            if (m_counter >= m_amplitude)
            {
                m_inc = false;
                m_counter = m_amplitude;
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

    uint16_t m_step;
    uint32_t m_frequency{1000U}; // Frequency in Hz, default 1kHz
    uint16_t m_amplitude;
    int32_t m_counter{0};
    bool m_inc{true}; // true = increasing, false = decreasing
};

struct SineData : WaveformData
{
    explicit SineData(bool enabled, uint8_t channel, uint32_t frequency, uint16_t amplitude, uint32_t phase)
        : WaveformData(enabled, channel), m_frequency(frequency), m_amplitude(amplitude), m_phase(phase)
    {
    }

    uint32_t m_frequency{1000U}; // Frequency in Hz, default 1kHz
    uint16_t m_amplitude;
    uint32_t m_phase;
};

using ChannelData = std::variant<RectangleData, SawtoothData, TriangleData, SineData>;

#endif