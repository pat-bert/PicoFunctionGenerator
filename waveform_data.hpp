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

struct ConstantData : WaveformData
{
    explicit ConstantData(bool enabled, uint8_t channel, uint16_t amplitude)
        : WaveformData(enabled, channel), m_amplitude(amplitude)
    {
    }

    uint16_t m_amplitude{0U};
};

struct SawtoothData : WaveformData
{
    explicit SawtoothData(bool enabled, uint8_t channel, uint32_t frequency, uint16_t amplitude, bool inc)
        : WaveformData(enabled, channel), m_frequency(frequency), m_amplitude(amplitude), m_inc{inc}
    {
    }

    uint32_t m_frequency;
    uint16_t m_amplitude;
    bool m_inc; // true = increasing, false = decreasing
    int32_t m_counter{0};
};

struct TriangleData : WaveformData
{
    explicit TriangleData(bool enabled, uint8_t channel, uint32_t frequency, uint16_t amplitude)
        : WaveformData(enabled, channel), m_frequency(frequency), m_amplitude(amplitude)
    {
    }

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