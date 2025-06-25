#ifndef UI_BUILDER_HPP_INCLUDED
#define UI_BUILDER_HPP_INCLUDED

#include "hal.hpp"

#include "lvgl/lvgl.h"

#include <array>

namespace Ui
{
    struct ChannelUI
    {
        lv_obj_t *m_waveformType;
        lv_obj_t *m_frequency;
        lv_obj_t *m_amplitude;
        lv_obj_t *m_enabled;
    };

    class UiBuilder
    {
    public:
        void createUi();

        void setAmplitude(size_t channelIndex, uint16_t value);

        void setFrequency(size_t channelIndex, uint32_t frequency);

        void setEnabled(size_t channelIndex, bool enabled);

        void setWaveformType(size_t channelIndex, const char* waveFormType);

    private:
        std::array<ChannelUI, numberOfChannels> m_channelUIs{};
    };
}

#endif