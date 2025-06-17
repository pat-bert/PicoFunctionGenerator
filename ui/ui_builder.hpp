#ifndef UI_BUILDER_HPP_INCLUDED
#define UI_BUILDER_HPP_INCLUDED

#include "hal.hpp"

#include "lvgl/lvgl.h"

#include <array>

namespace Ui
{
    struct ChannelUI
    {
        lv_obj_t *m_labelAdc;
        lv_obj_t *m_enabled;
    };

    class UiBuilder
    {
    public:
        void createUi();

        void setAdcValue(size_t channelIndex, uint16_t value);

        void setEnabled(size_t channelIndex, bool enabled);

    private:
        std::array<ChannelUI, numberOfChannels> m_channelUIs{};
    };
}

#endif