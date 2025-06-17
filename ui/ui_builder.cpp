#include "ui/ui_builder.hpp"

namespace Ui
{
    static void event_handler(lv_event_t *e)
    {
        lv_event_code_t code = lv_event_get_code(e);

        if (code == LV_EVENT_VALUE_CHANGED)
        {
            lv_obj_t *label = static_cast<lv_obj_t *>(lv_event_get_user_data(e));
            LV_LOG_USER("Toggled");
        }
    }

    void UiBuilder::createUi()
    {
        lv_obj_t *screen = lv_screen_active();

        for (int i = 0; i < numberOfChannels; ++i)
        {
            lv_align_t alignmentAdc = (i == 0) ? LV_ALIGN_TOP_LEFT : LV_ALIGN_TOP_RIGHT;
            lv_align_t alignmentEnabled = (i == 0) ? LV_ALIGN_BOTTOM_LEFT : LV_ALIGN_BOTTOM_RIGHT;

            m_channelUIs[i].m_labelAdc = lv_label_create(screen);
            lv_label_set_text_fmt(m_channelUIs[i].m_labelAdc, "ADC%d:", i);
            lv_obj_align(m_channelUIs[i].m_labelAdc, alignmentAdc, 0, 0);

            m_channelUIs[i].m_enabled = lv_label_create(screen);
            lv_label_set_text(m_channelUIs[i].m_enabled, "Disabled");
            lv_obj_align(m_channelUIs[i].m_enabled, alignmentEnabled, 0, 0);
        }
    }

    void UiBuilder::setAdcValue(size_t channelIndex, uint16_t value)
    {
        if (channelIndex < m_channelUIs.size() && m_channelUIs[channelIndex].m_labelAdc)
        {
            lv_label_set_text_fmt(m_channelUIs[channelIndex].m_labelAdc, "ADC%d: %u", channelIndex, value);
        }
    }

    void UiBuilder::setEnabled(size_t channelIndex, bool enabled)
    {
        if (channelIndex < m_channelUIs.size() && m_channelUIs[channelIndex].m_enabled)
        {
            lv_label_set_text(m_channelUIs[channelIndex].m_enabled, enabled ? "Enabled" : "Disabled");
        }
    }
}