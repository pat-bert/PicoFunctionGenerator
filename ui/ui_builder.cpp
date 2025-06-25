#include "ui/ui_builder.hpp"

namespace Ui
{
    static void text_blink_animation_cb(void *label, int32_t v)
    {
        lv_obj_set_style_text_opa(static_cast<lv_obj_t *>(label), v, LV_PART_MAIN);
    }

    static void box_blink_animation_cb(void *box, int32_t v)
    {
        lv_obj_set_style_border_color(static_cast<lv_obj_t *>(box), lv_color_hex(v), LV_PART_MAIN);
    }

    void UiBuilder::createUi()
    {
        lv_obj_t *screen = lv_screen_active();

        lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_column(screen, 2, LV_PART_MAIN);

        for (int i = 0; i < numberOfChannels; ++i)
        {
            lv_obj_t *channelContainer = lv_obj_create(screen);
            lv_obj_set_flex_grow(channelContainer, 1); // Allow the container to grow
            lv_obj_set_flex_flow(channelContainer, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_height(channelContainer, lv_pct(100));
            lv_obj_set_style_border_width(channelContainer, 1, LV_PART_MAIN);
            lv_obj_set_style_bg_color(channelContainer, lv_color_white(), LV_PART_MAIN);
            lv_obj_set_style_border_color(channelContainer, lv_color_black(), LV_PART_MAIN);

            // Add blinking animation to container
            {
                lv_anim_t anim;
                lv_anim_init(&anim);
                lv_anim_set_var(&anim, channelContainer);
                lv_anim_set_values(&anim, lv_color_to_int(lv_color_white()), lv_color_to_int(lv_color_black())); // Toggle opacity between transparent and fully visible
                lv_anim_set_duration(&anim, 500);                                                                // Duration of one blink (500ms)
                lv_anim_set_reverse_duration(&anim, 500);                                                        // Playback duration (reverse animation)
                lv_anim_set_repeat_count(&anim, LV_ANIM_REPEAT_INFINITE);                                        // Repeat indefinitely
                lv_anim_set_exec_cb(&anim, box_blink_animation_cb);
                lv_anim_start(&anim);
            }

            // Create and style waveform type label
            m_channelUIs[i].m_waveformType = lv_label_create(channelContainer);
            lv_obj_set_style_text_color(m_channelUIs[i].m_waveformType, lv_color_black(), LV_PART_MAIN);
            lv_obj_set_style_bg_color(m_channelUIs[i].m_waveformType, lv_color_white(), LV_PART_MAIN);
            setWaveformType(i, "Sine"); // Initialize waveform type

            // Add blinking animation to the waveform type label
            {
                lv_anim_t anim;
                lv_anim_init(&anim);
                lv_anim_set_var(&anim, m_channelUIs[i].m_waveformType);
                lv_anim_set_values(&anim, LV_OPA_TRANSP, LV_OPA_COVER);   // Toggle opacity between transparent and fully visible
                lv_anim_set_duration(&anim, 500);                         // Duration of one blink (500ms)
                lv_anim_set_reverse_duration(&anim, 500);                 // Playback duration (reverse animation)
                lv_anim_set_repeat_count(&anim, LV_ANIM_REPEAT_INFINITE); // Repeat indefinitely
                lv_anim_set_exec_cb(&anim, text_blink_animation_cb);
                lv_anim_start(&anim);
            }

            // Create and style frequency label
            m_channelUIs[i].m_frequency = lv_label_create(channelContainer);
            lv_obj_set_style_text_color(m_channelUIs[i].m_frequency, lv_color_black(), LV_PART_MAIN);
            lv_obj_set_style_bg_color(m_channelUIs[i].m_frequency, lv_color_white(), LV_PART_MAIN);
            setFrequency(i, 0); // Initialize frequency to 0

            // Create and style amplitude label
            m_channelUIs[i].m_amplitude = lv_label_create(channelContainer);
            lv_obj_set_style_text_color(m_channelUIs[i].m_amplitude, lv_color_black(), LV_PART_MAIN);
            lv_obj_set_style_bg_color(m_channelUIs[i].m_amplitude, lv_color_white(), LV_PART_MAIN);
            setAmplitude(i, 0); // Initialize amplitude to 0

            // Create and style enabled label
            m_channelUIs[i].m_enabled = lv_label_create(channelContainer);
            lv_obj_set_style_text_color(m_channelUIs[i].m_enabled, lv_color_black(), LV_PART_MAIN);
            lv_obj_set_style_bg_color(m_channelUIs[i].m_enabled, lv_color_white(), LV_PART_MAIN);
            setEnabled(i, false); // Initialize enabled state to false
        }
    }

    void UiBuilder::setAmplitude(size_t channelIndex, uint16_t value)
    {
        if (channelIndex < m_channelUIs.size() && m_channelUIs[channelIndex].m_amplitude)
        {
            lv_label_set_text_fmt(m_channelUIs[channelIndex].m_amplitude, "A:%u", value);
        }
    }

    void UiBuilder::setFrequency(size_t channelIndex, uint32_t frequency)
    {
        if (channelIndex < m_channelUIs.size() && m_channelUIs[channelIndex].m_frequency)
        {
            // Set frequency with fixed width of four
            lv_label_set_text_fmt(m_channelUIs[channelIndex].m_frequency, "%3u Hz", frequency);
        }
    }

    void UiBuilder::setEnabled(size_t channelIndex, bool enabled)
    {
        if (channelIndex < m_channelUIs.size() && m_channelUIs[channelIndex].m_enabled)
        {
            lv_label_set_text(m_channelUIs[channelIndex].m_enabled, enabled ? "On" : "Off");
        }
    }

    void UiBuilder::setWaveformType(size_t channelIndex, const char *waveFormType)
    {
        if (channelIndex < m_channelUIs.size() && m_channelUIs[channelIndex].m_waveformType)
        {
            lv_label_set_text(m_channelUIs[channelIndex].m_waveformType, waveFormType);
        }
    }
}