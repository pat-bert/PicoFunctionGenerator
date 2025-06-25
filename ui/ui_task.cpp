#include "ui/ui_task.hpp"

#include "ui/ui_visitor.hpp"

#include "waveform/waveform_data.hpp"

#include "hardware/adc.h"
#include "hardware/gpio.h"

#include <array>
#include <iostream>

namespace Ui
{
    constexpr uint32_t timerPeriodMs{20U};

    bool Task::timer_callback(repeating_timer_t *rt)
    {
        lv_tick_inc(timerPeriodMs);
        return true; // Return true to keep the timer running
    }

    void Task::lv_draw_sw_i1_convert_to_vtiled_pages_first(const void *buf, uint32_t buf_size, uint32_t width, uint32_t height,
                                                           void *out_buf, uint32_t out_buf_size, bool bit_order_lsb)
    {
        LV_ASSERT(buf && out_buf);
        LV_ASSERT(width % 8 == 0 && height % 8 == 0);
        LV_ASSERT(buf_size >= (width / 8) * height);
        LV_ASSERT(out_buf_size >= buf_size);

        lv_memset(out_buf, 0, out_buf_size);

        const uint8_t *src_buf = (uint8_t *)buf;
        uint8_t *dst_buf = (uint8_t *)out_buf;

        for (uint32_t y = 0; y < height; y++)
        {
            for (uint32_t x = 0; x < width; x++)
            {
                uint32_t src_index = (x + (y * width)) >> 3;
                uint32_t dst_index = x + (y >> 3) * width;
                uint8_t bit = (src_buf[src_index] >> (7 - (x % 8))) & 0x01;
                if (bit_order_lsb)
                {
                    dst_buf[dst_index] |= (bit << (y % 8));
                }
                else
                {
                    dst_buf[dst_index] |= (bit << (7 - (y % 8)));
                }
            }
        }
    }

    void Task::flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
    {
        // Skip the first 8 bytes of the buffer, which are used for metadata
        px_map += 8;

        auto *displayDriver = static_cast<DisplayDriverType *>(lv_display_get_user_data(display));

        constexpr uint16_t buffer_size = DisplayDriverType::get_buffer_size();
        uint8_t buffer_converted[buffer_size];

        lv_draw_sw_i1_convert_to_vtiled_pages_first(
            px_map,
            buffer_size,
            displayDriver->get_width(),
            displayDriver->get_height(),
            buffer_converted,
            buffer_size,
            true);

        displayDriver->write_area(
            static_cast<uint8_t>(area->x1),
            static_cast<uint8_t>(area->x2),
            static_cast<uint8_t>(area->y1),
            static_cast<uint8_t>(area->y2),
            buffer_converted);

        lv_display_flush_ready(display);
    }

    void Task::rounder_cb(lv_event_t *e)
    {
        lv_area_t *area = static_cast<lv_area_t *>(lv_event_get_param(e));

        /* Round the height to the nearest multiple of 8 */
        area->y1 = (area->y1 & ~0x7);
        area->y2 = (area->y2 | 0x7);
    }

    void Task::run()
    {
        m_uiBuilder.createUi();

        repeating_timer_t timer{};
        // Negative value means that the period is between starts of repeated calls
        add_repeating_timer_ms(-timerPeriodMs, timer_callback, nullptr, &timer);

        std::array<Waveform::ChannelData, numberOfChannels> waveFormDataArray{
            Waveform::SawtoothData{true, 1000U, 4095U, true},
            Waveform::TriangleData{true, 500U, 2048U}};

        while (true)
        {
            for (int i = 0; i < numberOfChannels; ++i)
            {
                adc_select_input(i);
                const uint16_t adcValue = adc_read();
                m_uiBuilder.setAmplitude(i, adcValue);

                const bool isChannelEnabled = !gpio_get(enablePins[i]);
                m_uiBuilder.setEnabled(i, isChannelEnabled);

                const EnableVisitor enableVisitor{isChannelEnabled};
                const AmplitudeVisitor amplitudeVisitor{adcValue};

                std::visit(enableVisitor, waveFormDataArray[i]);
                std::visit(amplitudeVisitor, waveFormDataArray[i]);
            }

            const uint32_t timeUntilNextRunMs = lv_timer_handler();

            xStreamBufferSend(m_messageBufferHandle,
                              waveFormDataArray.data(),
                              sizeof(waveFormDataArray),
                              portMAX_DELAY);

            sleep_ms(100U);
        }
    }

    void Task::init()
    {
        std::cout << "Starting UI Task..." << std::endl;

        m_i2cDisplay.init();

        m_displayDriver.init();
        m_displayDriver.clear_display();
        m_displayDriver.inverted(false);
        m_displayDriver.flipped(true);
        m_displayDriver.reverse_cols(true);
        sleep_ms(5000);

        adc_init();

        constexpr uint32_t adcLowRipplePin{23U};
        gpio_init(adcLowRipplePin);
        gpio_set_dir(adcLowRipplePin, GPIO_OUT);
        gpio_put(adcLowRipplePin, true); // Enable low ripple mode

        for (int i = 0; i < numberOfChannels; ++i)
        {
            adc_gpio_init(adcPins[i]);
            gpio_init(enablePins[i]);
            gpio_set_dir(enablePins[i], GPIO_IN);
            gpio_pull_up(enablePins[i]);
        }

        lv_init();

        lv_display_t *display = lv_display_create(DisplayDriverType::get_width(), DisplayDriverType::get_height());
        lv_theme_t *theme = lv_theme_mono_init(
            display,        // Active display
            true,           // Enable dark mode
            LV_FONT_DEFAULT // Default font
        );

        lv_display_set_theme(display, theme);
        lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);
        lv_display_set_user_data(display, static_cast<void *>(&m_displayDriver));

        lv_display_set_buffers(display, &m_displayBuffer0, nullptr, sizeof(m_displayBuffer0), LV_DISPLAY_RENDER_MODE_FULL);
        lv_display_set_flush_cb(display, flush_cb);
        lv_display_add_event_cb(display, rounder_cb, LV_EVENT_INVALIDATE_AREA, display);

        lv_indev_t* indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_ENCODER);
        lv_indev_set_read_cb(indev, [](lv_indev_t *indev, lv_indev_data_t *data) {
            // Handle encoder input here
            data->state = LV_INDEV_STATE_REL; // Example: set state to released
            data->enc_diff = 0; // Example: no change in encoder position
        });
    }
}