/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Output PWM signals on pins 0 and 1

#include "pico/divider.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

#include "lvgl/lvgl.h"
#include "sh1106.hpp"
#include "i2c_pico.hpp"

#include "mcp4725.hpp"
#include "waveform_data.hpp"

#include <cmath>
#include <variant>
#include <cstring>

using DisplayDriverType = SH1106::SH1106_128x64;

template <class... Ts>
struct overloaded : Ts...
{
    using Ts::operator()...;
};
// explicit deduction guide (not needed as of C++20)
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

constexpr uint8_t SDA0{16U};
constexpr uint8_t SCL0{17U};
constexpr uint8_t SDA1{18U};
constexpr uint8_t SCL1{19U};
constexpr uint8_t PWM0{0U};
constexpr uint8_t PWM1{2U};

constexpr uint16_t i2cSpeedKHz{400U};

bool pwm_set_freq_duty(uint32_t slice_num, uint32_t chan, uint32_t freq,
                       int duty_cycle);

void lv_draw_sw_i1_convert_to_vtiled_pages_first(const void *buf, uint32_t buf_size, uint32_t width, uint32_t height,
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

void flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
    // Skip the first 8 bytes of the buffer, which are used for metadata
    px_map += 8;

    auto *displayDriver = static_cast<DisplayDriverType *>(lv_display_get_user_data(display));

    uint16_t buffer_size = DisplayDriverType::get_buffer_size();
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

static void rounder_cb(lv_event_t *e)
{
    lv_area_t *area = static_cast<lv_area_t *>(lv_event_get_param(e));

    /* Round the height to the nearest multiple of 8 */
    area->y1 = (area->y1 & ~0x7);
    area->y2 = (area->y2 | 0x7);
}

void core1_function()
{
    I2C::I2CPico i2cDisplay{i2c1};

    constexpr uint8_t column_offset{2};
    DisplayDriverType displayDriver{&i2cDisplay, SH1106::I2C_ADDR_PRIMARY, column_offset};
    displayDriver.init();
    displayDriver.clear_display();
    displayDriver.inverted(false);
    displayDriver.flipped(true);
    displayDriver.reverse_cols(true);
    sleep_ms(5000);

    static uint8_t buf1[SH1106::SH1106_128x64::get_buffer_size() + 8];

    lv_init();

    lv_display_t *display = lv_display_create(displayDriver.get_width(), displayDriver.get_height());
    lv_theme_t *theme = lv_theme_mono_init(
        display,        // Active display
        true,           // Enable dark mode
        LV_FONT_DEFAULT // Default font
    );

    lv_display_set_theme(display, theme);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);
    lv_display_set_user_data(display, static_cast<void *>(&displayDriver));

    lv_display_set_buffers(display, buf1, nullptr, sizeof(buf1), LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(display, flush_cb);
    lv_display_add_event_cb(display, rounder_cb, LV_EVENT_INVALIDATE_AREA, display);

    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello world");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    while (true)
    {
        uint32_t sleepMs{lv_timer_handler()};
        sleep_ms(sleepMs);
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(2000); // wait for USB serial to be ready

    printf("Raspberry Pico Function Generator\n");

    MCP4725_PICO dacArray[2]{};
    if (!dacArray[0].begin(MCP4725_PICO::MCP4725A0_Addr_A00, i2c0, i2cSpeedKHz, SDA0, SCL0, 50000))
    {
        printf("DAC0 not connected\n");
        return -1;
    }

    MCP4725_PICO dac1{};
    if (!dacArray[1].begin(MCP4725_PICO::MCP4725A0_Addr_A00, i2c1, i2cSpeedKHz, SDA1, SCL1, 50000))
    {
        printf("DAC1 not connected\n");
        return -1;
    }

    for (auto &dac : dacArray)
    {
        dac.setInputCode(0, MCP4725_PICO::MCP4725_EEPROM_Mode, MCP4725_PICO::MCP4725_PowerDown_500kOhm);
    }

    multicore_launch_core1(core1_function);

    gpio_set_function(PWM0, GPIO_FUNC_PWM);
    gpio_set_function(PWM1, GPIO_FUNC_PWM);

    uint scliceArray[2]{};
    scliceArray[0] = pwm_gpio_to_slice_num(PWM0);
    scliceArray[1] = pwm_gpio_to_slice_num(PWM1);

    ChannelData waveformData0{RectangleData{true, 0U, 250U, 30U}};
    // ChannelData waveformData1{TriangleData{true, 1U, 100U, 2000U}};

    const overloaded setupVisitor{
        [&scliceArray](const RectangleData &arg)
        {
            // Find out which PWM slice is connected to GPIO
            uint slice_num = scliceArray[arg.m_channel];
            printf("Executing rectangle: CH%d (slice%d), %d Hz %d%%\n", arg.m_channel, slice_num, arg.m_frequency, arg.m_dutyCycle);
            pwm_set_freq_duty(slice_num, PWM_CHAN_A, arg.m_frequency, arg.m_dutyCycle);
            pwm_set_enabled(slice_num, arg.m_enabled);
        },
        [&dacArray](const ConstantData &arg)
        {
            printf("Executing constant: CH%d, %d Ampl\n", arg.m_channel, arg.m_amplitude);
            auto &dac = dacArray[arg.m_channel];

            if (!arg.m_enabled)
            {
                printf("Disabling channel %d\n", arg.m_channel);
                dac.setInputCode(0, MCP4725_PICO::MCP4725_FastMode, MCP4725_PICO::MCP4725_PowerDown_500kOhm);
                return;
            }

            dac.setInputCode(arg.m_amplitude, MCP4725_PICO::MCP4725_FastMode, MCP4725_PICO::MCP4725_PowerDown_Off);
        },
        [&dacArray](const SawtoothData &arg)
        {
            printf("Executing saw tooth: CH%d, %d Hz %d Ampl, %s\n", arg.m_channel, arg.m_frequency, arg.m_amplitude, arg.m_inc ? "inc" : "dec");
            auto &dac = dacArray[arg.m_channel];

            if (!arg.m_enabled)
            {
                printf("Disabling channel %d\n", arg.m_channel);
                dac.setInputCode(0, MCP4725_PICO::MCP4725_FastMode, MCP4725_PICO::MCP4725_PowerDown_500kOhm);
                return;
            }

            // 29 Bits per I2C transfer, transfer speed is 400KHz, maximum 4096 steps per period
            // 400000 / (29 * 4096) = 2.4Hz
            // 29 / 400 kHz = 0.0000725 sec
            int16_t stepCount = i2cSpeedKHz * 1000 / (35 * arg.m_frequency);
            int16_t step = arg.m_amplitude / (stepCount - 1);

            if (arg.m_inc)
            {
                while (true)
                {
                    for (int16_t counter = 0; counter <= arg.m_amplitude; counter += step)
                    {
                        dac.setInputCode(counter, MCP4725_PICO::MCP4725_FastMode, MCP4725_PICO::MCP4725_PowerDown_Off);
                    }
                }
            }
            else
            {
                while (true)
                {
                    for (int16_t counter = arg.m_amplitude; counter >= 0; counter -= step)
                    {
                        dac.setInputCode(counter, MCP4725_PICO::MCP4725_FastMode, MCP4725_PICO::MCP4725_PowerDown_Off);
                    }
                }
            }
        },
        [&dacArray](const TriangleData &arg)
        {
            printf("Executing triangle: CH%d, %d Hz %d Ampl\n", arg.m_channel, arg.m_frequency, arg.m_amplitude);
            auto &dac = dacArray[arg.m_channel];

            if (!arg.m_enabled)
            {
                printf("Disabling channel %d\n", arg.m_channel);
                dac.setInputCode(0, MCP4725_PICO::MCP4725_FastMode, MCP4725_PICO::MCP4725_PowerDown_500kOhm);
                return;
            }

            // 29 Bits per I2C transfer, transfer speed is 400KHz, maximum 4096 steps per period
            // 400000 / (29 * 4096) = 2.4Hz
            // 29 / 400 kHz = 0.0000725 sec
            int16_t stepCount = i2cSpeedKHz * 1000 / (35 * arg.m_frequency);
            int16_t step = 2 * arg.m_amplitude / (stepCount - 1);

            while (true)
            {
                for (int16_t counter = 0; counter <= arg.m_amplitude; counter += step)
                {
                    dac.setInputCode(counter, MCP4725_PICO::MCP4725_FastMode, MCP4725_PICO::MCP4725_PowerDown_Off);
                }
                for (int16_t counter = arg.m_amplitude; counter >= 0; counter -= step)
                {
                    dac.setInputCode(counter, MCP4725_PICO::MCP4725_FastMode, MCP4725_PICO::MCP4725_PowerDown_Off);
                }
            }
        },
        [&dacArray](const SineData &arg)
        {
            printf("Executing sine\n");
        }};

    std::visit(setupVisitor, waveformData0);
    // std::visit(setupVisitor, waveformData1);

    while (true)
    {
        sleep_ms(1000);
    }
}

bool pwm_set_freq_duty(uint32_t slice_num, uint32_t chan, uint32_t freq,
                       int duty_cycle)
{
    // f_PWM = f_SYS / ((TOP + 1) * (DIV_INT + DIV_FRAC/16))
    // TOP is 16 bits, DIV_INT is 8 bits, DIV_FRAC is 4 bits

    uint8_t clk_divider = 0;
    uint32_t wrap = 0;
    uint32_t clock_div = 0;
    uint32_t clock = clock_get_hz(clk_sys);

    if (freq < 8 && freq > clock)
    {
        // This is the frequency range of generating a PWM in RP2040 at 125MHz
        return false;
    }

    for (clk_divider = 1; clk_divider < UINT8_MAX; clk_divider++)
    {
        // Find clock_division to fit current frequency
        clock_div = div_u32u32(clock, clk_divider);
        wrap = div_u32u32(clock_div, freq);
        if (div_u32u32(clock_div, UINT16_MAX) <= freq && wrap <= UINT16_MAX)
        {
            break;
        }
    }
    if (clk_divider < UINT8_MAX)
    {
        // Only considering whole number division
        pwm_set_clkdiv_int_frac(slice_num, clk_divider, 0);
        pwm_set_wrap(slice_num, (uint16_t)wrap);
        pwm_set_chan_level(slice_num, chan,
                           (uint16_t)div_u32u32((((uint16_t)(duty_cycle == 100 ? (wrap + 1) : wrap)) * duty_cycle), 100));
    }
    else
    {
        return false;
    }

    return true;
}
