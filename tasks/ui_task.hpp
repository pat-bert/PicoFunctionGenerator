#ifndef UI_TASK_HPP_INCLUDED
#define UI_TASK_HPP_INCLUDED

#include "hal.hpp"

#include "hardware/timer.h"

#include "lvgl/lvgl.h"

#include <cstdint>

namespace Ui
{
    constexpr uint8_t column_offset{2};

    /// @brief Class representing the UI task for the Pico Waveform Generator
    /// @details This class is responsible for initializing the display, creating UI elements, and updating the UI in a loop.
    class Task
    {
    public:
        /// @brief Function to run the UI task
        /// @details This function initializes the display, creates UI elements, and enters a loop to update the UI.
        ///          It uses LVGL for rendering the UI and handles ADC readings to display on the screen.
        /// @note This function is intended to be run in a separate task or thread.
        void run();

    protected:
        /// @brief Initializes the display and sets up the LVGL environment
        /// @details This function initializes the display driver, sets up the LVGL display and registers callbacks.
        void init();

        /// @brief Callback function for rounding the display area
        /// @details This function is called by LVGL to round the display area to the nearest multiple of 8 pixels in height.
        /// It modifies the area parameter to ensure that the height is a multiple of 8, which is required for the display driver.
        /// @param e Pointer to the LVGL event object
        static void rounder_cb(lv_event_t *e);

        /// @brief Callback function for flushing the display
        /// @details This function is called by LVGL to flush the display buffer to the actual display hardware.
        /// It converts the pixel map to a format suitable for the display and writes it to the display driver.
        /// @param display Pointer to the LVGL display object
        /// @param area Pointer to the area to flush
        /// @param px_map Pointer to the pixel map buffer
        static void flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map);

        /// @brief Callback function for the repeating timer
        /// @details This function is called periodically to update the LVGL internal time
        /// @param rt Pointer to the repeating timer object
        /// @return true if the timer shall be kept running, false if it should be stopped
        static bool timer_callback(repeating_timer_t *rt);

        /// @brief Converts a 1-bit pixel buffer to a vertical tiled page format
        /// @details This function converts a 1-bit pixel buffer to a vertical tiled page format
        ///          suitable for the display driver. It processes the buffer in a way that is compatible with the display's
        ///          vertical tiling and bit order requirements.
        /// @param buf Pointer to the input buffer containing the 1-bit pixel data
        /// @param buf_size Size of the input buffer in bytes
        /// @param width Width of the display in pixels
        /// @param height Height of the display in pixels
        /// @param out_buf Pointer to the output buffer where the converted data will be stored
        /// @param out_buf_size Size of the output buffer in bytes
        /// @param bit_order_lsb If true, the least significant bit is used first; otherwise, the most significant bit is used first
        static void lv_draw_sw_i1_convert_to_vtiled_pages_first(const void *buf, uint32_t buf_size, uint32_t width, uint32_t height,
                                                                void *out_buf, uint32_t out_buf_size, bool bit_order_lsb);

    private:
        I2C::I2CPicoPIO m_i2cDisplay{pio0, pioStatemachineDisplay, sdaLcd, sclLcd};
        DisplayDriverType m_displayDriver{&m_i2cDisplay, DisplayDriverType::I2CAddr::PRIMARY, column_offset};

        uint8_t m_displayBuffer0[DisplayDriverType::get_buffer_size() + 8];
    };

    static void run_task_wrapper()
    {
        Task task{};
        task.run();
    }
}

#endif