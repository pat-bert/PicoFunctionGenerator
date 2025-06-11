#ifndef UI_TASK_HPP_INCLUDED
#define UI_TASK_HPP_INCLUDED

#include "hal.hpp"

#include "lvgl/lvgl.h"

#include <cstdint>

namespace Ui
{
    void lv_draw_sw_i1_convert_to_vtiled_pages_first(const void *buf, uint32_t buf_size, uint32_t width, uint32_t height,
                                                     void *out_buf, uint32_t out_buf_size, bool bit_order_lsb);

    void flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map);

    static void rounder_cb(lv_event_t *e);

    void ui_task();
}

#endif