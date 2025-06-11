/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tasks/ui_task.hpp"
#include "tasks/waveform_task.hpp"

#include "pico/multicore.h"

int main()
{
    stdio_init_all();
    sleep_ms(2000); // wait for USB serial to be ready

    printf("Raspberry Pico Function Generator\n");
    multicore_launch_core1(Ui::ui_task);

    Waveform::waveform_task();
}


