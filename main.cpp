// Own includes
#include "tasks/ui_task.hpp"
#include "tasks/waveform_task.hpp"

// Pico SDK includes
#include "pico/multicore.h"

// Std C++ includes
#include <iostream>

int main()
{
    stdio_init_all();
    sleep_ms(2000); // wait for USB serial to be ready

    std::cout << "Starting Pico Waveform Generator..." << std::endl;
    multicore_launch_core1(Ui::ui_task);

    Waveform::waveform_task();
}
