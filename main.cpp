// Own includes
#include "ui/ui_task.hpp"
#include "waveform/waveform_task.hpp"

// Third-party includes
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"

// Std C++ includes
#include <iostream>

#define WAVEFORM_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define UI_TASK_STACK_SIZE 5000

#define WAVEFORM_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define UI_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)

int main()
{
    stdio_init_all();
    sleep_ms(2000); // wait for USB serial to be ready

    std::cout << "Starting Pico Waveform Generator..." << std::endl;

    MessageBufferHandle_t messageBufferHandle = xMessageBufferCreate(1024);
    if (messageBufferHandle == NULL)
    {
        std::cerr << "Failed to create message buffer!" << std::endl;
        return -1; // Exit or handle the error
    }

    TaskHandle_t waveFormTaskHandle = NULL;
    if (xTaskCreate(
            Waveform::run_task_wrapper,
            "WaveformThread",
            WAVEFORM_TASK_STACK_SIZE,
            static_cast<void *>(messageBufferHandle),
            WAVEFORM_TASK_PRIORITY,
            &waveFormTaskHandle) != pdPASS)
    {
        std::cerr << "Failed to create WaveformThread task!" << std::endl;
        return -1; // Exit or handle the error
    }

    vTaskCoreAffinitySet(waveFormTaskHandle, 1 << 0);

    TaskHandle_t uiTaskHandle = NULL;
    if (xTaskCreate(
            Ui::run_task_wrapper,
            "UiThread",
            UI_TASK_STACK_SIZE,
            static_cast<void *>(messageBufferHandle),
            UI_TASK_PRIORITY,
            &uiTaskHandle) != pdPASS)
    {
        std::cerr << "Failed to create UiThread task!" << std::endl;
        return -1; // Exit or handle the error
    }

    vTaskCoreAffinitySet(uiTaskHandle, 1 << 1);

    vTaskStartScheduler(); // Should never return, but in case it does, we can handle it gracefully.
}
