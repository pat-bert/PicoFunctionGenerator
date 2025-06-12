#include "waveform_task.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <variant>

namespace Waveform
{
    void Task::init()
    {
        for (size_t i = 0; i < numberOfChannels; ++i)
        {
            // Initialize I2C drivers for each DAC
            m_i2cDrivers[i] = DacInterfaceDriverType{i2c_get_instance(i), i2cSpeedKHz, sdaDacs[i], sclDacs[i]};
            if (!m_i2cDrivers[i].init())
            {
                std::cout << "Failed to initialize I2C driver for DAC " << i << std::endl;
                return;
            }

            // Initialize DAC drivers, check connection and disable output
            m_dacs[i] = DacDriverType{&(m_i2cDrivers[i]), DacDriverType::I2CAddr::VariantA0_PinA00};
            if (!m_dacs[i].isConnected())
            {
                std::cout << "Failed to connect to DAC " << i << std::endl;
                return;
            }

            m_dacs[i].setInputCode(0, DacDriverType::CmdType::EEPROM_Mode, DacDriverType::PowerMode::Off_500kOhm);

            // Initialize PWM pins
            gpio_set_function(pwmPins[i], GPIO_FUNC_PWM);
            uint pwmSlice = pwm_gpio_to_slice_num(pwmPins[i]);

            // Setup visitor for each channel
            m_visitors[i] = WaveformVisitor{&m_dacs[i], pwmSlice};
        }
    }

    void Task::run()
    {
        init();

        std::array<ChannelData, numberOfChannels> waveFormDataArray{
            SawtoothData{true, 1000U, 4095U, true},
            TriangleData{true, 500U, 2048U}};

        while (true)
        {
            for (int i = 0; i < numberOfChannels; ++i)
            {
                std::visit(m_visitors[i], waveFormDataArray[i]);
            }
        }
    }

} // namespace Waveform