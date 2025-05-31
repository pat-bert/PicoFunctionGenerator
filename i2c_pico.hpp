#ifndef I2C_PICO_HPP_INCLUDED
#define I2C_PICO_HPP_INCLUDED

#include "i2c_interface.hpp"

#include "hardware/i2c.h"

namespace I2C
{
    class I2CPico : public I2CInterface
    {
    public:
        I2CPico(i2c_inst_t *port) : m_port(port) {}

        int write(uint8_t addr, const uint8_t *data, size_t length) override
        {
            return i2c_write_blocking(m_port, addr, data, length, false);
        }

    private:
        i2c_inst_t *m_port; // I2C port instance
    };
}
#endif