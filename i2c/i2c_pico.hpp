#ifndef I2C_PICO_HPP_INCLUDED
#define I2C_PICO_HPP_INCLUDED

#include "i2c_interface.hpp"

#include "hardware/i2c.h"
#include "pio_i2c.h"

namespace I2C
{
    class I2CPicoHw : public I2CInterface<I2CPicoHw>
    {
    public:
        I2CPicoHw(i2c_inst_t *port) : m_port(port) {}

        int writeImpl(uint8_t addr, const uint8_t *data, size_t length)
        {
            return i2c_write_blocking(m_port, addr, data, length, false);
        }

    private:
        i2c_inst_t *m_port; // I2C port instance
    };

    class I2CPicoPIO : public I2CInterface<I2CPicoPIO>
    {
    public:
        I2CPicoPIO(PIO pio, uint sm) : m_pio(pio), m_sm(sm) {}

        int writeImpl(uint8_t addr, const uint8_t *data, size_t length)
        {
            return pio_i2c_write_blocking(m_pio, m_sm, addr, const_cast<uint8_t *>(data), length);
        }

    private:
        PIO m_pio; // PIO instance
        uint m_sm; // State machine number
    };
}
#endif