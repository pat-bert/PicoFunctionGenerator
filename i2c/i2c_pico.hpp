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
        I2CPicoHw(i2c_inst_t *port, uint baudrate, uint sda, uint scl) : m_port(port), m_baudrate(baudrate), m_sda(sda), m_scl(scl) {}

        bool initImpl()
        {
            gpio_set_function(m_sda, GPIO_FUNC_I2C);
            gpio_set_function(m_scl, GPIO_FUNC_I2C);
            i2c_init(m_port, m_baudrate * 1000);
            busy_wait_ms(50);
            return true;
        }

        bool deinitImpl()
        {
            i2c_deinit(m_port);
            gpio_set_function(m_sda, GPIO_FUNC_NULL);
            gpio_set_function(m_scl, GPIO_FUNC_NULL);
            return true;
        }

        int writeImpl(uint8_t addr, const uint8_t *data, size_t length)
        {
            return i2c_write_blocking(m_port, addr, data, length, false);
        }

        int readImpl(uint8_t addr, uint8_t *data, size_t length)
        {
            return i2c_read_blocking(m_port, addr, data, length, false);
        }

    private:
        i2c_inst_t *m_port; // I2C port instance
        uint m_baudrate;    // Baudrate in Hz
        uint m_sda;         // SDA GPIO pin
        uint m_scl;         // SCL GPIO pin
    };

    class I2CPicoPIO : public I2CInterface<I2CPicoPIO>
    {
    public:
        I2CPicoPIO(PIO pio, uint sm, uint baudrate, uint sda, uint scl) : m_pio(pio), m_sm(sm), m_baudrate(baudrate), m_sda(sda), m_scl(scl) {}

        bool initImpl()
        {
            m_pioProgramOffset = pio_add_program(m_pio, &i2c_program);
            i2c_program_init(m_pio, m_sm, m_pioProgramOffset, m_sda, m_scl);
            return true;
        }

        bool deinitImpl()
        {
            pio_remove_program_and_unclaim_sm(&i2c_program, m_pio, m_sm, m_pioProgramOffset);
            return true;
        }

        int writeImpl(uint8_t addr, const uint8_t *data, size_t length)
        {
            return pio_i2c_write_blocking(m_pio, m_sm, addr, const_cast<uint8_t *>(data), length);
        }

        int readImpl(uint8_t addr, uint8_t *data, size_t length)
        {
            return pio_i2c_read_blocking(m_pio, m_sm, addr, data, length);
        }

    private:
        PIO m_pio;       // PIO instance
        uint m_sm;       // State machine number
        uint m_baudrate; // Baudrate in Hz
        uint m_sda;      // SDA GPIO pin
        uint m_scl;      // SCL GPIO pin
        uint m_pioProgramOffset;
    };
}
#endif