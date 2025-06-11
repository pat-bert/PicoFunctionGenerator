#ifndef I2C_PICO_HPP_INCLUDED
#define I2C_PICO_HPP_INCLUDED

#include "i2c_interface.hpp"

#include "hardware/i2c.h"

extern "C"
{
#include "pio_i2c.h"
}

#include "i2c_dma.h"

namespace I2C
{
    class I2CPicoHw : public I2CInterface<I2CPicoHw>
    {
    public:
        I2CPicoHw() = default;
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

    class I2CPicoHwDma : public I2CInterface<I2CPicoHwDma>
    {
    public:
        I2CPicoHwDma() = default;
        I2CPicoHwDma(i2c_inst_t *port, uint baudrate, uint sda, uint scl) : m_i2cDma{nullptr}, m_port(port), m_baudrate(baudrate), m_sda(sda), m_scl(scl) {}

        bool initImpl()
        {
            return (0 == i2c_dma_init(&m_i2cDma, m_port, m_baudrate * 1000, m_sda, m_scl));
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
            int errorCode = i2c_dma_write(m_i2cDma, addr, data, length);
            if (errorCode == 0)
            {
                return length; // Return number of bytes written
            }

            return 0;
        }

        int readImpl(uint8_t addr, uint8_t *data, size_t length)
        {
            int errorCode = i2c_dma_read(m_i2cDma, addr, data, length);
            if (errorCode == 0)
            {
                return length; // Return number of bytes read
            }

            return 0;
        }

    private:
        i2c_dma_t *m_i2cDma; // I2C DMA instance
        i2c_inst_t *m_port;  // I2C port instance
        uint m_baudrate;     // Baudrate in Hz
        uint m_sda;          // SDA GPIO pin
        uint m_scl;          // SCL GPIO pin
    };

    class I2CPicoPIO : public I2CInterface<I2CPicoPIO>
    {
    public:
        // SCL must be SDA + 1 (for wait mapping)
        I2CPicoPIO(PIO pio, uint sm, uint sda, uint scl) : m_pio(pio), m_sm(sm), m_sda(sda), m_scl(scl)
        {
            assert(scl == sda + 1);
        }

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
            int result = pio_i2c_write_blocking(m_pio, m_sm, addr, const_cast<uint8_t *>(data), length);
            if (result == 0)
            {
                return length; // Return number of bytes written
            }
            return 0; // Return error code
        }

        int readImpl(uint8_t addr, uint8_t *data, size_t length)
        {
            int result = pio_i2c_read_blocking(m_pio, m_sm, addr, data, length);
            if (result == 0)
            {
                return length; // Return number of bytes read
            }
            return 0; // Return error code
        }

    private:
        PIO m_pio;  // PIO instance
        uint m_sm;  // State machine number
        uint m_sda; // SDA GPIO pin
        uint m_scl; // SCL GPIO pin
        uint m_pioProgramOffset;
    };
}
#endif