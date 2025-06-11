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
    /// Implementation of I2CInterface for Raspberry Pi Pico using hardware peripherals
    class I2CPicoHw : public I2CInterface<I2CPicoHw>
    {
    public:
        I2CPicoHw() = default;

        /// @brief Constructor that initializes the I2C port, baudrate, and GPIO pins for SDA and SCL
        /// @param port Pointer to the I2C instance (i2c0 or i2c1)
        /// @param baudrate Baudrate in kHz
        /// @param sda GPIO pin number for SDA
        /// @param scl GPIO pin number for SCL
        I2CPicoHw(i2c_inst_t *port, uint baudrate, uint sda, uint scl) : m_port(port), m_baudrate(baudrate), m_sda(sda), m_scl(scl) {}

        /// @brief Initialize the I2C interface
        /// @details This function sets the GPIO functions for SDA and SCL to I2C and initializes the I2C port with the specified baudrate
        /// @return true if initialization was successful, false otherwise
        bool initImpl()
        {
            gpio_set_function(m_sda, GPIO_FUNC_I2C);
            gpio_set_function(m_scl, GPIO_FUNC_I2C);
            i2c_init(m_port, m_baudrate * 1000);
            return true;
        }

        /// @brief Deinitialize the I2C interface
        /// @details This function sets the GPIO functions for SDA and SCL to NULL
        /// @return true if deinitialization was successful, false otherwise
        bool deinitImpl()
        {
            i2c_deinit(m_port);
            gpio_set_function(m_sda, GPIO_FUNC_NULL);
            gpio_set_function(m_scl, GPIO_FUNC_NULL);
            return true;
        }

        /// @brief Write data to the specified I2C address
        /// @details This function uses the i2c_write_blocking function to write data to the I2C bus.
        /// @param addr The I2C address to write to
        /// @param data Pointer to the data to write
        /// @param length The number of bytes to write
        /// @return The number of bytes written, or a negative error code on failure
        int writeImpl(uint8_t addr, const uint8_t *data, size_t length)
        {
            return i2c_write_blocking(m_port, addr, data, length, false);
        }

        /// @brief Read data from the specified I2C address
        /// @details This function uses the i2c_read_blocking function to read data from the I2C bus.
        /// @param addr The I2C address to read from
        /// @param data Pointer to the buffer to store the read data
        /// @param length The number of bytes to read
        /// @return The number of bytes read, or a negative error code on failure
        int readImpl(uint8_t addr, uint8_t *data, size_t length)
        {
            return i2c_read_blocking(m_port, addr, data, length, false);
        }

    private:
        i2c_inst_t *m_port; /// I2C port instance
        uint m_baudrate;    /// Baudrate in Hz
        uint m_sda;         /// SDA GPIO pin
        uint m_scl;         /// SCL GPIO pin
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
                return length; /// Return number of bytes written
            }

            return 0;
        }

        int readImpl(uint8_t addr, uint8_t *data, size_t length)
        {
            int errorCode = i2c_dma_read(m_i2cDma, addr, data, length);
            if (errorCode == 0)
            {
                return length; /// Return number of bytes read
            }

            return 0;
        }

    private:
        i2c_dma_t *m_i2cDma; /// I2C DMA instance
        i2c_inst_t *m_port;  /// I2C port instance
        uint m_baudrate;     /// Baudrate in Hz
        uint m_sda;          /// SDA GPIO pin
        uint m_scl;          /// SCL GPIO pin
    };

    class I2CPicoPIO : public I2CInterface<I2CPicoPIO>
    {
    public:
        /// SCL must be SDA + 1 (for wait mapping)
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
                return length; /// Return number of bytes written
            }
            return 0; /// Return error code
        }

        int readImpl(uint8_t addr, uint8_t *data, size_t length)
        {
            int result = pio_i2c_read_blocking(m_pio, m_sm, addr, data, length);
            if (result == 0)
            {
                return length; /// Return number of bytes read
            }
            return 0; /// Return error code
        }

    private:
        PIO m_pio;  /// PIO instance
        uint m_sm;  /// State machine number
        uint m_sda; /// SDA GPIO pin
        uint m_scl; /// SCL GPIO pin
        uint m_pioProgramOffset;
    };
}
#endif