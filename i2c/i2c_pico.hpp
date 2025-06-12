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
        bool initImpl();

        /// @brief Deinitialize the I2C interface
        /// @details This function sets the GPIO functions for SDA and SCL to NULL
        /// @return true if deinitialization was successful, false otherwise
        bool deinitImpl();

        /// @brief Write data to the specified I2C address
        /// @details This function is blocking but uses the full TX FIFO to write data to the I2C bus.
        /// If the requested transfer is targetting the same device as the last transfer,
        /// it will wait until the TX FIFO has enough space for the requested length.
        /// Otherwise, it will wait until the TX FIFO is empty before writing the data.
        /// @param addr The I2C address to write to
        /// @param data Pointer to the data to write
        /// @param length The number of bytes to write
        /// @param blocking If true, the function will block until the write is complete
        /// @return The number of bytes written, or a negative error code on failure
        int writeImpl(uint8_t addr, const uint8_t *data, size_t length, bool blocking);

        /// @brief Read data from the specified I2C address
        /// @details This function uses the i2c_read_blocking function to read data from the I2C bus.
        /// @param addr The I2C address to read from
        /// @param data Pointer to the buffer to store the read data
        /// @param length The number of bytes to read
        /// @param blocking If true, the function will block until the read is complete
        /// @return The number of bytes read, or a negative error code on failure
        int readImpl(uint8_t addr, uint8_t *data, size_t length, bool blocking);

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

        int writeImpl(uint8_t addr, const uint8_t *data, size_t length, bool blocking)
        {
            int result = i2c_dma_write(m_i2cDma, addr, data, length, blocking);
            return (result == 0) ? length : result;
        }

        int readImpl(uint8_t addr, uint8_t *data, size_t length, bool blocking)
        {
            int result = i2c_dma_read(m_i2cDma, addr, data, length);
            return (result == 0) ? length : result;
        }

    private:
        i2c_dma_t *m_i2cDma; /// I2C DMA instance
        i2c_inst_t *m_port;  /// I2C port instance
        uint m_baudrate;     /// Baudrate in Hz
        uint m_sda;          /// SDA GPIO pin
        uint m_scl;          /// SCL GPIO pin
    };

    /// Implementation of I2CInterface for Raspberry Pi Pico using PIO
    /// This class uses the PIO peripheral to implement I2C communication.
    /// It requires the PIO program to be loaded and initialized.
    class I2CPicoPIO : public I2CInterface<I2CPicoPIO>
    {
    public:
        /// @brief Constructor that initializes the PIO instance, state machine, and GPIO pins for SDA and SCL
        /// @param pio PIO instance (e.g., pio0 or pio1)
        /// @param sm State machine number (0 or 1)
        /// @param sda GPIO pin number for SDA
        /// @param scl GPIO pin number for SCL
        I2CPicoPIO(PIO pio, uint sm, uint sda, uint scl) : m_pio(pio), m_sm(sm), m_sda(sda), m_scl(scl)
        {
            assert(scl == sda + 1);
        }

        /// @brief Initialize the I2C interface using PIO
        /// @details This function adds the I2C program to the PIO instance and initializes it with the specified SDA and SCL pins.
        /// It also sets up the state machine for I2C communication.
        /// @return true if initialization was successful, false otherwise
        bool initImpl()
        {
            m_pioProgramOffset = pio_add_program(m_pio, &i2c_program);
            i2c_program_init(m_pio, m_sm, m_pioProgramOffset, m_sda, m_scl);
            return m_pioProgramOffset >= 0;
        }

        /// @brief Deinitialize the I2C interface using PIO
        /// @details This function removes the I2C program from the PIO instance and unclaims the state machine.
        /// @return true if deinitialization was successful, false otherwise
        bool deinitImpl()
        {
            pio_remove_program_and_unclaim_sm(&i2c_program, m_pio, m_sm, m_pioProgramOffset);
            return true;
        }

        /// @brief Write data to the specified I2C address using PIO
        /// @details This function uses the PIO I2C write blocking function to write data to the I2C bus.
        /// @param addr The I2C address to write to
        /// @param data Pointer to the data to write
        /// @param length The number of bytes to write
        /// @return The number of bytes written, or a negative error code on failure
        int writeImpl(uint8_t addr, const uint8_t *data, size_t length, bool blocking)
        {
            assert(blocking); // Ensure that blocking is true for this implementation
            int result = pio_i2c_write_blocking(m_pio, m_sm, addr, const_cast<uint8_t *>(data), length);
            return (result == 0) ? length : result;
        }

        /// @brief Read data from the specified I2C address using PIO
        /// @details This function uses the PIO I2C read blocking function to read data from the I2C bus.
        /// @param addr The I2C address to read from
        /// @param data Pointer to the buffer to store the read data
        /// @param length The number of bytes to read
        /// @return The number of bytes read, or a negative error code on failure
        int readImpl(uint8_t addr, uint8_t *data, size_t length, bool blocking)
        {
            assert(blocking); // Ensure that blocking is true for this implementation
            int result = pio_i2c_read_blocking(m_pio, m_sm, addr, data, length);
            return (result == 0) ? length : result;
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