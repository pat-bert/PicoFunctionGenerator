#ifndef I2C_INTERFACE_HPP_INCLUDED
#define I2C_INTERFACE_HPP_INCLUDED

#include <cstdint>
#include <cstddef>

namespace I2C
{
    /// @brief  Interface for I2C communication.
    /// @details This interface defines the methods required for I2C communication.
    ///         It is designed to be used with a derived class that implements the actual
    ///         I2C communication logic. The CRTP (Curiously Recurring Template Pattern)
    ///         is used to allow the derived class to implement the methods without
    ///         needing to know the details of the base class.
    /// @tparam Derived The derived class that implements the I2C communication logic.
    template <typename Derived>
    class I2CInterface
    {
    public:
        /// @brief  Initializes the I2C interface.
        /// @details This method should be implemented by the derived class to perform
        ///         the actual initialization of the I2C interface.
        /// @return true if initialization was successful, false otherwise.
        bool init()
        {
            return static_cast<Derived *>(this)->initImpl();
        }

        /// @brief  Deinitializes the I2C interface.
        /// @details This method should be implemented by the derived class to perform
        ///         the actual deinitialization of the I2C interface.
        /// @return true if deinitialization was successful, false otherwise.
        bool deinit()
        {
            return static_cast<Derived *>(this)->deinitImpl();
        }

        /// @brief  Writes data to the specified I2C address.
        /// @details This method should be implemented by the derived class to perform
        ///         the actual write operation to the I2C bus.
        /// @param  addr   The I2C address to write to.
        /// @param  data   Pointer to the data to write.
        /// @param  length The number of bytes to write.
        /// @param  blocking If true, the method will block until the write is complete.
        /// @return The number of bytes written, or a negative error code on failure.
        int write(uint8_t addr, const uint8_t *data, size_t length, bool blocking = true)
        {
            return static_cast<Derived *>(this)->writeImpl(addr, data, length, blocking);
        }

        /// @brief  Reads data from the specified I2C address.
        /// @details This method should be implemented by the derived class to perform
        ///         the actual read operation from the I2C bus.
        /// @param  addr   The I2C address to read from.
        /// @param  data   Pointer to the buffer to store the read data.
        /// @param  length The number of bytes to read.
        /// @param  blocking If true, the method will block until the read is complete.
        /// @return The number of bytes read, or a negative error code on failure.
        int read(uint8_t addr, uint8_t *data, size_t length, bool blocking = true)
        {
            return static_cast<Derived *>(this)->readImpl(addr, data, length, blocking);
        }
    };
}

#endif