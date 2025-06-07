#ifndef I2C_INTERFACE_HPP_INCLUDED
#define I2C_INTERFACE_HPP_INCLUDED

#include <cstdint>
#include <cstddef>

namespace I2C
{
    template <typename Derived>
    class I2CInterface
    {
    public:
        bool init()
        {
            return static_cast<Derived *>(this)->initImpl();
        }

        bool deinit()
        {
            return static_cast<Derived *>(this)->deinitImpl();
        }

        int write(uint8_t addr, const uint8_t *data, size_t length)
        {
            return static_cast<Derived *>(this)->writeImpl(addr, data, length);
        }

        int read(uint8_t addr, uint8_t *data, size_t length)
        {
            return static_cast<Derived *>(this)->readImpl(addr, data, length);
        }
    };
}

#endif