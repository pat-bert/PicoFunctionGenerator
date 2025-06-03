#ifndef I2C_INTERFACE_HPP_INCLUDED
#define I2C_INTERFACE_HPP_INCLUDED

namespace I2C
{
    template <typename Derived>
    class I2CInterface
    {
    public:
        int write(uint8_t addr, const uint8_t *data, size_t length)
        {
            return static_cast<Derived *>(this)->writeImpl(addr, data, length);
        }
    };
}

#endif