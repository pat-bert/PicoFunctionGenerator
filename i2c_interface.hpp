#ifndef I2C_INTERFACE_HPP_INCLUDED
#define I2C_INTERFACE_HPP_INCLUDED

namespace I2C
{
    class I2CInterface
    {
    public:
        virtual int write(uint8_t addr, const uint8_t *data, size_t length) = 0;
        virtual ~I2CInterface() = default;
    };
}

#endif