#include "i2c_pico.hpp"

#include "pico/assert.h"

// Addresses of the form 000 0xxx or 111 1xxx are reserved. No slave should
// have these addresses.
#define i2c_reserved_addr(addr) (((addr) & 0x78) == 0 || ((addr) & 0x78) == 0x78)

namespace I2C
{
    bool I2CPicoHw::initImpl()
    {
        gpio_set_function(m_sda, GPIO_FUNC_I2C);
        gpio_set_function(m_scl, GPIO_FUNC_I2C);
        gpio_set_slew_rate(m_sda, GPIO_SLEW_RATE_SLOW);
        gpio_set_slew_rate(m_scl, GPIO_SLEW_RATE_SLOW);
        gpio_set_input_hysteresis_enabled(m_sda, true);
        gpio_set_input_hysteresis_enabled(m_scl, true);
        i2c_init(m_port, m_baudrate * 1000);
        return true;
    }

    bool I2CPicoHw::deinitImpl()
    {
        i2c_deinit(m_port);
        gpio_set_function(m_sda, GPIO_FUNC_NULL);
        gpio_set_function(m_scl, GPIO_FUNC_NULL);
        return true;
    }

    int I2CPicoHw::writeImpl(uint8_t addr, const uint8_t *data, size_t length, bool blocking)
    {
        invalid_params_if(HARDWARE_I2C, addr >= 0x80); // 7-bit addresses
        invalid_params_if(HARDWARE_I2C, i2c_reserved_addr(addr));
        // Synopsys hw accepts start/stop flags alongside data items in the same
        // FIFO word, so no 0 byte transfers.
        invalid_params_if(HARDWARE_I2C, length == 0);
        invalid_params_if(HARDWARE_I2C, ((int)length) < 0);

        const bool isTargetDeviceSameAsQueued{addr == m_port->hw->tar};

        do
        {
            tight_loop_contents();
        } while (
            !(m_port->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS) && // Wait for TX FIFO to be empty
            !(isTargetDeviceSameAsQueued && (i2c_get_write_available(m_port) >= length))); // Wait for enough space in TX FIFO if the slave is the same

        m_port->hw->enable = 0;
        m_port->hw->tar = addr;
        m_port->hw->enable = 1;

        bool abort{false};

        uint32_t abort_reason{0U};
        int byte_ctr;

        const int ilen{static_cast<int>(length)};
        for (byte_ctr = 0; byte_ctr < ilen; ++byte_ctr)
        {
            const bool first{byte_ctr == 0};
            const bool last{byte_ctr == ilen - 1};

            m_port->hw->data_cmd =
                bool_to_bit(first && m_port->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
                bool_to_bit(last) << I2C_IC_DATA_CMD_STOP_LSB | *data++;

            abort_reason = m_port->hw->tx_abrt_source;
            if (abort_reason)
            {
                // Note clearing the abort flag also clears the reason, and
                // this instance of flag is clear-on-read! Note also the
                // IC_CLR_TX_ABRT register always reads as 0.
                m_port->hw->clr_tx_abrt;
                abort = true;
            }

            // Note the hardware issues a STOP automatically on an abort condition.
            // Note also the hardware clears RX FIFO as well as TX on abort,
            // because we set hwparam IC_AVOID_RX_FIFO_FLUSH_ON_TX_ABRT to 0.
            if (abort)
            {
                break;
            }
        }

        int rval{0};

        // A lot of things could have just happened due to the ingenious and
        // creative design of I2C. Try to figure things out.
        if (abort)
        {
            if (!abort_reason || abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS)
            {
                // No reported errors - seems to happen if there is nothing connected to the bus.
                // Address byte not acknowledged
                rval = PICO_ERROR_GENERIC;
            }
            else if (abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS)
            {
                // Address acknowledged, some data not acknowledged
                rval = byte_ctr;
            }
            else
            {
                // panic("Unknown abort from I2C instance @%08x: %08x\n", (uint32_t) i2c->hw, abort_reason);
                rval = PICO_ERROR_GENERIC;
            }
        }
        else
        {
            rval = byte_ctr;
        }

        // nostop means we are now at the end of a *message* but not the end of a *transfer*
        m_port->restart_on_next = false;
        return rval;
    }

    int I2CPicoHw::readImpl(uint8_t addr, uint8_t *data, size_t length, bool blocking)
    {
        assert(blocking); // Ensure that blocking is true for this implementation
        return i2c_read_blocking(m_port, addr, data, length, false);
    }
}