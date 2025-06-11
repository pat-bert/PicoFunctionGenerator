/**
 * This Library was originally written by Olivier Van den Eede (4ilo) in 2016.
 * Some refactoring was done and SPI support was added by Aleksander Alekseev (afiskon) in 2018.
 *
 * https://github.com/afiskon/stm32-sh1106
 */

#ifndef SH1106_HPP_INCLUDED
#define SH1106_HPP_INCLUDED

#include <cstring>

namespace Lcd
{
    constexpr uint8_t MAX_WIDTH{132};
    constexpr uint8_t MAX_HEIGHT{64};

    template <uint8_t Width, uint8_t Height, typename I2CInterface>
    class SH1106
    {
    public:
        enum class I2CAddr : uint8_t
        {
            PRIMARY = 0x3C,
            SECONDARY = 0x3D
        };

        static_assert((Height > 0) && (Height <= MAX_HEIGHT));
        static_assert((Width > 0) && (Width <= MAX_WIDTH));

        SH1106(I2CInterface *interface, I2CAddr addr, uint8_t col_offset) : m_interface(interface),
                                                                            m_addr(addr),
                                                                            m_col_offset(col_offset),
                                                                            m_inverted(false),
                                                                            m_flipped(false),
                                                                            m_mirrored(false)
        {
            memset(m_page_buffer, 0, sizeof(m_page_buffer));
            memset(m_cmd_buffer, 0, sizeof(m_cmd_buffer));
        }

        static constexpr uint8_t get_width()
        {
            return Width;
        }

        static constexpr uint8_t get_height()
        {
            return Height;
        }

        static constexpr uint16_t get_buffer_size()
        {
            return (Width * Height) / 8; // 1 bit per pixel, 8 pixels per byte
        }

        // Initialize the oled screen
        bool init()
        {
            reset();      // Reset OLED
            sleep_ms(10); // Wait for the screen to boot

            // Init OLED
            display_on(false);    // display off
            chargepump(2);        // POR = 2, 8.0V
            power_on(true);       // Turn on the DC-DC converter
            com_pads(false);      // POR = false
            multiplex(64);        // POR = 64
            normal_display(true); // POR = true
            clk_div(1, 5);        // POR = 1, 5
            dis_pre_charge(2, 2); // POR is 2, 2
            vcomp(0x20);          // POR = 0x35
            set_page_addr(0);     // POR = 0, page 0
            col_start(0);         // POR = 0, column 0
            start_line(0);        // POR = 0, RAM Display line 0
            vert_offset(0x00);    // POR = 0, offset lines = 0
            contrast(0x80);       // POR = 128, Half Brightness
            flipped(m_flipped);
            reverse_cols(m_mirrored);
            inverted(m_inverted);
            clear_display();
            display_on(true); // turn on display

            return true;
        }

        void reset()
        {
            /* for I2C - do nothing */
        }

        /**
         * @brief   5. Set Contrast Control Register: (Double Bytes Command)
         *          This command is to set contrast setting of the display. The chip
         *          has 256 contrast steps from 00 to FF. The segment output current
         *          increases as the contrast step value increases. Segment output
         *          current setting: ISEG = α/256 X IREF X scale factor Where: α is
         *          contrast step; IREF is reference current equals 18.75µA; Scale
         *          factor = 16.
         * @param   uint8_t value (0-255), POR = 128
         */
        void contrast(uint8_t value)
        {
            write_2byte_cmd(0x81, value);
        }

        /**
         * @brief   10. Set DC-DC OFF/ON: (Double Bytes Command)
         *          This command is to control the DC-DC voltage converter. The
         *          converter will be turned on by issuing this command then display ON
         *          command. The panel display must be off while issuing this command.
         *          When D = “L”, DC-DC is disable.
         *          When D = “H”, DC-DC will be turned on when display on. (POR)
         * @param   bool On, POR = true
         */
        void power_on(bool on)
        {
            write_2byte_cmd(0xAD, on ? 0x8B : 0x8A);
        }

        /**
         * @brief   11. Display OFF/ON: (AEH - AFH)
         *          Alternatively turns the display on and off.
         *          When D = “L”, Display OFF OLED. (POR)
         *          When D = “H”, Display ON OLED.
         *          When the display OFF command is executed, power saver mode will be
         *          entered.
         *          Sleep mode:
         *              This mode stops every operation of the OLED display system, and
         *              can reduce current consumption nearly to a static current value
         *              if no access is made from the microprocessor. The internal
         *              status in the sleep mode is as follows:
         *              1) Stops the oscillator circuit and DC-DC circuit.
         *              2) Stops the OLED drive and outputs Hz as the segment/common
         *                 driver output.
         *              3) Holds the display data and operation mode provided before
         *                 the start of the sleep mode.
         *              4) The MPU can access to the built-in display RAM.
         * @param   bool On, POR = true
         */
        void display_on(bool on)
        {
            write_1byte_cmd(on ? 0xAF : 0xAE);
        }

        /**
         * @brief   6. Set Segment Re-map: (A0H - A1H)
         *          Change the relationship between RAM column address and segment
         *          driver. The order of segment driver output pads can be reversed by
         *          software. This allows flexible IC layout during OLED module
         *          assembly. For details, refer to the column address section of
         *          Figure. 8. When display data is written or read, the column address
         *          is incremented by 1 as shown in Figure. 1.
         *          When ADC = “L”, the right rotates (normal direction). (POR)
         *          When ADC = “H”, the left rotates (reverse direction).
         * @param   bool mirrored, POR = false
         */
        void reverse_cols(bool mirrored)
        {
            write_1byte_cmd(mirrored ? 0xA1 : 0xA0);
            m_mirrored = mirrored;
        }

        /**
         * @brief   8. Set Normal/Reverse Display: (A6H -A7H)
         *          Reverses the display ON/OFF status without rewriting the contents
         *          of the display data RAM.
         *          When D = “L”, the RAM data is high, being OLED ON potential
         *          (normal display). (POR)
         *          When D = “H”, the RAM data is low, being OLED ON potential (reverse display)
         * @param   bool inverted, POR = false
         */
        void inverted(bool invert)
        {
            write_1byte_cmd(invert ? 0xA6 : 0xA7); //--set inverse color
            m_inverted = invert;
        }

        /**
         * @brief   13. Set Common Output Scan Direction: (C0H - C8H)
         *          This command sets the scan direction of the common output allowing
         *          layout flexibility in OLED module design. In addition, the display
         *          will have immediate effect once this command is issued. That is,
         *          if this command is sent during normal display, the graphic display
         *          will be vertically flipped.
         *          When D = “L”, Scan from COM0 to COM [N -1]. (POR)
         *          When D = “H”, Scan from COM [N -1] to COM0.
         * @param   bool flipped, POR = false
         */
        void flipped(bool flipped)
        {
            write_1byte_cmd(flipped ? 0xC8 : 0xC0);
            m_flipped = flipped;
        }

        /**
         * @brief   Write the buffer to the display RAM
         * @param   uint8_t *buffer, pointer to the buffer, buffer must have a size
         *                           equal to (132 * 8) bytes.
         */
        int write_screen(uint8_t *buffer)
        {
            for (uint8_t page = 0; page < 8; page++)
            {
                set_page_addr(page);
                col_start(m_col_offset);
                write_data(buffer + Width * page, Width);
            }

            return 0;
        }

        int write_area(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, uint8_t *buffer)
        {
            uint8_t width = x2 - x1 + 1;
            uint8_t start_page = y1 >> 3;
            uint8_t end_page = y2 >> 3;

            int overall_return = 0;

            for (uint8_t page = start_page; page <= end_page; page++)
            {
                set_page_addr(page); // Set the page start address
                col_start(x1);       // Set the lower start column address
                overall_return = overall_return | write_data(buffer + x1, width);
                buffer += Width;
            }

            return overall_return;
        }

        // Fill the whole screen with the given color
        void clear_display()
        {
            memset(&m_page_buffer, 0, sizeof(m_page_buffer));
            m_page_buffer[0] = 0x40; // Continuation bit not set, Data bit set
            for (uint8_t page = 0; page < 8; page++)
            {
                set_page_addr(page);
                col_start(0);
                m_interface->write(static_cast<uint8_t>(m_addr), m_page_buffer, sizeof(m_page_buffer));
            }
        }

    protected:
        /**
         * @brief   1/2. Set Column Address (0x00-0x0F, 0x10-0x1F):
         *          Specifies column address of display RAM. Divide the column address
         *          into 4 higher bits and 4 lower bits. Set each of them into
         *          successions. When the microprocessor repeats to access to the
         *          display RAM, the column address counter is incremented during each
         *          access until address 131 is accessed. The page address is not
         *          changed during this time.
         * @param   uint_8t column, starting the column to map to the fisrt
         */
        void col_start(uint8_t column)
        {
            column += m_col_offset;
            column %= MAX_WIDTH;
            write_1byte_cmd(0x00 | (column & 0x0F));
            write_1byte_cmd(0x10 | (column & 0xF0) >> 4);
        }

        /**
         * @brief   3. Set Charge Pump voltage value: (30H~33H)
         *          Specifies output voltage (VPP) of the internal charger pump:
         * @param    uint8_t value, 0 = 6.4V, 1 = 7.4V, 2 = 8.0V (POR), 3 = 9.0V
         */
        void chargepump(uint8_t value)
        {
            value = value & 0x03; // Mask the value to the lower two bits
            write_1byte_cmd(0x30 | value);
        }

        /**
         * @brief   4. Set Display Start Line: (40H - 7FH)
         *          Specifies line address (refer to Figure. 8) to determine the
         *          initial display line or COM0. The RAM display data becomes the top
         *          line of OLED screen. It is followed by the higher number of lines
         *          in ascending order, corresponding to the duty cycle. When this
         *          command changes the line address, the smooth scrolling or page
         *          change takes place.
         * @param   uint8_t line
         */
        void start_line(uint8_t line)
        {
            if (line > 63)
                line = 63;
            write_1byte_cmd(0x40 | line);
        }

        /**
         * @brief   7. Set Entire Display OFF/ON: (A4H - A5H)
         *          Forcibly turns the entire display on regardless of the contents of
         *          the display data RAM. At this time, the contents of the display
         *          data RAM are held. This command has priority over the
         *          normal/reverse display command.
         *          normal = true, Display reflects the contents of RAM.
         *          normal = false, All display segments are ON.
         * @param   bool normal, POR = true
         */
        void normal_display(bool normal)
        {
            write_1byte_cmd(normal ? 0xA4 : 0xA5);
        }

        /**
         * @brief   9. Set Multiplex Ration: (Double Bytes Command)
         *          This command switches default 64 multiplex modes to any multiplex
         *          ratio from 1 to 64. The output pads COM0-COM63 will be switched to
         *          corresponding common signal.
         * @param   uint8_t ratio (1-64), POR = 64
         */
        void multiplex(uint8_t ratio)
        {
            ratio = -1;
            if (ratio > 63)
                ratio = 63;
            write_2byte_cmd(0xA8, ratio);
        }

        /**
         * @brief   12. Set Page Address: (B0H - B7H)
         *          Specifies page address to load display RAM data to page address
         *          register. Any RAM data bit can be accessed when its page address
         *          and column address are specified. The display remains unchanged
         *          even when the page address is changed.
         * @param   uint8_t page (0-7)
         */
        void set_page_addr(uint8_t page)
        {
            if (page > 7)
                page = 7;
            write_1byte_cmd(0xB0 | page);
        }

        /**
         * @brief   14. Set Display Offset: (0xD3, Double Bytes Command)
         *          This is a double byte command. The next command specifies the
         *          mapping of display start line to one of COM0-63 (it is assumed that
         *          COM0 is the display start line, that equals to 0). For example, to
         *          move the COM16 towards the COM0 direction for 16 lines, the 6-bit
         *          data in the second byte should be given by 010000. To move in the
         *          opposite direction by 16 lines, the 6-bit data should be given by
         *          (64-16), so the second byte should be 100000.
         * @param   uint8_t offset (0-63), POR = 0
         */
        void vert_offset(uint8_t offset)
        {
            if (offset > 63)
                offset = 63;
            write_2byte_cmd(0xD3, offset);
        }

        /**
         * @brief   15. Set Display Clock Divide Ratio/Oscillator Frequency: (0xD5,
         *              Double Bytes Command)
         *          This command is used to set the frequency of the internal display
         *          clocks (DCLKs). It is defined as the divide ratio (Value from 1 to
         *          16) used to divide the oscillator frequency. POR is 1. Frame
         *          frequency is determined by divide ratio, number of display clocks
         *          per row, MUX ratio and oscillator frequency.
         * @param   uint8_t divider (1-16), POR = 1
         * @param   uint8_t freq (0-15), 0 = -25%, 5 = 0% (POR), 15 = +50%
         */
        void clk_div(uint8_t divider, uint8_t freq)
        {
            divider = -1;
            if (divider > 15)
                divider = 0;
            if (freq > 15)
                freq = 5;
            freq = freq << 4;
            write_2byte_cmd(0xD5, freq | divider);
        }

        /**
         * @brief   16. Set Dis-charge/Pre-charge Period: (0xD9, Double Bytes Command)
         *          This command is used to set the duration of the pre-charge period.
         *          The interval is counted in number of DCLK. POR is 2 DCLKs.
         * @param   uint8_t discharge (1-16), POR = 2
         * @param   uint8_t precharge (1-15), POR = 2
         */
        void dis_pre_charge(uint8_t discharge, uint8_t precharge)
        {
            if (discharge = 0)
                discharge = 2;
            if (discharge > 15)
                discharge = 2;
            if (precharge = 0)
                precharge = 2;
            if (precharge > 15)
                precharge = 2;
            write_2byte_cmd(0xD9, discharge << 4 | precharge);
        }

        /**
         * @brief   17. Set Common pads hardware configuration: (0xDA, Double Bytes
         *              Command)
         *          This command is to set the common signals pad configuration
         *          (sequential or alternative) to match the OLED panel hardware
         *          layout.
         * @param   bool sequential, POR = false
         */
        void com_pads(bool sequential)
        {
            write_2byte_cmd(0xDA, sequential ? 0x02 : 0x12);
        }

        /**
         * @brief   18. Set VCOM Deselect Level: (0xDB, Double Bytes Command)
         *          This command is to set the common pad output voltage level at
         *          deselect stage.
         *          VCOM = β X VREF = (0.430 + A[7:0] X 0.006415) X VREF
         *          0x00 = 0.430
         *          0x35 = 0.770 (POR)
         *          0x3F = 0.834
         *          0x40-0xFF = 1
         * @param   uint8_t value, POR = 0x35
         */
        void vcomp(uint8_t level)
        {
            write_2byte_cmd(0xDB, level);
        }

        // Send a byte to the command register
        void write_1byte_cmd(uint8_t reg)
        {
            m_cmd_buffer[0] = 0x00;
            m_cmd_buffer[1] = reg;
            m_interface->write(static_cast<uint8_t>(m_addr), m_cmd_buffer, 2);
        }

        void write_2byte_cmd(uint8_t reg, uint8_t data)
        {
            // 0x00 is a byte indicating to sh1106 that a command is being sent
            m_cmd_buffer[0] = 0x80; // Continuation bit set, Data bit not set
            m_cmd_buffer[1] = reg;
            m_cmd_buffer[2] = 0x00; // Continuation bit not set, Data bit not set
            m_cmd_buffer[3] = data;
            m_interface->write(static_cast<uint8_t>(m_addr), m_cmd_buffer, 4);
        }

        // Send data
        int write_data(uint8_t *buffer, size_t size)
        {
            if (size > sizeof(m_page_buffer))
            {
                return -1;
            }

            m_page_buffer[0] = 0x40; // Continuation bit not set, Data bit set
            memcpy(&m_page_buffer[1], buffer, size);
            return m_interface->write(static_cast<uint8_t>(m_addr), m_page_buffer, size + 1);
        }

    private:
        I2CInterface *m_interface;
        I2CAddr m_addr;
        uint8_t m_col_offset;
        bool m_inverted;
        bool m_flipped;
        bool m_mirrored;
        uint8_t m_page_buffer[Width + 1];
        uint8_t m_cmd_buffer[4];
    };

    // Not implemented:
    /**
     * @brief   19. Read-Modify-Write: (E0H)
     *          A pair of Read-Modify-Write and End commands must always be used.
     *          Once read-modify-write is issued, column address is not incremental
     *          by read display data command but incremental by write display data
     *          command only. It continues until End command is issued. When the
     *          End is issued, column address returns to the address when
     *          read-modify-write is issued. This can reduce the microprocessor
     *          load when data of a specific display area is repeatedly changed
     *          during cursor blinking or others.
     */

    /**
     * @brief  20. End: (EEH)
     *         Cancels Read-Modify-Write mode and returns column address to the
     *         original address (when Read-Modify-Write is issued.)
     */

    /**
     * @brief   21. NOP: (E3H)
     *          Non-Operation Command.
     */

    template <typename I2CInterface>
    using SH1106_128x64 = SH1106<128, 64, I2CInterface>; // 128x64 pixel display
}

#endif
