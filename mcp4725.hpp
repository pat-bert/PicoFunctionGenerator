#ifndef MCP4725_INCLUDED_HPP
#define MCP4725_INCLUDED_HPP

// Libraries
#include <stdio.h> // optional for printf debug error messages
#include "pico/stdlib.h"

namespace Dac
{

	template <typename I2CInterface>
	class MCP4725
	{
	public:
		// Section Enums

		/*! 8-bit i2c address. */
		enum class I2CAddr : uint8_t
		{
			VariantA0_PinA00 = 0x60, /**< MCP4725A0 with A0 = GND */
			VariantA0_PinA01 = 0x61, /**< MCP4725A0 with A0 = VCC */
			VariantA1_PinA00 = 0x62, /**< MCP4725A1 with A0 = GND */
			VariantA1_PinA01 = 0x63, /**< MCP4725A1 with A0 = VCC */
			VariantA2_PinA00 = 0x64, /**< MCP4725A2 with A0 = GND */
			VariantA2_PinA01 = 0x65	 /**< MCP4725A2 with A0 = VCC */
		};

		/*! DAC register, command bits C2C1C0 */
		enum class CmdType : uint8_t
		{
			FastMode = 0x00,	 /**< Writes data to DAC register */
			RegisterMode = 0x40, /**< Writes data & config bits to DAC register */
			EEPROM_Mode = 0x60	 /**< Writes data & config bits to DAC register & EEPROM */
		};

		/*! DAC register, power down bits PD1 PD0 , BSY,POR,xx,xx,xx,PD1,PD0,xx */
		enum class PowerMode : uint8_t
		{
			On = 0x00,			/**< Power down off draws 0.40mA no load & 0.29mA max load */
			Off_1kOhm = 0x01,	/**< Power down on, with 1.0 kOhm to GND, draws ~60nA */
			Off_100kOhm = 0x02, /**< Power down on, with 100 kOhm to GND */
			Off_500kOhm = 0x03	/**< Power down on, with 500 kOhm to GND */
		};

		/*! DAC library read register type  */
		enum class ReadType : uint8_t
		{
			Settings = 1, /**< Read 1 byte,  Settings data  */
			DACReg = 3,	  /**< Read 3 bytes, DAC register data */
			EEPROM = 5	  /**< Read 5 bytes, EEPROM data */
		};

		/*! DAC general call command datasheet 7.3 */
		enum class GeneralCallType : uint8_t
		{
			Address = 0x00, /**< General call address */
			Reset = 0x06,	/**< General call reset command */
			WakeUp = 0x09	/**< General call wake-up command */
		};

		/*!
			@brief Constructor for class MCP4725_PIC0
			@param refV The the reference voltage to be set in Volts.
		*/
		MCP4725() = default;

		MCP4725(I2CInterface *interface, I2CAddr addr, float refV = reference_voltage) : m_interface(interface), m_addr(addr)
		{
			setReferenceVoltage(refV);
		}

		/*!
			@brief Checks if DAC is connected.
			@return true if DAC is connected , false if not
		*/
		bool isConnected()
		{
			int ReturnCode = 0;
			uint8_t rxData = 0;
			// check connection?
			ReturnCode = m_interface->read(static_cast<uint8_t>(m_addr), &rxData, 1);
			if (ReturnCode < 1)
			{ // no bytes read back from device or error issued
				return false;
			}
			return true;
		}

		/*!
			@brief General Call, name from datasheet section 7.3
			@param typeCall Reset or wakeup see GeneralCallType.
			@return  True on success, false on I2c error OR wrong input(GeneralCallAddress)
			@note
				1. Reset MCP4725 & upload data from EEPROM to DAC register.
				Immediately after reset event, uploads contents of EEPROM into the DAC reg.
				2. Wake up & upload value from DAC register,
				Current power-down bits are set to normal, EEPROM power-down bit are not affected
		*/
		bool GeneralCall(GeneralCallType typeCall)
		{
			if (typeCall == GeneralCallType::Address)
			{
				return false;
			}

			int ReturnCode = 0;
			uint8_t dataBuffer[1];

			dataBuffer[0] = static_cast<uint8_t>(typeCall);
			// Note I2c address is MCP4725_GENERAL_CALL_ADDRESS
			ReturnCode = m_interface->write(static_cast<uint8_t>(GeneralCallType::Address), dataBuffer, 1);

			return ReturnCode >= 1;
		}

		/*!
			@brief Sets the reference voltage.
			@param voltage the reference voltage to be set, called from constructor.
		*/
		void setReferenceVoltage(float value)
		{
			if (value == 0)
			{
				m_refVoltage = reference_voltage;
			}
			else
			{
				m_refVoltage = value;
			}

			m_bitsPerVolt = (float)steps / m_refVoltage;
		}

		/*!
			@brief Gets the reference voltage.
			@return The reference voltage in volts.
		*/
		float getReferenceVoltage(void)
		{
			return m_refVoltage;
		}

		/*!
			@brief Set voltage out based on DAC input code.
			@param InputCode 0 to MCP4725_MAX_VALUE.
			@param mode MCP4725DAC mode, see enum CmdType.
			@param powerType MCP4725DAC power type, see enum MCP4725_PowerType_e
			@return  output of writeCommand method, true for success, false for failure.
		*/
		bool setInputCode(uint16_t inputCode, CmdType mode = CmdType::FastMode, PowerMode powerType = PowerMode::On)
		{
			if (m_safetyCheck == true)
			{
				if (inputCode > max_value)
				{
					inputCode = max_value;
				}
			}

			return writeCommand(inputCode, mode, powerType);
		}

		/*!
			@brief Get current DAC InputCode from DAC register
			@return  DAC InputCode :or 0xFFFF if I2C error
		*/
		uint16_t getInputCode(void)
		{
			uint16_t inputCode = readRegister(ReadType::DACReg);
			// InputCode = D11,D10,D9,D8,D7,D6,D5,D4, D3,D2,D1,D0,x,x,x,x

			if (inputCode != errorCode)
			{
				return inputCode >> 4; // 0,0,0,0,D11,D10,D9,D8,  D7,D6,D5,D4,D3,D2,D1,D0
			}
			else
			{
				return inputCode; // i2c Error return 0xFFFF
			}
		}

		/*!
			@brief Set voltage out based on voltage input in volts.
			@param voltage  0 to MCP4725_MAX_VOLTAGE, voltage out
			@param mode MCP4725DAC mode, see enum CmdType.
			@param powerType MCP4725DAC power type, see enum MCP4725_PowerType_e
			@return  output of writeCommand method, true for success, false for failure.
		*/
		bool setVoltage(float voltage, CmdType mode = CmdType::FastMode, PowerMode powerType = PowerMode::On)
		{
			uint16_t voltageValue = 0;

			// Convert voltage to DAC bits
			// xx,xx,xx,xx,D11,D10,D9,D8 ,D7,D6,D4,D3,D2,D9,D1,D0
			if (m_safetyCheck == true)
			{
				if (voltage >= m_refVoltage)
				{
					voltageValue = max_value;
				}
				else if (voltage <= 0)
				{
					voltageValue = 0; // make sure value never below zero
				}
				else
				{
					voltageValue = voltage * m_bitsPerVolt;
				}
			}
			else if (m_safetyCheck == false)
			{
				voltageValue = voltage * m_bitsPerVolt;
			}

			return writeCommand(voltageValue, mode, powerType);
		}

		/*!
			@brief Get current voltage out from DAC register
			@return  voltage out in volts or 0xFFFF if I2C error
		*/
		float getVoltage(void)
		{
			float inputCode = getInputCode();
			if (inputCode != errorCode)
			{
				return inputCode / m_bitsPerVolt; // convert to volts
			}
			else
			{
				return inputCode; // i2c Error return 0xFFFF
			}
		}

		/*!
			@brief Get current voltage out from DAC register
			@return  voltage out in volts or 0xFFFF if I2C error
		*/
		uint16_t getStoredInputCode(void)
		{
			uint16_t inputCode = readRegister(ReadType::EEPROM);
			// InputCode = x,PD1,PD0,x,D11,D10,D9,D8, D7,D6,D5,D4,D3,D2,D1,D0

			if (inputCode != errorCode)
				return inputCode & 0x0FFF; // 0,0,0,0,D11,D10,D9,D8, D7,D6,D5,D4,D3,D2,D1,D0
			else
				return inputCode; // i2c Error return 0xFFFF
		}

		/*!
			@brief Read stored DAC InputCode from EEPROM & convert to voltage
			@return  stored EEPROM voltage  or 0xFFFF if I2C error
		*/
		float getStoredVoltage(void)
		{
			float inputCode = getStoredInputCode();

			if (inputCode != errorCode)
			{
				return inputCode / m_bitsPerVolt; // convert to volts
			}
			else
			{
				return inputCode; // i2c Error return 0xFFFF
			}
		}

		/*!
			@brief Get current power type from DAC register
			@return  power type or 0xFFFF if I2C error
			@note Power type corresponds to enum PowerMode
		*/
		uint16_t getPowerType(void)
		{
			uint16_t powerTypeValue = readRegister(ReadType::Settings);
			// powerTypeValue = BSY,POR,xx,xx,xx,PD1,PD0,xx

			if (powerTypeValue != errorCode)
			{
				powerTypeValue &= 0x0006;	// 00,00,00,00,00,PD1,PD0,00
				return powerTypeValue >> 1; // 00,00,00,00,00,00,PD1,PD0
			}
			else
			{
				return powerTypeValue; // i2c Error return 0xFFFF
			}
		}

		/*!
			@brief Get stored power type from EEPROM
			@return  EEPROM power type or 0xFFFF if I2C error
			@note Power type corresponds to enum PowerMode
		*/
		uint16_t getStoredPowerType(void)
		{
			uint16_t powerTypeValue = readRegister(ReadType::EEPROM);
			// powerTypeValue = x,PD1,PD0,xx,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0

			if (powerTypeValue != errorCode)
			{
				powerTypeValue = powerTypeValue << 1; // PD1,PD0,xx,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0,00
				return powerTypeValue >> 14;		  // 00,00,00,00,00,00,00,00,00,00,00,00,00,00,PD1,PD0
			}
			else
			{
				return powerTypeValue;
			}
		}

		void setSafetyCheckFlag(bool onOff)
		{
			m_safetyCheck = onOff;
		}

		bool getSafetyCheckFlag(void)
		{
			return m_safetyCheck;
		}

		/*!
			@brief Gets EEPROM write time
			@return mSec Memory write time, maximum 50 mSec
		*/
		uint16_t getEEPROMwriteTime(void)
		{
			return m_eepromWriteTime;
		}

		/*!
			@brief  Sets the EEPROM write time
			@param WriteTime mSec Memory write time, maximum 50 mSec
		*/
		void setEEPROMwriteTime(uint16_t writeTime)
		{
			m_eepromWriteTime = writeTime;
		}

	protected:
		/*!
			@brief get EEPROM writing status from DAC register
			@return  1 for completed or 0( busy or I2C error)
			@note The BSY bit is low (during the EEPROM writing)
		*/
		bool getEEPROMBusyFlag(void)
		{
			uint16_t registerValue = readRegister(ReadType::Settings);
			// register value = BSY,POR,xx,xx,xx,PD1,PD0,xx
			bool ReturnValue = false;
			if (registerValue != errorCode)
			{
				ReturnValue = ((registerValue >> 7) & 0x01);
				return ReturnValue; // 1 - Not Busy, 0 - Busy
			}
			else
			{
				return ReturnValue; // I2C error
			}
		}

		/*!
			@brief Writes data to DAC register or EEPROM
			@param inputCode  0 to MCP4725_MAX_VALUE input code
			@param mode MCP4725DAC mode, see enum CmdType.
			@param PowerType MCP4725 power type, see enum MCP4725_PowerType_e
			@return   true for success, false for failure.
		*/
		bool writeCommand(uint16_t inputCode, CmdType mode, PowerMode powerType)
		{
			uint8_t dataBuffer[3];
			uint8_t lowByte = 0;
			uint8_t highByte = 0;
			int ReturnCode = 0;

			switch (mode)
			{
			case CmdType::FastMode:
				// C2=0,C1=0,PD1,PD0,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0
				lowByte = static_cast<uint8_t>(inputCode & 0x00FF);
				highByte = static_cast<uint8_t>((inputCode >> 8) & 0x00FF);
				dataBuffer[0] = static_cast<uint8_t>(mode) | (static_cast<uint8_t>(powerType) << 4) | highByte; // C2,C1,PD1,PD0,D11,D10,D9,D8
				dataBuffer[1] = lowByte;																		// D7,D6,D5,D4,D3,D2,D1,D0
				ReturnCode = m_interface->write(static_cast<uint8_t>(m_addr), dataBuffer, 2);
				if (ReturnCode < 1)
				{
					return false;
				}

				break;

			case CmdType::RegisterMode:
			// C2=0,C1=1,C0=0,x,x,PD1,PD0,x,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0,x,x,x,x
			case CmdType::EEPROM_Mode:
				// C2=0,C1=1,C0=1,x,x,PD1,PD0,x,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0,x,x,x,x
				inputCode = inputCode << 4; // D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0,x,x,x,x
				lowByte = static_cast<uint8_t>(inputCode & 0x00FF);
				highByte = static_cast<uint8_t>((inputCode >> 8) & 0x00FF);
				dataBuffer[0] = static_cast<uint8_t>(mode) | (static_cast<uint8_t>(powerType) << 1); // C2,C1,C0,x,x,PD1,PD0,x
				dataBuffer[1] = highByte;															 // D11,D10,D9,D8,D7,D6,D5,D4
				dataBuffer[2] = lowByte;															 // D3,D2,D1,D0,x,x,x,x
				ReturnCode = m_interface->write(static_cast<uint8_t>(m_addr), dataBuffer, 3);
				if (ReturnCode < 1)
				{
					return false;
				}
				break;
			}

			if (mode == CmdType::EEPROM_Mode)
			{
				if (getEEPROMBusyFlag() == true)
					return true;
				busy_wait_ms(m_eepromWriteTime); // typical EEPROM write time 25 mSec
				if (getEEPROMBusyFlag() == true)
					return true;
				busy_wait_ms(m_eepromWriteTime); // maximum EEPROM write time 25*2 mSec
			}

			return true;
		}

		/*!
			@brief Read register from MCP4725
			@param readType MCP4725DAC datatype 1 3 or 5, see enum ReadType.
			@return  Requested value of read or type 0XFFFF if I2c error
		*/
		uint16_t readRegister(ReadType readType)
		{
			uint16_t dataWord = static_cast<uint8_t>(readType);
			uint8_t dataBuffer[6];
			int ReturnCode = 0;

			/*Format of read data :
			== Settings data one byte
			BSY,POR,xx,xx,xx,PD1,PD0,xx,
			== DAC register data 3 byte(1st 1 don't care)
			D11,D10,D9,D8,D7,D6,D5,D4, D3,D2,D1,D0,xx,xx,xx,xx,
			== EEPROM data 5 byte (1st 3 don't care)
			xx,PD1,PD0,xx,D11,D10,D9,D8, D7,D6,D5,D4,D3,D2,D1,D0
			*/
			switch (readType)
			{
			case ReadType::Settings: // Read one byte settings
				ReturnCode = m_interface->read(static_cast<uint8_t>(m_addr), dataBuffer, 1);
				dataWord = dataBuffer[0];
				break;

			case ReadType::DACReg: // Read 3 bytes  DAC register data, skip first 1 don't care
				ReturnCode = m_interface->read(static_cast<uint8_t>(m_addr), dataBuffer, 3);
				dataWord = dataBuffer[1];
				dataWord = (dataWord << 8) | dataBuffer[2];
				break;

			case ReadType::EEPROM: // Read 5 bytes EEPROM data , first 3 don't care
				ReturnCode = m_interface->read(static_cast<uint8_t>(m_addr), dataBuffer, 5);
				dataWord = dataBuffer[3];
				dataWord = (dataWord << 8) | dataBuffer[4];
				break;
			}

			if (ReturnCode < 1)
			{ // no bytes read back from device or error issued
				return errorCode;
			}
			else
			{
				return dataWord;
			}
		}

	private:
		I2CInterface *m_interface;
		I2CAddr m_addr;

		float m_refVoltage;
		uint16_t m_bitsPerVolt;
		uint16_t m_eepromWriteTime = 25; /**<  mSec Memory write time, maximum 50 mSec */

		/**< Constants for MCP4725 DAC voltage levels  */
		constexpr static float reference_voltage = 3.3;	 /**<  default supply-reference Voltage in volts */
		constexpr static auto resolution = 12;			 /**< Resolution in bits, 12-bit */
		constexpr static auto steps = (1 << resolution); /**< Quantity of DAC steps: 2^12 = 4096 */
		constexpr static auto max_value = steps - 1;	 /**< Max value = 4096 - 1, range 0 to 4095 */

		bool m_safetyCheck = true;					  // Safety check for voltage level's , true  = on
		constexpr static uint16_t errorCode = 0xFFFF; /**<  returns this value if I2C bus error from some methods */
	};

}

#endif
