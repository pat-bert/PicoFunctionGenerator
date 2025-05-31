#ifndef MCP4725_INCLUDED_HPP
#define MCP4725_INCLUDED_HPP

#ifdef __cplusplus
extern "C"
{
#endif

// Libraries
#include <stdio.h> // optional for printf debug error messages
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus

/*!
	@brief Class for MCP4725 DAC
*/
class MCP4725
{
public:
	// Section Enums

	/*! 8-bit i2c address. */
	enum class I2C_Addr : uint8_t
	{
		MCP4725A0_Addr_A00 = 0x60, /**< MCP4725A0 with A0 = GND */
		MCP4725A0_Addr_A01 = 0x61, /**< MCP4725A0 with A0 = VCC */
		MCP4725A1_Addr_A00 = 0x62, /**< MCP4725A1 with A0 = GND */
		MCP4725A1_Addr_A01 = 0x63, /**< MCP4725A1 with A0 = VCC */
		MCP4725A2_Addr_A00 = 0x64, /**< MCP4725A2 with A0 = GND */
		MCP4725A2_Addr_A01 = 0x65  /**< MCP4725A2 with A0 = VCC */
	};

	/*! DAC register, command bits C2C1C0 */
	enum class CmdType : uint8_t
	{
		FastMode = 0x00,	 /**< Writes data to DAC register */
		RegisterMode = 0x40, /**< Writes data & config bits to DAC register */
		EEPROM_Mode = 0x60	 /**< Writes data & config bits to DAC register & EEPROM */
	};

	/*! DAC register, power down bits PD1 PD0 , BSY,POR,xx,xx,xx,PD1,PD0,xx */
	enum class PowerDownType : uint8_t
	{
		Off = 0x00,		   /**< Power down off draws 0.40mA no load & 0.29mA max load */
		On_1kOhm = 0x01,   /**< Power down on, with 1.0 kOhm to GND, draws ~60nA */
		On_100kOhm = 0x02, /**< Power down on, with 100 kOhm to GND */
		On_500kOhm = 0x03  /**< Power down on, with 500 kOhm to GND */
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

	MCP4725(float refV = reference_voltage);

	bool init(I2C_Addr addr, i2c_inst_t *type, uint16_t speed, uint8_t SDA, uint8_t SCLK, uint32_t I2C_Timeout);
	bool isConnected();
	bool GeneralCall(GeneralCallType);
	void deinitI2C();

	void setReferenceVoltage(float value);
	float getReferenceVoltage(void);

	bool setInputCode(uint16_t inputCode, CmdType = CmdType::FastMode, PowerDownType = PowerDownType::Off);
	uint16_t getInputCode(void);

	bool setVoltage(float voltage, CmdType = CmdType::FastMode, PowerDownType = PowerDownType::Off);
	float getVoltage(void);

	uint16_t getStoredInputCode(void);
	float getStoredVoltage(void);

	uint16_t getPowerType(void);
	uint16_t getStoredPowerType(void);

	void setSafetyCheckFlag(bool onOff);
	bool getSafetyCheckFlag(void);

	uint16_t getEEPROMwriteTime(void);
	void setEEPROMwriteTime(uint16_t);

private:
	// I2c related
	I2C_Addr m_addr;
	i2c_inst_t *m_i2c; // i2C port number, i2c0 or i2c1
	uint8_t m_sda_pin;
	uint8_t m_scl_pin;
	uint16_t m_clk_speed = 100;	 // I2C bus speed in khz
	uint32_t m_I2C_Delay = 50000; /**<  uS delay , I2C timeout */

	float m_refVoltage;
	uint16_t m_bitsPerVolt;
	uint16_t m_eepromWriteTime = 25; /**<  mSec Memory write time, maximum 50 mSec */

	/**< Constants for MCP4725 DAC voltage levels  */
	constexpr static float reference_voltage = 3.3;	 /**<  default supply-reference Voltage in volts */
	constexpr static auto resolution = 12;			 /**< Resolution in bits, 12-bit */
	constexpr static auto steps = (1 << resolution); /**< Quantity of DAC steps: 2^12 = 4096 */
	constexpr static auto max_value = steps - 1;	 /**< Max value = 4096 - 1, range 0 to 4095 */

	bool m_safetyCheck = true;						   // Safety check for voltage level's , true  = on
	constexpr static uint16_t errorCode = 0xFFFF; /**<  returns this value if I2C bus error from some methods */

	bool getEEPROMBusyFlag(void);
	bool writeCommand(uint16_t value, CmdType mode, PowerDownType powerType);
	uint16_t readRegister(ReadType dataType);
};

#endif // __cplusplus

#endif // library file header guard endif
