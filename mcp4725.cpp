#include "mcp4725.hpp"

/*!
	@brief Constructor for class MCP4725_PIC0
	@param refV The the reference voltage to be set in Volts.
*/
MCP4725::MCP4725(float refV)
{
	setReferenceVoltage(refV);
}

/*!
	@brief Init & config i2c
	@param addr I2C address 8 bit address 0x6?.
	@param i2c_type I2C instance of port, IC20 or I2C1.
	@param CLKspeed I2C Bus Clock speed in Kbit/s. see 7.1 datasheet
	@param SDApin I2C Data GPIO
	@param SCLKpin I2C Clock GPIO
	@return  true if success , false for failure
*/
bool MCP4725::init(I2C_Addr addr, i2c_inst_t *i2c_type, uint16_t CLKspeed, uint8_t SDApin, uint8_t SCLKpin, uint32_t I2C_timeout)
{

	// init I2c pins and interface
	m_addr = addr;
	m_i2c = i2c_type;
	m_scl_pin = SCLKpin;
	m_sda_pin = SDApin;
	m_clk_speed = CLKspeed;
	m_I2C_Delay = I2C_timeout;

	gpio_set_function(m_sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(m_scl_pin, GPIO_FUNC_I2C);
	gpio_pull_up(m_sda_pin);
	gpio_pull_up(m_scl_pin);
	i2c_init(m_i2c, m_clk_speed * 1000);
	busy_wait_ms(50);
	// check connection?
	return isConnected();
}

/*!
	@brief Switch off the  I2C interface and return I2C GPIO to default state
*/
void MCP4725::deinitI2C()
{
	gpio_set_function(m_sda_pin, GPIO_FUNC_NULL);
	gpio_set_function(m_scl_pin, GPIO_FUNC_NULL);
	i2c_deinit(m_i2c);
}

/*!
	@brief Checks if DAC is connected.
	@return true if DAC is connected , false if not
*/
bool MCP4725::isConnected()
{
	int ReturnCode = 0;
	uint8_t rxData = 0;
	// check connection?
	ReturnCode = i2c_read_timeout_us(m_i2c, static_cast<uint8_t>(m_addr), &rxData, 1, false, m_I2C_Delay);
	if (ReturnCode < 1)
	{ // no bytes read back from device or error issued
		return false;
	}
	return true;
}

/*!
	@brief Sets the reference voltage.
	@param voltage the reference voltage to be set, called from constructor.
*/
void MCP4725::setReferenceVoltage(float voltage)
{
	if (voltage == 0)
		m_refVoltage = reference_voltage;
	else
		m_refVoltage = voltage;

	m_bitsPerVolt = (float)steps / m_refVoltage;
}

/*!
	@brief Gets the reference voltage.
	@return The reference voltage in volts.
*/
float MCP4725::getReferenceVoltage() { return m_refVoltage; }

/*!
	@brief Set voltage out based on DAC input code.
	@param InputCode 0 to MCP4725_MAX_VALUE.
	@param mode MCP4725DAC mode, see enum CmdType.
	@param powerType MCP4725DAC power type, see enum MCP4725_PowerType_e
	@return  output of writeCommand method, true for success, false for failure.
*/
bool MCP4725::setInputCode(uint16_t InputCode, CmdType mode, PowerDownType powerType)
{
	if (m_safetyCheck == true)
	{
		if (InputCode > max_value)
			InputCode = max_value;
	}

	return writeCommand(InputCode, mode, powerType);
}

/*!
	@brief Set voltage out based on voltage input in volts.
	@param voltage  0 to_MCP4725_REFERENCE_VOLTAGE, voltage out
	@param mode MCP4725DAC mode, see enum CmdType.
	@param powerType MCP4725DAC power type, see enum MCP4725_PowerType_e
	@return  output of writeCommand method, true for success, false for failure.
*/
bool MCP4725::setVoltage(float voltage, CmdType mode, PowerDownType powerType)
{
	uint16_t voltageValue = 0;

	// Convert voltage to DAC bits
	// xx,xx,xx,xx,D11,D10,D9,D8 ,D7,D6,D4,D3,D2,D9,D1,D0
	if (m_safetyCheck == true)
	{
		if (voltage >= m_refVoltage)
			voltageValue = max_value;
		else if (voltage <= 0)
			voltageValue = 0; // make sure value never below zero
		else
			voltageValue = voltage * m_bitsPerVolt;
	}
	else if (m_safetyCheck == false)
	{
		voltageValue = voltage * m_bitsPerVolt;
	}

	return writeCommand(voltageValue, mode, powerType);
}

/*!
	@brief get current DAC InputCode from DAC register
	@return  DAC InputCode :or 0xFFFF if I2C error
*/
uint16_t MCP4725::getInputCode()
{
	uint16_t inputCode = readRegister(ReadType::DACReg);
	// InputCode = D11,D10,D9,D8,D7,D6,D5,D4, D3,D2,D1,D0,x,x,x,x

	if (inputCode != errorCode)
		return inputCode >> 4; // 0,0,0,0,D11,D10,D9,D8,  D7,D6,D5,D4,D3,D2,D1,D0
	else
		return inputCode; // i2c Error return 0xFFFF
}

/*!
	@brief  get DAC inputCode from DAC register & convert to volts
	@return DAC voltage or 0xFFFF if I2C error
*/
float MCP4725::getVoltage()
{
	float InputCode = getInputCode();
	if (InputCode != errorCode)
		return InputCode / m_bitsPerVolt;
	else
		return InputCode; // i2c Error return 0xFFFF
}

/*!
	@brief Read DAC inputCode from EEPROM
	@return  stored EEPROM inputcode value or 0xFFFF if I2C error
*/
uint16_t MCP4725::getStoredInputCode()
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
float MCP4725::getStoredVoltage()
{
	float InputCode = getStoredInputCode();

	if (InputCode != errorCode)
		return InputCode / m_bitsPerVolt;
	else
		return InputCode;
}

/*!
	@brief Get current power type from DAC register
	@return  power type or 0xFFFF if I2C error
	@note Power type corresponds to enum PowerDownType
*/
uint16_t MCP4725::getPowerType()
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
		return powerTypeValue;
	}
}

/*!
	@brief Get stored power type from EEPROM
	@return  EEPROM power type or 0xFFFF if I2C error
	@note Power type corresponds to enum PowerDownType
*/
uint16_t MCP4725::getStoredPowerType()
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

/*!
	@brief get EEPROM writing status from DAC register
	@return  1 for completed or 0( busy or I2C error)
	@note The BSY bit is low (during the EEPROM writing)
*/
bool MCP4725::getEEPROMBusyFlag()
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
bool MCP4725::writeCommand(uint16_t inputCode, CmdType mode, PowerDownType powerType)
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
		ReturnCode = i2c_write_timeout_us(m_i2c, static_cast<uint8_t>(m_addr), dataBuffer, 2, false, m_I2C_Delay);
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
		ReturnCode = i2c_write_timeout_us(m_i2c, static_cast<uint8_t>(m_addr), dataBuffer, 3, false, m_I2C_Delay);
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
	@brief Read DAC register
	@param mode MCP4725DAC datatype 1 3 or 5, see enum ReadType.
	@return  Requested value of read or type 0XFFFF if I2c error
*/
uint16_t MCP4725::readRegister(ReadType readType)
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
		ReturnCode = i2c_read_timeout_us(m_i2c, static_cast<uint8_t>(m_addr), dataBuffer, 1, false, m_I2C_Delay);
		dataWord = dataBuffer[0];
		break;

	case ReadType::DACReg: // Read 3 bytes  DAC register data, skip first 1 don't care
		ReturnCode = i2c_read_timeout_us(m_i2c, static_cast<uint8_t>(m_addr), dataBuffer, 3, false, m_I2C_Delay);
		dataWord = dataBuffer[1];
		dataWord = (dataWord << 8) | dataBuffer[2];
		break;

	case ReadType::EEPROM: // Read 5 bytes EEPROM data , first 3 don't care
		ReturnCode = i2c_read_timeout_us(m_i2c, static_cast<uint8_t>(m_addr), dataBuffer, 5, false, m_I2C_Delay);
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

/*!
	@brief  Setter for safety Check flag
	@param onOff Turns or or off the safety check  flag
*/
void MCP4725::setSafetyCheckFlag(bool onOff) { m_safetyCheck = onOff; }

/*!
	@brief Gets the safety Check flag value
	@return The safety Check flag value
*/
bool MCP4725::getSafetyCheckFlag() { return m_safetyCheck; }

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
bool MCP4725::GeneralCall(GeneralCallType typeCall)
{

	if (typeCall == GeneralCallType::Address)
	{
		return false;
	}

	int ReturnCode = 0;
	uint8_t dataBuffer[1];

	dataBuffer[0] = static_cast<uint8_t>(typeCall);
	// Note I2c address is MCP4725_GENERAL_CALL_ADDRESS
	ReturnCode = i2c_write_timeout_us(m_i2c, static_cast<uint8_t>(GeneralCallType::Address), dataBuffer, 1, false, m_I2C_Delay);

	if (ReturnCode < 1)
	{ // no bytes read back from device or error issued
		return false;
	}
	else
	{
		return true;
	}
}

/*!
	@brief Gets EEPROM write time
	@return mSec Memory write time, maximum 50 mSec
*/
uint16_t MCP4725::getEEPROMwriteTime() { return m_eepromWriteTime; }

/*!
	@brief  Sets the EEPROM write time
	@param WriteTime mSec Memory write time, maximum 50 mSec
*/
void MCP4725::setEEPROMwriteTime(uint16_t WriteTime)
{
	m_eepromWriteTime = WriteTime;
}
// ------------------ EOF ------------------------