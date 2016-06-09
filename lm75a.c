/*
    ChibiOS - Copyright (C) 2016 Theodore Ateba

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
	
*/

#include "lm75a.h"

/**
* @fn		float lm75aReadTemperature(void)
* @brief	Read the LM75A temperature register and send back the temperature
* @return	the temperature measured by the LM75A sensor.
*/
float lm75aReadTemperature(void){
	uint16_t temp;
	uint16_t mask = 0x07FF; // For the 2 complement.
	uint8_t txbuf;
	uint8_t rxbuf[2];
	float temperature;

	txbuf = LM75A_T_REG;

	// Read from I2C BUS
	i2cAcquireBus(&I2CD1);
	i2cMasterTransmitTimeout(&I2CD1, LM75A_ADDR, &txbuf, 1, rxbuf, 2,
			MS2ST(10));
	i2cReleaseBus(&I2CD1);

	// Get all the 16bits read from the sensor
	temp = (rxbuf[0] << 8) + rxbuf[1];
	// Just keep the 11 MSB bits.
	temp = temp >> 5;
	
	// Positives values
	if(!(temp & (1 << 10)))
		temperature = temp * 0.125;
	// Negatives values
	else
		temperature = -((((~temp)&mask) + 1) * 0.125);
	
	return temperature;
}

/**
* @fn		flaot lm75aReadOvertemperature
* @brief	Read the Overtemperature shutdown threshold
* @return	the user define high limit temperature
*/
float lm75aReadSetpoint(uint8_t setpoint){
	uint16_t value;
	uint16_t mask = 0x01FF;
	uint8_t txbuf;
	uint8_t rxbuf[2];
	float temp;

	if(setpoint == LM75A_T_O)
		txbuf = LM75A_O_REG;
	else if(setpoint == LM75A_T_H)
		txbuf = LM75A_H_REG;
	else
		return 255.0;

	i2cAcquireBus(&I2CD1);
	i2cMasterTransmitTimeout(&I2CD1, LM75A_ADDR, &txbuf, 1, rxbuf, 2,
			MS2ST(4));
	i2cReleaseBus(&I2CD1);

	value = (rxbuf[0] << 8) + rxbuf[1];
	value = value >> 7; // Just keep the 9 MSB bits over the 16 bits.

	if(!(value &(1u < 8)))
		temp = value * 0.5;
	else
		temp = -((((~value)&mask) + 1) * 0.5);

	return temp;
}

/**
* @fn		uint8_t lm75aReadConfiguration(void)
* @brief	Read the LM75A senseor configuration register
*
* @return	config	the configuration read from the sensor
*/
uint8_t lm75aReadConfiguration(void){
	uint8_t config;
	uint8_t txbuf;

	txbuf = LM75A_C_REG;

	i2cAcquireBus(&I2CD1);
	i2cMasterTransmitTimeout(&I2CD1, LM75A_ADDR, &txbuf, 1, &config, 1,
			MS2ST(4));
	i2cReleaseBus(&I2CD1);

	return config;
}

/**
* @fn			void lm75aWriteConfiguration(uint8_t config)
* @brief		Write the LM75A senseor configuration
*
* @param[in]	config		the configuration to write on the sensor
*/
void lm75aWriteConfiguration(uint8_t config){
	uint8_t txbuf[2];

	txbuf[0] = LM75A_C_REG;
	txbuf[1] = config;

	i2cAcquireBus(&I2CD1);
	i2cMasterTransmitTimeout(&I2CD1, LM75A_ADDR, txbuf, 2, NULL, 0,
			MS2ST(4));
	i2cReleaseBus(&I2CD1);
}

/**
*
*/
/*void lm75aConfigSetpoint(float setpoint){}*/

/**
* @fn			void lm75aConfigOSOperationMode(uint8_t pinMode)
* @brief		Control the LM75A senseor power mode
*
* @param[in]	powerMode	the power mode to configure.
*//*
void lm75aConfigOSOperationMode(uint8_t pinMode){
	uint8_t config;
	uint8_t newConfig;

	config = lm75aReadConfiguration();

	if(pinMode == LM75A_OS_PIN_C)
		newConfig = config & ~(1 << 1);
	else if(pinMode == LM75A_OS_PIN_I)
		newConfig = config | (1 << 1);
	else
		return;

	lm75aWriteConfiguration(newConfig);
}
*/


/**
* @fn			void lm75aConfigOSFaultQueue(uint8_t queueValue)
* @brief		Set the fault queue value of the LM75A sensor OS pin
*
* @param[in]	queueValue	the numbre of fault to get before to react.
*/
void lm75aConfigOSFaultQueue(uint8_t queueValue){
	uint8_t config;
	uint8_t newConfig;

	config = lm75aReadConfiguration();

	if(queueValue == LM75A_OS_QV1){
		newConfig = config & ~(1 << 3);
		newConfig = newConfig & ~(1 << 4);
	}
	else if(queueValue == LM75A_OS_QV2){
		newConfig = config | (1 << 3);
		newConfig = newConfig & ~(1 << 4);
	}
	else if(queueValue == LM75A_OS_QV4){
		newConfig = config & ~(1 << 3);
		newConfig = newConfig | (1 << 4);
	}
	else if(queueValue == LM75A_OS_QV6){
		newConfig = config | (1 << 3);
		newConfig = newConfig | (1 << 4);
	}
	else
		return;

	lm75aWriteConfiguration(newConfig);
}

/**
* @fn			void lm75aConfigOSPolarity(uint8_t state)
* @brief		Set the LM75A senseor OS output pin active state to LOW/HIGH.
*
* @param[in]	state	the active state to configure for the OS output pin.
*/
void lm75aConfigOSPolarity(uint8_t state){
	uint8_t config;
	uint8_t newConfig;

	config = lm75aReadConfiguration();

	if(state == LM75A_OS_PIN_ACTIVE_L)
		newConfig = config & ~(1 << 2);
	else if(state == LM75A_OS_PIN_ACTIVE_H)
		newConfig = config | (1 << 2);
	else
		return;

	lm75aWriteConfiguration(newConfig);
}

/**
* @fn			void lm75aConfigOSOperationMode(uint8_t pinMode)
* @brief		Control the LM75A senseor power mode
*
* @param[in]	powerMode	the power mode to configure.
*/
void lm75aConfigOSOperationMode(uint8_t pinMode){
	uint8_t config;
	uint8_t newConfig;

	config = lm75aReadConfiguration();

	if(pinMode == LM75A_OS_PIN_C)
		newConfig = config & ~(1 << 1);
	else if(pinMode == LM75A_OS_PIN_I)
		newConfig = config | (1 << 1);
	else
		return;

	lm75aWriteConfiguration(newConfig);
}

/**
* @fn			void lm75aConfigPowerMode(uint8_t powerMode)
* @brief		Control the LM75A senseor power mode
*
* @param[in]	powerMode	the power mode to configure.
*/
void lm75aConfigPowerMode(uint8_t powerMode){
	uint8_t config;
	uint8_t newConfig;

	config = lm75aReadConfiguration();

	if(powerMode == LM75A_POWER_MODE_N)
		newConfig = config & ~(1 << 0);
	else if(powerMode == LM75A_POWER_MODE_S)
		newConfig = config | (1 << 0);
	else
		return;

	lm75aWriteConfiguration(newConfig);
}
