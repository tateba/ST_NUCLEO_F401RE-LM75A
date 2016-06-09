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
float lm75aReadOvertemperature(void){
	uint16_t setpoint;
	uint16_t mask = 0x01FF;
	uint8_t txbuf[2];
	uint8_t rxbuf[2];
	float overtemp;

	txbuf[0] = LM75A_O_REG;
	i2cAcquireBus(&I2CD1);
	i2cMasterTransmitTimeout(&I2CD1, LM75A_ADDR, txbuf, 1, rxbuf, 2,
			MS2ST(4));
	i2cReleaseBus(&I2CD1);

	setpoint = (rxbuf[0] << 8) + rxbuf[1];
	setpoint = setpoint >> 7; // Just keep the 9 MSB bits over the 16 bits.

	if(!(setpoint &(1u < 8)))
		overtemp = setpoint * 0.5;
	else
		overtemp = -((((~setpoint)&mask) + 1) * 0.5);

	return overtemp;
}

/**
*
*/
void lm75aSetOvertemperature(float tos){}

/**
*
*/
float lm75aReadHysteresis(void){
	return 75.0;
}

/**
*
*/
void lm75aSetHysteresis(float thyst){}

/**
*
*/
void lm75aSleep(void){}

/**
*
*/
void lm75aWake(void){}

/**
*
*/
void lm75aSetOsActiveState(uint8_t state){}
