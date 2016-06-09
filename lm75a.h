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
 
#ifndef _LM75A_H_
#define _LM75A_H_

/*===========================================================================*/
/* Include files.					                                         */
/*===========================================================================*/
#include <hal.h>

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/
 
/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
* LM75A Operation mode:
*	- Normal: conversion every 100ms, this is the default mode
*	- Shutdown: Temperature measurement is desabled, sensor in sleep mode
*/
typedef enum{
	LM75A_POWER_MODE_N = 0x00,	// Periodicaly monitor the temperature
	LM75A_POWER_MODE_S = 0x01	// Minimize the power consomption
}lm75a_power_mode_e;

/**
* LM75A OS Output Pin mode:
*	- Comparator
*	- Interrupt
*/
typedef enum{
	LM75A_OS_PIN_C = 0x00,	// OS PIN in comparator mode
	LM75A_OS_PIN_I = 0x01	// OS PIN in interrupt mode
}lm75_os_pin_mode_e;

/**
* LM75A OS Output Pin active state:
*	- HIGH
*	- LOW
*/
typedef enum{
	LM75A_OS_PIN_ACTIVE_L = 0x00,	// OS pin is active low
	LM75A_OS_PIN_ACTIVE_H = 0x01	// OS pin is actve high
}lm75a_os_pin_active_state_e;

/**
* LM75A OS fault queue programming
*/
typedef enum{
	LM75A_OS_QV1 = 0x00,	// Queue value 1
	LM75A_OS_QV2 = 0x01,	// Queue value 2
	LM75A_OS_QV4 = 0x02,	// Queue value 4
	LM75A_OS_QV6 = 0x03		// Queue value 6
}lm75a_os_queue_value_e;

/*
* LM75A set-point enumeration
*/
typedef enum{
	LM75A_T_O = 0x00,	// Overtemperature set-point
	LM75A_T_H = 0x01	// Hysteresis set-point
}lm75a_setpoint_e;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

// LM75A Address
#define LM75A_ADDR		((uint8_t)0x48)	// Adresse of the sensor
// LM75A Register list
#define LM75A_C_REG		((uint8_t)0x01) // Configuration register
#define LM75A_T_REG		((uint8_t)0x00)	// Temperature register
#define LM75A_H_REG		((uint8_t)0x02)	// Hysteresis register
#define LM75A_O_REG		((uint8_t)0x03)	// Overtemperature shutdown register

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Functions.                   			                                 */
/*===========================================================================*/
float lm75aReadTemperature(void);
float lm75aReadSetpoint(uint8_t setpoint);
uint8_t lm75aReadConfiguration(void);
void lm75aWriteConfiguration(uint8_t config);
void lm75aConfigOSFaultQueue(uint8_t queueValue);
void lm75aConfigOSPolarity(uint8_t state);
void lm75aConfigOSOperationMode(uint8_t pinMode);
void lm75aConfigPowerMode(uint8_t powerMode);

#endif // _LM75A_H_
