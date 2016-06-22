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
/* Include files.                                                            */
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
 * @brief LM75A Operation Mode:
 *  - Normal: conversion every 100ms, this is the default mode
 *  - Shutdown: Temperature measurement is desabled, sensor in sleep mode
 */
typedef enum{
  LM75A_PMN = 0x00, /**< Power Mode Normal                                  */
  LM75A_PMS = 0x01  /**< Power Mode Shutdown                                */
}lm75a_om_e;        /**< Device Operation Mode                              */

/**
 * @brief LM75A OS Output Pin Mode:
 *  - Comparator
 *  - Interrupt
 */
typedef enum{
  LM75A_OS_CPM = 0x00,  /**< OS Comparator Pin Mode                         */
  LM75A_OS_IPM = 0x01   /**< OS Interrupt Pin Mode                          */
}lm75a_os_opm_e;        /**< Device OS Output Pin Mode                      */

/**
 * @brief LM75A OS Pin Active State:
 *  - HIGH
 *  - LOW
 */
typedef enum{
  LM75A_OS_PAL = 0x00,  /**< OS Pin Active Low                              */
  LM75A_OS_PAH = 0x01   /**< OS Pin Actve High                              */
}lm75a_os_pas_e;        /**< Device OS Pin Active State                     */

/**
 * @brief LM75A OS fault queue value
 */
typedef enum{
  LM75A_OS_FQV1 = 0x00, /**< OS Fault Queue Value 1                         */
  LM75A_OS_FQV2 = 0x01, /**< OS Fault Queue Value 2                         */
  LM75A_OS_FQV4 = 0x02, /**< OS Fault Queue Value 4                         */
  LM75A_OS_FQV6 = 0x03  /**< OS Fault Queue Value 6                         */
}lm75a_os_fqv_e;        /**< Device OS Fault Queue Value                    */

/**
 * @brief LM75A Set Point
 */
typedef enum{
  LM75A_T_O = 0x00, /**< Overtemperature Set Point                          */
  LM75A_T_H = 0x01  /**< Hysteresis Set Point                               */
}lm75a_sp_e;        /**< Device Set Point                                   */

/**
 * @brief LM75A Thermometer subsystem configuration structure
 */
typedef struct{
  lm75a_om_e      om;     /**< Device Operation Mode: Normal/Shutdown       */
  lm75a_os_opm_e  os_opm; /**< OS Output Pin Mode: Comparator/Interrupt     */
  lm75a_os_pas_e  os_oas; /**< OS Output Active State: High/Low             */
  lm75a_os_fqv_e  os_fqv; /**< OS Fault Queue Value: 1/2/4/6                */
  float           tos;    /**< overtemperature set point: 80°c default      */
  float           thyst;  /**< Hysteresis set point                         */
}LM75ATherConfig;         /**< LM75A Thermometer Configuration              */

/**
 * @brief LM75A Driver State machine possible states.
 */
typedef enum{
  LM75A_UNINIT  = 0,  /**< Not initialized                                  */
  LM75A_STOP    = 1,  /**< Stopped                                          */
  LM75A_READY   = 2   /**< Ready                                            */
}lm75a_state_e;       /**< LM75A State machine enumeration type             */

/**
 * @brief LM75A configuration structure
 */
typedef struct{
  I2CDriver             *i2cp;    /**< LM75A I2C Interface                  */
  const I2CConfig       *i2ccfg;  /**< LM75A I2C Interface config           */
  const LM75ATherConfig thercfg;  /**< LM75A Thermometer config             */
}LM75AConfig;                     /**< LM75A Device Configuration           */

/**
 * @brief Structure representing a LM75A driver
 */
typedef struct LM75ADriver LM75ADriver;

/**
 * @brief LM75A driver.
 */
struct LM75ADriver{
  lm75a_state_e	state;
  LM75AConfig		config;
};


/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#define LM75A_ADDR  ((uint8_t)0x48) /**< LM75A Device I2C Address           */
#define LM75A_C_REG ((uint8_t)0x01) /**< LM75A Configuration register       */
#define LM75A_T_REG ((uint8_t)0x00) /**< LM75A Temperature register         */
#define LM75A_H_REG ((uint8_t)0x02) /**< LM75A Hysteresis register          */
#define LM75A_O_REG ((uint8_t)0x03) /**< LM75A Overtemp shutdown register   */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Functions.                                                                */
/*===========================================================================*/
float   lm75aReadTemperature (LM75ADriver *devp);
float   lm75aReadSetpoint (uint8_t setpoint);
bool    lm75aWriteSetpoint(uint8_t setpoint, float val);
uint8_t lm75aReadConfiguration (void);
msg_t   lm75aWriteConfiguration (uint8_t config);
bool    lm75aConfigOSFaultQueue (uint8_t queueValue);
bool    lm75aConfigOSPolarity (uint8_t state);
bool    lm75aConfigOSOperationMode (uint8_t pinMode);
bool    lm75aConfigPowerMode (uint8_t powerMode);

#endif // _LM75A_H_
