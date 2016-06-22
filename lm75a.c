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
 * @fn      float lm75aReadTemperature(void)
 * @brief   Read the LM75A temperature register.
 *
 * @return  The temperature measured by the LM75A sensor.
 */
float lm75aReadTemperature(LM75ADriver *devp){
  uint16_t  temp;
  uint16_t  mask = 0x07FF;
  uint8_t   txbuf;
  uint8_t   rxbuf[2];
  msg_t     msg;
  
  txbuf = LM75A_T_REG;
  
  //i2cAcquireBus(&I2CD1);
  i2cAquireBus(i2cp);
  msg = i2cMasterTransmitTimeout(i2cp, LM75A_ADDR, &txbuf, 1, rxbuf, 2,
      MS2ST(4));
  //i2cReleaseBusi(&I2CD1);
  i2cReleaseBus(i2cp);
  
  if(msg == MSG_OK){
    temp = (rxbuf[0] << 8) + rxbuf[1];
    temp = temp >> 5;
    
    if(!(temp & (1 << 10)))
      return (float)(temp * 0.125);
    else
      return (float)(-((((~temp)&mask) + 1) * 0.125));
  }
  else
    return 255.0;
}

/**
 * @fn      float lm75aReadSetpoint(uint8_t setpoint)
 * @brief   Read the Overtemperature shutdown threshold.
 *
 * @return  The user define high limit temperature.
 */
float lm75aReadSetpoint(uint8_t setpoint){
  uint16_t  value;
  uint16_t  mask = 0x01FF;
  uint8_t   txbuf;
  uint8_t   rxbuf[2];
  msg_t     msg;
  
  if(setpoint == LM75A_T_O)
    txbuf = LM75A_O_REG;
  else if(setpoint == LM75A_T_H)
    txbuf = LM75A_H_REG;
  else
    return 255.0;
  
  i2cAcquireBus(&I2CD1);
  msg = i2cMasterTransmitTimeout(&I2CD1, LM75A_ADDR, &txbuf, 1, rxbuf, 2,
      MS2ST(4));
  i2cReleaseBus(&I2CD1);
  
  if(msg == MSG_OK){
    value = (rxbuf[0] << 8) + rxbuf[1];
    value = value >> 7;
    
    if(!(value &(1 << 8)))
      return (float)(value * 0.5);
    else
      return (float)(-((((~value)&mask) + 1) * 0.5));
  }
  else
    return 255.0;
}

/**
 * @fn    bool lm75aWriteSetpoint(uint8_t setpoint, float val)
 * @brief Write the configuration value to the corresponding setpoint.
 * @note  The setpoint to configure can be:
 *          - Overtemperature setpoint
 *          - Hysteresis setpoint
 *
 * @param[in] setpoint  The setpoint to configure.
 * @param[in] val       The temperature value to set as the setpoint.
 */
bool lm75aWriteSetpoint(uint8_t setpoint, float val){
  int32_t		it;
  int32_t   temp;
  uint16_t  ut;
  uint8_t   txbuf[3];
  msg_t     msg;
  
  if(setpoint == LM75A_T_H)
    txbuf[0] = LM75A_H_REG;
  else if(setpoint == LM75A_T_O)
    txbuf[0] = LM75A_O_REG;
  else
    return false; /* TODO: return a message number corresponding
                     to bad setpoint selection */
  temp = (int32_t)(val * 2);
  
  if(setpoint > 0.0){
    txbuf[1] = (uint8_t)((temp >> 0x1) & 0xFF);
    txbuf[2] = (uint8_t)((temp & 0x1) << 7);
  }
  else{
    it = ~temp + 1;
    ut = (uint8_t)((it & 0x1) << 7);
    ut = ((uint8_t)((it >> 0x1) & 0xFF)) << 0x1;
    ut = ~ut + 1;
    ut = ut >> 1;
    txbuf[1] = ut;
    txbuf[2] = ut >> 0x1;
  }
  i2cAcquireBus(&I2CD1);
  msg = i2cMasterTransmitTimeout(&I2CD1, LM75A_ADDR, txbuf, 3, NULL, 0,
      MS2ST(4));
  i2cReleaseBus(&I2CD1);
  if(msg == MSG_OK)
    return true;
  else
    return false;
}

/**
 * @fn      uint8_t lm75aReadConfiguration(void)
 * @brief   Read the LM75A sensor configuration register
 *
 * @return  config  The configuration read from the sensor
 */
uint8_t	lm75aReadConfiguration(void){
  uint8_t config;
  uint8_t txbuf;
  msg_t   msg;
  
  txbuf = LM75A_C_REG;
  
  i2cAcquireBus(&I2CD1);
  msg = i2cMasterTransmitTimeout(&I2CD1, LM75A_ADDR, &txbuf, 1, &config, 1,
      MS2ST(4));
  i2cReleaseBus(&I2CD1);
  
  if(msg == MSG_OK)
    return config;
  else
    return msg; // TODO: be sure that the value of msg error can't correspond
                // to a configuration value.
}

/**
 * @fn        smg_t lm75aWriteConfiguration(uint8_t config)
 * @brief     Write the LM75A sensor configuration
 * @note      This function help to write diver configuration such as:
 *              - config[7:5] Reserved for manufacturer's used.
 *              - config[4:3] OS fault queue programming.
 *              - config[2]   OS polarity selection.
 *              - config[1]   OS operation mode selection.
 *              - config[0]   Device operation mode selection.
 *
 * @param[in] config  the configuration to write on the sensor.
 * @return    msg     the writing operation result.
 */
msg_t lm75aWriteConfiguration(uint8_t config){
  uint8_t txbuf[2];
  uint8_t rxbuf[2];
  msg_t   msg;
  
  txbuf[0] = LM75A_C_REG;
  txbuf[1] = config;
  
  i2cAcquireBus(&I2CD1);
  msg = i2cMasterTransmitTimeout(&I2CD1, LM75A_ADDR, txbuf, 2, rxbuf, 0,
      MS2ST(4));
  i2cReleaseBus(&I2CD1);
  return msg;
}

/**
 * @fn        bool lm75aConfigOSFaultQueue(uint8_t queueValue)
 * @brief     Set the fault queue value of the LM75A sensor OS pin
 *
 * @param[in] queueValue  the numbre of fault to get before to react.
 * @return    The result of the configuration operation.
 *
 * @TODO      creer une enumeration lm75a_erro_e qui listera toutes les
 *            erreurs que l'on peut avoir lors de l'utilisation de ce
 *            capteur. Changer le type de retour pour renvoyer une
 *            erreur correspondante en fonction ce ce qui s'est produit.
 */
bool lm75aConfigOSFaultQueue(uint8_t queueValue){
  uint8_t config;
  uint8_t newConfig;
  msg_t   msg;
  
  config = lm75aReadConfiguration();
  
  if(queueValue == LM75A_OS_FQV1){
    newConfig = config & ~(1 << 3);
    newConfig = newConfig & ~(1 << 4);
  }
  else if(queueValue == LM75A_OS_FQV2){
    newConfig = config | (1 << 3);
    newConfig = newConfig & ~(1 << 4);
  }
  else if(queueValue == LM75A_OS_FQV4){
    newConfig = config & ~(1 << 3);
    newConfig = newConfig | (1 << 4);
  }
  else if(queueValue == LM75A_OS_FQV6){
    newConfig = config | (1 << 3);
    newConfig = newConfig | (1 << 4);
  }
  else
    return false; // TODO: return an enumeration for wrong queue_value
  
  msg = lm75aWriteConfiguration(newConfig);
  
  if(msg == MSG_OK)
    return true;
  else
    return false;
}

/**
 * @fn        bool lm75aConfigOSPolarity(uint8_t state)
 * @brief     Set the LM75A sensor OS output pin active state.
 * @note      The OS output pin can have one of the two state as active state:
 *              - LOW if state is egal to LM75A_OS_PAL.
 *              - HIGH if state is egal to LM75A_OS_PAH.
 *
 * @param[in] state The active state to configure for the OS output pin.
 * @return    The result of the configuration operation
 */
bool lm75aConfigOSPolarity(uint8_t state){
  uint8_t config;
  uint8_t newConfig;
  msg_t   msg;
  
  config = lm75aReadConfiguration();
  
  if(state == LM75A_OS_PAL)
    newConfig = config & ~(1 << 2);
  else if(state == LM75A_OS_PAH)
    newConfig = config | (1 << 2);
  else
    return false;
  
  msg = lm75aWriteConfiguration(newConfig);
  
  if(msg == MSG_OK)
    return true;
  else
    return false;
}

/**
 * @fn        bool lm75aConfigOSOperationMode(uint8_t pinMode)
 * @brief     Control the LM75A sensor power mode.
 * @note      There are two operation mode for the OS output pin:
 *              - Comparator mode ==> pinMode == LM75A_OS_CPM.
 *              - Interrupt  mode ==> pinMode == LM75A_OS_IPM.
 *
 * @param[in] pinMode	The mode in wich the pin must be configure.
 */
bool lm75aConfigOSOperationMode(uint8_t pinMode){
  uint8_t config;
  uint8_t newConfig;
  msg_t   msg;
  
  config = lm75aReadConfiguration();
  
  if(pinMode == LM75A_OS_CPM)
    newConfig = config & ~(1 << 1);
  else if(pinMode == LM75A_OS_IPM)
    newConfig = config | (1 << 1);
  else
    return false;
  
  msg = lm75aWriteConfiguration(newConfig);
  
  if(msg == MSG_OK)
    return true;
  else
    return false;
}

/**
 * @fn        bool lm75aConfigPowerMode(uint8_t powerMode)
 * @brief     Control the LM75A senseor power mode
 *
 * @param[in] powerMode The power mode to configure.
 */
bool lm75aConfigPowerMode(uint8_t powerMode){
  uint8_t config;
  uint8_t newConfig;
  msg_t   msg;
  
  config = lm75aReadConfiguration();
  
  if(powerMode == LM75A_PMN)
    newConfig = config & ~(1 << 0);
  else if(powerMode == LM75A_PMS)
    newConfig = config | (1 << 0);
  else
    return false;
  
  msg = lm75aWriteConfiguration(newConfig);
  
  if(msg == MSG_OK)
    return true;
  else
    return false;
}
