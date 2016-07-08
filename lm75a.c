/**
 *
 * @file    lm75a.c
 *
 * @brief   Digital temperature driver
 *
 * @author  Theodore Ateba
 *
 * @date    26 June 2016
 *
 */

/*===========================================================================*/
/* Include Libraries                                                         */
/*===========================================================================*/
#include "lm75a.h"

/*===========================================================================*/
/* Driver Functions                                                          */ 
/*===========================================================================*/

/**
 * @fn      msg_t lm75aReadTemperature(I2CDriver *i2cp, float *tempp)
 * @brief   Read the LM75A temperature register.
 *
 * @param[in] i2cp  The pointer to the I2C driver interface.
 * @param[in] tempp The temperature measured by the LM75A sensor.
 * @return          The result of the reading operation.
 */
msg_t lm75aReadTemperature(I2CDriver *i2cp, float *tempp){
  uint16_t  temp;
  uint16_t  mask = 0x07FF;
  uint8_t   txbuf;
  uint8_t   rxbuf[2];
  msg_t     msg;
  
  txbuf = LM75A_T_REG;

  msg = i2cReadRegisters(i2cp, LM75A_ADDR, &txbuf, rxbuf, 2);

  if(msg == MSG_OK){
    temp = (rxbuf[0] << 8) + rxbuf[1];
    temp = temp >> 5;
    
    if(!(temp & (1 << 10)))
      *tempp =  (float)(temp * 0.125);
    else
      *tempp =  (float)(-((((~temp)&mask) + 1) * 0.125));
    return msg;
  }
  else
    return msg;
}

/**
 * @fn      msg_t lm75aReadSetpoint(I2CDriver *i2cp, uint8_t setpoint,
 *                    float *tempp)
 * @brief   Read the Overtemperature shutdown threshold.
 *
 * @param[in] i2cp      The pointer to the I2C driver interface.
 * @param[in] setpoint  The setpoint to read (hysteresis or overtemperature)
 * @param[in] tempp     The user define high limit temperature.
 * @return              The result of the reading operation.
 */
msg_t lm75aReadSetpoint(I2CDriver *i2cp, uint8_t setpoint, float *tempp){
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
    return MSG_SETPOINT;

  msg = i2cReadRegisters(i2cp, LM75A_ADDR, &txbuf, rxbuf, 2);

  if(msg == MSG_OK){
    value = (rxbuf[0] << 8) + rxbuf[1];
    value = value >> 7;
    
    if(!(value &(1 << 8)))
      *tempp =  (float)(value * 0.5);
    else
      *tempp = (float)(-((((~value)&mask) + 1) * 0.5));

    return msg;
  }
  else
    return msg;
}

/**
 * @fn    msg_t lm75aWriteSetpoint(I2CDriver *i2cp, uint8_t setpoint, float val)
 * @brief Write the configuration value to the corresponding setpoint.
 * @note  The setpoint to configure can be:
 *          - Overtemperature setpoint
 *          - Hysteresis setpoint
 *
 * @param[in] i2cp      The pointer to the I2C driver interface.
 * @param[in] setpoint  The setpoint to configure.
 * @param[in] val       The temperature value to set as the setpoint.
 * @return              The result of the writing opration to the setpoint.
 */
msg_t lm75aWriteSetpoint(I2CDriver *i2cp, uint8_t setpoint, float val){
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
    return MSG_SETPOINT;

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

  msg = i2cWriteRegisters(i2cp, LM75A_ADDR, txbuf, 3); 
  
  return msg;
}

/**
 * @fn    msg_t lm75aReadConfiguration(I2CDriver *i2cp, uint8_t *configp)
 * @brief Read the LM75A sensor configuration register
 *
 * @param[in] i2cp    The pointer to the I2C driver interface.
 * @param[in] config  The configuration read from the LM75A sensor.
 * @return    config  The result of the reading operation.
 */
msg_t	lm75aReadConfiguration(I2CDriver *i2cp, uint8_t *configp){
  uint8_t config;
  uint8_t txbuf;
  msg_t   msg;
  
  txbuf = LM75A_C_REG;
  
  msg = i2cReadRegisters(i2cp, LM75A_ADDR, &txbuf, &config, 1);

  if(msg == MSG_OK){
    *configp = config;
    return msg;
  }
  else
    return msg;
}

/**
 * @fn    smg_t lm75aWriteConfiguration(I2CDriver *i2cp, uint8_t config)
 * @brief Write the LM75A sensor configuration
 * @note  This function help to write diver configuration such as:
 *          - config[7:5] Reserved for manufacturer's used.
 *          - config[4:3] OS fault queue programming.
 *          - config[2]   OS polarity selection.
 *          - config[1]   OS operation mode selection.
 *          - config[0]   Device operation mode selection.
 *
 * @param[in] i2cp    The pointer to the I2C driver interface
 * @param[in] config  The configuration to write on the sensor.
 * @return    msg     The writing operation result.
 */
msg_t lm75aWriteConfiguration(I2CDriver *i2cp, uint8_t config){
  uint8_t txbuf[2];
  msg_t   msg;
  
  txbuf[0] = LM75A_C_REG;
  txbuf[1] = config;
  
 // i2cAcquireBus(i2cp);
  msg = i2cWriteRegisters(i2cp, LM75A_ADDR, txbuf, 2);
 // i2cReleaseBus(i2cp);

  return msg;
}

/**
 * @fn    msg_t lm75aConfigOSFaultQueue(uint8_t queueValue)
 * @brief Set the fault queue value of the LM75A sensor OS pin
 *
 * @param[in] i2cp        The pointer of the I2C driver interface.
 * @param[in] queueValue  the numbre of fault to get before to react.
 * @return                The result of the configuration operation.
 *
 */
msg_t lm75aConfigOSFaultQueue(I2CDriver *i2cp, uint8_t queueValue){
  uint8_t config;
  uint8_t newConfig;
  msg_t   msg;
  
  msg = lm75aReadConfiguration(i2cp, &config);
  
  if(msg != MSG_OK)
    return msg;

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
    return MSG_OSFAULTQUEUE;
  
  msg = lm75aWriteConfiguration(i2cp, newConfig);
  
  return msg;
}

/**
 * @fn    msg_t lm75aConfigOSPolarity(uint8_t state)
 * @brief Set the LM75A sensor OS output pin active state.
 * @note  The OS output pin can have one of the two state as active state:
 *          - LOW if state is egal to LM75A_OS_PAL.
 *          - HIGH if state is egal to LM75A_OS_PAH.
 *
 * @param[in] i2cp  The pointer of the I2C driver interface.
 * @param[in] state The active state to configure for the OS output pin.
 * @return    The result of the configuration operation
 */
msg_t lm75aConfigOSPolarity(I2CDriver *i2cp, uint8_t state){
  uint8_t config;
  uint8_t newConfig;
  msg_t   msg;
  
  msg = lm75aReadConfiguration(i2cp, &config);
  
  if(msg != MSG_OK)
    return msg;

  if(state == LM75A_OS_PAL)
    newConfig = config & ~(1 << 2);
  else if(state == LM75A_OS_PAH)
    newConfig = config | (1 << 2);
  else
    return MSG_OSPOLARITY;;
  
  msg = lm75aWriteConfiguration(i2cp, newConfig);
  
  return msg;
}

/**
 * @fn    msg_t lm75aConfigOSOperationMode(I2CDriver *i2cp, uint8_t pinMode)
 * @brief Control the LM75A sensor power mode.
 * @note  There are two operation mode for the OS output pin:
 *          - Comparator mode ==> pinMode == LM75A_OS_CPM.
 *          - Interrupt  mode ==> pinMode == LM75A_OS_IPM.
 *
 * @param[in] i2cp    The pointer of the I2C driver interface.
 * @param[in] pinMode	The mode in wich the pin must be configure.
 * @return            The result of the OS operation mode.
 */
msg_t lm75aConfigOSOperationMode(I2CDriver *i2cp, uint8_t pinMode){
  uint8_t config;
  uint8_t newConfig;
  msg_t   msg;
  
  msg = lm75aReadConfiguration(i2cp, &config);

  if(msg != MSG_OK)
    return msg;
  
  if(pinMode == LM75A_OS_CPM)
    newConfig = config & ~(1 << 1);
  else if(pinMode == LM75A_OS_IPM)
    newConfig = config | (1 << 1);
  else
    return MSG_OPERATIONMODE;
  
  msg = lm75aWriteConfiguration(i2cp, newConfig);
  
  return msg;
}

/**
 * @fn    msg_t lm75aConfigPowerMode(I2CDriver *i2cp, uint8_t powerMode)
 * @brief Control the LM75A senseor power mode
 *
 * @param[in] i2cp      The pointer of the I2C driver interface.
 * @param[in] powerMode The power mode to configure.
 * @return              The result of the power mode configuration.
 */
msg_t lm75aConfigPowerMode(I2CDriver *i2cp, uint8_t powerMode){
  uint8_t config;
  uint8_t newConfig;
  msg_t   msg;
  
  msg = lm75aReadConfiguration(i2cp, &config);
  
  if(msg != MSG_OK)
    return msg;

  if(powerMode == LM75A_PMN)
    newConfig = config & ~(1 << 0);
  else if(powerMode == LM75A_PMS)
    newConfig = config | (1 << 0);
  else
    return MSG_POWERMODE;
  
  msg = lm75aWriteConfiguration(i2cp, newConfig);
  
  return msg;
}
