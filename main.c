/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 *
 * @file    main.c
 *
 * @brief   Use the BMP085 library to mesure the temperature and the pressure
 *          with the Bosch Digital pressure sensor.
 *
 * @author  Theodore ATEBA
 *
 * @version 1.0
 *
 * @date    14  May 2016
 *
 */

//=============================================================================
// Include Files
//=============================================================================
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "lm75a.h"

//=============================================================================
// Global variables, I2C TX and RX buffers, I2C and Serial Configurations
//=============================================================================
float   temp;
float   overtemperature;
float   thysteresis;
uint8_t config;

BaseSequentialStream* chp = (BaseSequentialStream*) &SD2;

/**
 * I2C Configuration structure
 */
static const I2CConfig i2cConfig = {
  OPMODE_I2C,         /* I2C Operation mode   */
  400000,             /* I2C Clock speed      */
  FAST_DUTY_CYCLE_2,  /* I2C Duty cycle mode  */
};

//=============================================================================
// Functions
//=============================================================================

// Alife Thread, Blink the LED
static THD_WORKING_AREA(waLedGreenThread, 128);
static THD_FUNCTION(BlinkThread, arg){
  (void)arg;
  chRegSetThreadName("Led-Green-Binker");
  while(TRUE){
    palTogglePad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(1000);
  }
}


/**
 * @fn    int main(void)
 * @brief Application entry point.
 */
int main(void){
  bool delay = false;
  float hyst = -5.0;
  float over = -10.0;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  
  /* Start Serial driver */
  sdStart(&SD2, NULL);
	
	/* Configure the I2C Driver and i2C Pins */
  i2cStart(&I2CD1, &i2cConfig);
  palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN); // SCL
  palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN); // SDA
  
  /* Create the thread for the LED. */
  chThdCreateStatic(waLedGreenThread, sizeof(waLedGreenThread), LOWPRIO, BlinkThread, NULL);
  
  if(!lm75aConfigPowerMode(LM75A_PMN)){
    chprintf(chp, "\n\r LM75A Power Mode Configuration error!");
    delay = true;
  }
  
  if(!lm75aConfigOSOperationMode(LM75A_OS_IPM)){
    chprintf(chp, "\n\r LM75A OS Operation Mode configuration erro!");
    delay = true;
  }
  
  if(!lm75aConfigOSFaultQueue(LM75A_OS_FQV6)){
    chprintf(chp, "\n\r LM75A OS Fault queue programming configuration error!");
    delay = true;
  }
  
  if(!lm75aConfigOSPolarity(LM75A_OS_PAH)){
    chprintf(chp, "\n\r LM75A OS pin polarity Configuration error!");
    delay = true;
  }
  
  if(!lm75aWriteSetpoint(LM75A_T_H, hyst)){
    chprintf(chp, "\n\r LM75A Setpoint Configuration error!");
    delay = true;
    chprintf(chp, "\n\r return false");
	}
  else
    chprintf(chp, "\n\r return true");
  if(delay)
    chThdSleepMilliseconds(5000);
  
  while (true){
    temp = lm75aReadTemperature();
    overtemperature = lm75aReadSetpoint(LM75A_T_O);
    thysteresis = lm75aReadSetpoint(LM75A_T_H);
    config = lm75aReadConfiguration();
    
    if(!lm75aWriteSetpoint(LM75A_T_H, hyst)){
      chprintf(chp, "\n\r LM75A Hysteresis Configuration error!");
    }
    
    if(!lm75aWriteSetpoint(LM75A_T_O, over)){
      chprintf(chp, "\n\r LM75A Overtemperature Configuration error!");
    }
    
    //thysteresis = lm75aReadSetpoint(LM75A_T_H);
    
    chprintf(chp, "\n\r LM75A measurement:");
    chprintf(chp, "\n\r   Temperature: %.3f °c", temp);
    chprintf(chp, "\n\r   Overtemperature: %.3f °c", overtemperature);
    chprintf(chp, "\n\r   Thysteresis: %.3f °c", thysteresis);
    //chprintf(chp, "\n\r   Hyst config: %.3f °c", hyst);
    chprintf(chp, "\n\r   Configuration register: %x\n\r", config);
    
    hyst += 1;
    over += 2;
    
    if(over >= 10)
      over = -10;
    
    if(hyst >= 5.0)
      hyst = -5.0;
    
    chThdSleepMilliseconds(1000);
    chprintf(chp, "\033[2J\033[1;1H");
  }
  return 0;
}
