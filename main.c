/**
 *
 * @file    main.c
 *
 * @brief   Use the LM75A library to mesure the temperature of the Digital
 *          NXP temperature sensor.
 *
 * @author  Theodore Ateba
 *
 * @version 1.0
 *
 * @date    14  May 2016
 *
 */

/*==========================================================================*/
/* Includes Files                                                           */
/*==========================================================================*/
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "lm75a.h"

/*==========================================================================*/
/* Global variables, I2C and Serial Configurations                          */
/*==========================================================================*/
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

//static LM75ADriver LM75AD1;

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

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
  float hyst = -5.0;
  float over = -10.0;
  msg_t msg;

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
  palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(4) |
      PAL_STM32_OTYPE_OPENDRAIN); // SCL
  palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4) |
      PAL_STM32_OTYPE_OPENDRAIN); // SDA
  
  /* Create the thread for the LED. */
  chThdCreateStatic(waLedGreenThread, sizeof(waLedGreenThread),
      LOWPRIO, BlinkThread, NULL);
  
  chprintf(chp, "\n\r LM75A configurations started...");
  
  msg = lm75aConfigPowerMode(&I2CD1, LM75A_PMN);
  if(msg != MSG_OK)
    chprintf(chp, "\n\r   Power Mode Configuration error!");
  
  msg = lm75aConfigOSOperationMode(&I2CD1, LM75A_OS_IPM);
  if(msg != MSG_OK)
    chprintf(chp, "\n\r   OS Operation Mode configuration erro!");
  
  msg = lm75aConfigOSFaultQueue(&I2CD1, LM75A_OS_FQV6);
  if(msg != MSG_OK)
    chprintf(chp, "\n\r   OS Fault queue programming configuration error!");
  
  msg = lm75aConfigOSPolarity(&I2CD1, LM75A_OS_PAH);
  if(msg != MSG_OK)
    chprintf(chp, "\n\r   OS pin polarity Configuration error!");
  
  msg = lm75aWriteSetpoint(&I2CD1, LM75A_T_H, hyst);
  if(msg != MSG_OK)
    chprintf(chp, "\n\r   Setpoint Configuration error!");
  
  chprintf(chp, "\n\r LM75A configurations ended.");
  chThdSleepMilliseconds(5000);

  while (true){
    
    chprintf(chp, "\n\r LM75A measurements:");
    
    /* Try to read temperature from the LM75A */
    msg = lm75aReadTemperature(&I2CD1, &temp);
    if(msg == MSG_OK)
      chprintf(chp, "\n\r   Temperature: %.3f °c.", temp);
    else
      chprintf(chp, "\n\r   Temperature reading error: msg = %d.", msg);
    /* Try to read overtemperature setpoint from the LM75A */
    msg = lm75aReadSetpoint(&I2CD1, LM75A_T_O, &overtemperature);
    if(msg == MSG_OK)
      chprintf(chp, "\n\r   Overtemperature: %.3f °c.", overtemperature);
    else
      chprintf(chp, "\n\r   Overtemperature reading error: msg = %d.", msg);
    
    /* Try to read Hysteresis temperature from the LM75A */
    msg = lm75aReadSetpoint(&I2CD1, LM75A_T_H, &thysteresis);
    if(msg == MSG_OK)
      chprintf(chp, "\n\r   Thysteresis: %.3f °c.", thysteresis);
    else
      chprintf(chp, "\n\r   Hysteresys reading error: msg = %d.", msg);
    
    /* Try to read LM75A configuration */
    msg = lm75aReadConfiguration(&I2CD1, &config);
    if(msg == MSG_OK)
      chprintf(chp, "\n\r   Configuration register value: %x.\n\r", config);
    else
      chprintf(chp, "\n\r   Configuration reading error: msg = %d.", msg);

    /* Try to configure the Hysteresis setpoint */
    msg = lm75aWriteSetpoint(&I2CD1, LM75A_T_H, hyst);
    if(msg != MSG_OK)
      chprintf(chp, "\n\r   Hysteresis writting erro: msg = %d.", msg);
    
    /* Try to configure the overtemperature setpoint */
    msg = lm75aWriteSetpoint(&I2CD1, LM75A_T_O, over);
    if(msg != MSG_OK)
      chprintf(chp, "\n\r   Overtemperature writting error: msg = %d.", msg);
    
    /* Change the hysteresis and overtemperature setpoint for next time */  
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
