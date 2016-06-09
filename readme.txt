*****************************************************************************
** ChibiOS/HAL + ChibiOS/EX - I2C + BMP085 demo for STM32F4xxxx.		   **
*****************************************************************************

** TARGET **

The demo runs on an STM32F401RE Nucleo board.

** The Demo **

The demo flashes the board LED using the main loop, read data from BMP085
printing it on a BaseSequentialStream (SD2, mapped on USB virtual COM port).

** Build Procedure **

The demo has been tested by using the free Codesourcery GCC-based toolchain
and YAGARTO.
Just modify the TRGT line in the makefile in order to use different GCC ports.

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.st.com
