/* Name: Vihanga Bare
   MCIOT : Bonus Assignment- Circular Buffer
   Date: April 4. 2017
*/

/*	Built in functions from emblib library referred to from Silicon Labs Leopard Gecko Software Documentation
  	this code is originally Silicon Labs and copy righted by Silicon Labsï¿½ in 2015 and Silicon Labsï¿½ grants
 	permission to anyone to use the software for any purpose, including commercial applications, and to alter it,
 	and redistribute it freely subject that the origins is not misrepresented, altered source version must be
 	plainly marked, and this notice cannot be altered or removed from any source distribution.
 	********************************************************************************
 	* @section License
 	* <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 	*******************************************************************************
 	CMU_ClockSelectSet();
	CMU_ClockEnable();
	GPIO_PinModeSet();
	GPIO_PinOutSet();GPIO_PinOutClear();
	LETIMER_IntEnable();LETIMER_IntDisable();LETIMER_IntClear();
	CHIP_Init();
	LETIMER_Init();TIMER_Init();
	ACMP_Init(); ACMP_Enable();ACMP_Disable();
	EMU_EnterEMx();
	NVIC_EnableIRQ();
*/
#ifndef SRC_SETUPTASKS_H_
#define SRC_SETUPTASKS_H_

#include "main.h"


/*Global Variables*/

float correction_ratio=1;
uint8_t led_state=0;
volatile bool transferActive;			/* Transfer Flag */
float temp_increment = 0;					/* Temperature average */

/*loop check variables*/
int adc_start=0;
int dmacallback=0;
int adcsamplecount=0;
float desired_period, LETIMER_Prescalar, temp, prescaled_two_power;
int interrupt;

DMA_CB_TypeDef call_back;           /* DMA callback routine */

int i2c_tx_buffer[2];
int i2c_rx_buffer[1];
int lowthres,highthres;
int letimer_state=0;


/*  Function declarations */
void CMU_EM3Setup(void);
void CMU_EMSetup(void);
void Systemtick(void);
void self_callibration(void);
void prescalar(void);
void GPIO_setup(void);
void LED0_on(void);
void LED0_off(void);
void LED1_on(void);
void LED1_off(void);
void excite(void);
void clear(void);
void LETIMER_setup(void);
void LETIMER0_IRQHandler(void);
void setup_ADC(void);
void DMA_Setup(void);
void transferComplete(unsigned int channel, bool primary, void *user);
float convertToCelsius(int32_t adcSample);
void setup_ACMP(void);
void ADC0_IRQHandler(void);
void GPIO_ODD_IRQHandler(void);
void setup_I2C(void);
void I2C_write(void);
void I2C_read(void);
void initialise_TSL2651(void);
void disable_TSL2651(void);
void LEUART_setup(void);
void LEUART0_IRQHandler(void);
void SendToSamb11(void);
void reset_gesture_parameters();
void enable_gesture_sensor();
void read_gesture();

#endif //endif for src_setuptasks.h*/




