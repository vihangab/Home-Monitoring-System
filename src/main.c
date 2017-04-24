/*
 * Name: Vihanga Bare
   MCIOT : Bonus Assignment- Circular Buffer
   Date: April 4, 2017
 */

#include "main.h"


int main(void)
{
	CHIP_Init();													//* Chip errata

	Systemtick();
	GPIO_setup(); 													//* GPIO set up - Enable to port E, 2,3 from gpio, which is connected to LED0,
														//ADC setup function
	/*wifi code modules */
	setupLeuart();

	/* Setup DMA */
	setupDma();
	WIFI_Connect();
	setup_ADC();
	LETIMER_setup();											    //* LETIMER setup function
	#ifdef LEUART_ENABLE
	//LEUART_setup();

	#endif

	#ifdef ONBOARD_LIGHT_SENSOR
		setup_ACMP();													// GPIO setup for ACMP and light sense channels to be set to drive LED output
	#else
//		setup_I2C();													//I2C setup function
	#endif




	set_sleep_mode(SLEEP_MODE_LIMIT);

	while(1)
	{
		sleep(); 													//*Calling the sleep routine which ensures that CPU does not go to sleep below the SLEEP_MODE_LIMIT set*by user*/
	}
}
