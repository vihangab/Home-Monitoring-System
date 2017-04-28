/*
 * Name: Vihanga Bare
   MCIOT : Bonus Assignment- Circular Buffer
   Date: April 4, 2017
 */

#include "main.h"


int main(void)
{
	length_buff = 6;
	cb_init(&cb,length_buff);
	CHIP_Init();													//* Chip errata

	temp_high = 35;
	temp_low = 15;
	channels_touched = 0;
	float sensitivity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0};
	Systemtick();
	GPIO_setup(); 													//* GPIO set up - Enable to port E, 2,3 from gpio, which is connected to LED0,
														//ADC setup function

	LETOUCH_Init(sensitivity);

	/* If any channels are touched while starting, the calibration will not be correct. */
	  /* Wait while channels are touched to be sure we continue in a calibrated state. */
	while(LETOUCH_GetChannelsTouched() != 0);
	//call_LESENSE_setup();
	  /*wifi code modules */
	setupLeuart();

	/* Setup DMA */
	setupDma();
#ifdef WIFI
	WIFI_Connect();
#endif

	setup_ADC();

	LETIMER_setup();											    //* LETIMER setup function
	#ifdef LEUART_ENABLE
	//LEUART_setup();

	#endif

	#ifdef ONBOARD_LIGHT_SENSOR
		setup_ACMP();													// GPIO setup for ACMP and light sense channels to be set to drive LED output
	#else
		setup_I2C();													//I2C setup function
	#endif

	set_sleep_mode(SLEEP_MODE_LIMIT);

	while(1)
	{
		sleep(); 													//*Calling the sleep routine which ensures that CPU does not go to sleep below the SLEEP_MODE_LIMIT set*by user*/
	}
}
