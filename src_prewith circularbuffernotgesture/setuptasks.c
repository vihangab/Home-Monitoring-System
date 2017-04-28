/*
 * Name: Vihanga Bare
   MCIOT : Bonus Assignment- Circular Buffer
   Date: April 4, 2017
 */

/*	Built in functions from emblib library referred to from Silicon Labs Leopard Gecko Software Documentation
  	this code is originally Silicon Labs and copy righted by Silicon Labs� in 2015 and Silicon Labs� grants
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
#include "main.h"
#include "setuptasks.h"
#include "math.h"


/*ACMP configuration*/

/* ACMP configuration constant table - structure referred from http://devtools.silabs.com/dl/documentation/doxygen/5.0.0/efm32lg/html/group__ACMP.html*/
  static const ACMP_Init_TypeDef ACMP_Init_Light =
  {
    .fullBias = false,                 /* fullBias */
    .halfBias = true,                  /* halfBias */
    .biasProg =  0x0,                  /* biasProg */
    .interruptOnFallingEdge =  false,  /* interrupt on rising edge */
    .interruptOnRisingEdge =  false,   /* interrupt on falling edge */
    .warmTime = ACMP_WARMUP_TIME,       /* 256 cycle warmup to be safe */
    .hysteresisLevel = ACMP_HYSTERESIS, /* hysteresis level 5 */
    .inactiveValue = false,            /* inactive value */
    .lowPowerReferenceEnabled = true, /* low power reference */
    .vddLevel = VDD_Level_Light,        /* VDD level */
    .enable = false                    /* Don't request enabling. */
  };


  static const ACMP_Init_TypeDef ACMP_Init_Dark =
    {
      .fullBias = false,                 /* fullBias */
      .halfBias = true,                  /* halfBias */
      .biasProg =  0x0,                  /* biasProg */
      .interruptOnFallingEdge =  false,  /* interrupt on rising edge */
      .interruptOnRisingEdge =  false,   /* interrupt on falling edge */
      .warmTime = ACMP_WARMUP_TIME,       /* 256 cycle warmup to be safe */
      .hysteresisLevel = ACMP_HYSTERESIS, /* hysteresis level 5 */
      .inactiveValue = false,            /* inactive value */
      .lowPowerReferenceEnabled = true, /* low power reference */
      .vddLevel = VDD_Level_Dark,       			/* VDD level */
      .enable = false                    /* Don't request enabling. */
    };


  /* TIMER0 configuration constant table - structure referred from http://devtools.silabs.com/dl/documentation/doxygen/5.0.0/efm32lg/html/group__TIMER.html*/
  TIMER_Init_TypeDef timer0Init = {
  			.enable     = true,
  			.debugRun   = false,
  			.clkSel     = timerClkSelHFPerClk, 				// Select HFPER clock
  			.count2x    = false, 							// double count mode
  			.ati        = false,
  			.fallAction = timerInputActionNone, 			// no action
  			.riseAction = timerInputActionNone, 			// no action
  			.mode       = timerModeUp, // count up
  			.dmaClrAct  = false, 							// no DMA involved
  			.quadModeX4 = false, 							// no quadrature mode
  			.oneShot    = false, 							// one shot
  			.sync       = true,
  		};

  	TIMER_Init_TypeDef timer1Init = {
  				.enable     = true,
  				.debugRun   = false,
  				.clkSel     = timerClkSelCascade,
  				.count2x    = false, 						// double count mode
  				.ati        = false,
  				.fallAction = timerInputActionNone, 		// no action
  				.riseAction = timerInputActionNone, 		// no action
  				.mode       = timerModeUp, 					// count up
  				.dmaClrAct  = false, 						// no dma involved
  				.quadModeX4 = false, 						// no quadrature mode
  				.oneShot    = false, 						// keep running
  				.sync       = true,
  			};

  	/* LETIMER configuration constant table.
  	 * Set configurations for LETIMER 0 - Referred to from default configurations defined in
  	 * #define LETIMER_INIT_DEFAULT
  	 * in Silicon Labs Leopard Gecko Documentation
  	 */

  	const LETIMER_Init_TypeDef letimerInitOneShot =
  			  {
  			  .enable         = true,                   // Start counting when init completed.
  			  .debugRun       = false,                  // Counter shall not keep running during debug halt.
  			  .rtcComp0Enable = false,                  // Don't start counting on RTC COMP0 match.
  			  .rtcComp1Enable = false,                  // Don't start counting on RTC COMP1 match.
  			  .comp0Top       = true,                   // use COMP0 as top value
  			  .bufTop         = true,                  	// Don't load COMP1 into COMP0 when REP0 reaches 0.
  			  .out0Pol        = 0,                      // Idle value for output 0.
  			  .out1Pol        = 0,                      // Idle value for output 1.
  			  .ufoa0          = letimerUFOANone,        // No output on output 0
  			  .ufoa1          = letimerUFOANone,       	// No output on output 1
  			  .repMode        = letimerRepeatOneshot    // One shot
  			  };
  	const LETIMER_Init_TypeDef letimerInitFree =
  		  {
  		  .enable         = true,                   			/* Start counting when init completed. */
  		  .debugRun       = false,                  			/* Counter shall not keep running during debug halt. */
  		  .rtcComp0Enable = false,                  			/* Don't start counting on RTC COMP0 match. */
  		  .rtcComp1Enable = false,                  			/* Don't start counting on RTC COMP1 match. */
  		  .comp0Top       = true,                   			/* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
  		  .bufTop         = false,                  			/* Don't load COMP1 into COMP0 when REP0 reaches 0. */
  		  .out0Pol        = 0,                      			/* Idle value for output 0. */
  		  .out1Pol        = 0,                      			/* Idle value for output 1. */
  		  .ufoa0          = letimerUFOAPwm,         			/* PWM output on output 0 */
  		  .ufoa1          = letimerUFOANone,         			/* No output on output 1*/
  		  .repMode        = letimerRepeatFree       			/* Count until stopped */
  		  };



/* CMU_EMxSetup routine to select and enable clocks for EM0- EM2 and EM3 mode
 * This routine enables clocks for peripherals used i.e LETIMER0, GPIO
 * Have used ULFRCO clock for EM3 mode
 * SystemTick function - selects ULFRCO / LFXO as heartbeat
 * Input Variables - None
 * Global Variables - None
 * Return Variables - None
 * */
void CMU_EM3Setup(void)
{
	if(ENABLE_CALLIBRATION == 1)
	{
		self_callibration();
	}
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
}

void CMU_EMSetup(void)
 {
  		CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  		prescalar();
  		CMU_ClockEnable(cmuClock_CORELE, true);
  		CMU_ClockEnable(cmuClock_LETIMER0, true);
  		CMU_ClockEnable(cmuClock_GPIO, true);
 }

void Systemtick(void)
{
  	if(SLEEP_MODE_LIMIT > EM2)
  		{
  			CMU_EM3Setup();
  		}
  		else
  		{
  			CMU_EMSetup();
  		}
}
/* self_callibration routine to callibrate ULFRCO by taking LFXO and HFPER as references. Resulting correction_ratio
 * used for self callibrating ULFRCO at beginning of program
 * This routine enables clocks for peripherals used i.e LETIMER0, GPIO
 * Have used ULFRCO clock for EM3 mode
 * SystemTick function - selects ULFRCO / LFXO as heartbeat
 * Input Variables - None
 * Global Variables - correction_ratio
 * Return Variables - None
 * */
void self_callibration(void)
{
	float timervalue1=0, timervalue2=0;

	CMU_ClockEnable(cmuClock_HFPER, true);						//enable HFPER clock for TIMER0 TIMER1
	CMU_ClockEnable(cmuClock_TIMER0, true);						//enable TIMER0 clock
	CMU_ClockEnable(cmuClock_TIMER1, true);						//enable TIMER1 clock

	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);			//select and enable LFA clock - first ref
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);					//select and enable LETIMER clock

	/* Initialize count for timers to 0 */
	TIMER_CounterSet(TIMER0,0);
	TIMER_CounterSet(TIMER1,0);

	LETIMER_CompareSet(LETIMER0, 0, LFXO_MAX_COUNT);
	LETIMER_RepeatSet(LETIMER0, 0, 0x01);
	LETIMER_RepeatSet(LETIMER0, 1, 0x01);

	TIMER_Init(TIMER0, &timer0Init);							//initialise both TIMER and LETIMER to start count
	TIMER_Init(TIMER1, &timer1Init);
	LETIMER_Init(LETIMER0, &letimerInitOneShot);				//LETIMER to run as one shot

	while(LETIMER_CounterGet(LETIMER0) != 1);

	timervalue1=((float)TIMER_TopGet(TIMER0))+
				((float)TIMER_CounterGet(TIMER1)* (float)TIMERn_MAX_COUNT);

	TIMER_Reset(TIMER0);
	TIMER_Reset(TIMER1);
	LETIMER_Reset(LETIMER0);

	/* Disable and re enable all clocks to get appropriate value for second count, as suggested by professor*/
	CMU_ClockEnable(cmuClock_HFPER, false);
	CMU_ClockEnable(cmuClock_TIMER0, false);
	CMU_ClockEnable(cmuClock_TIMER1, false);
	CMU_ClockEnable(cmuClock_CORELE, false);
	CMU_ClockEnable(cmuClock_LETIMER0, false);


	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);


	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	TIMER_CounterSet(TIMER0,0);
	TIMER_CounterSet(TIMER1,0);

	LETIMER_CompareSet(LETIMER0, 0, ULFRCO_MAX_COUNT);
	LETIMER_RepeatSet(LETIMER0, 0, 0x01);
	LETIMER_RepeatSet(LETIMER0, 1, 0x01);

	TIMER_Init(TIMER0, &timer0Init);
	TIMER_Init(TIMER1, &timer1Init);
	LETIMER_Init(LETIMER0, &letimerInitOneShot);

	while(LETIMER_CounterGet(LETIMER0) != 1);

	timervalue2=((float)TIMER_TopGet(TIMER0))+
			((float)TIMER_CounterGet(TIMER1))*((float)TIMERn_MAX_COUNT);

	TIMER_Reset(TIMER0);
	TIMER_Reset(TIMER1);
	LETIMER_Reset(LETIMER0);

	/*Disable all clocks between different counting operations */

	CMU_ClockEnable(cmuClock_TIMER0, false);
	CMU_ClockEnable(cmuClock_TIMER1, false);
	CMU_ClockEnable(cmuClock_HFPER, false);

	CMU_ClockEnable(cmuClock_LETIMER0, false);
	CMU_ClockEnable(cmuClock_CORELE, false);


	correction_ratio=((float)timervalue1)/timervalue2;
}
/* prescalar() routine necessary for LFXO to count more than 2 secs. Depends on TIME_PERIOD, set by user
 * if TIME_PERIOD<2 no prescaling done
 * referred to from Prof Keith Graham's lecture slides and Silicon Labs example codes
 * Input Variables - None
 * Global Variables - correction_ratio
 * Return Variables - None
 * */


void prescalar(void)
{

	desired_period= (float)CMU_ClockFreqGet(cmuClock_LFA)* TIME_PERIOD * TIME_PERIOD;
	LETIMER_Prescalar=0;
	temp = (int)desired_period/1;
	prescaled_two_power=1;
	while(temp > LETIMER_MAX_COUNT)
	{
		LETIMER_Prescalar++;
		prescaled_two_power=prescaled_two_power * 2;
		temp=desired_period/prescaled_two_power;
	}
	CMU->LFAPRESC0|=(((int)LETIMER_Prescalar) << 8);
}

/*GPIO mode, and LED on and off, GPIO pin set and clear routines to make LED0 function (blink on or off)
 *LED0 connected to Port E,2 as inferred from EFM32 Low Energy Timer LETIMER application note - AN0026
 *Port d,6 connected to ambient light sensor pin
 *Port c,6 connected to ambient light excite pin
 *Input Variables - None
 *Global Variables - None
 *Return Variables - None
 */
void GPIO_setup(void)
{
	GPIO_PinModeSet(LED0_PORT,LED0_PIN, gpioModePushPull,0);
	GPIO_PinModeSet(LED1_PORT,LED1_PIN, gpioModePushPull,0);
	GPIO_PinModeSet(LIGHTSENSE_EXCITE_PORT,LIGHTSENSE_EXCITE_PIN, gpioModePushPull,0);
	GPIO_PinModeSet(LIGHTSENSE_SENSOR_PORT,LIGHTSENSE_SENSOR_PIN, gpioModeInput,0);
}


void LED0_on(void)
{
	GPIO_PinOutSet(LED0_PORT,LED0_PIN);
}
void LED0_off(void)
{
	GPIO_PinOutClear(LED0_PORT,LED0_PIN);
}

void LED1_on(void)
{
	GPIO_PinOutSet(LED1_PORT,LED1_PIN);
}
void LED1_off(void)
{
	GPIO_PinOutClear(LED1_PORT,LED1_PIN);
}
void excite(void)
{
	GPIO_PinOutSet(LIGHTSENSE_EXCITE_PORT,LIGHTSENSE_EXCITE_PIN);
}

void clear(void)
{
	GPIO_PinOutClear(LIGHTSENSE_EXCITE_PORT,LIGHTSENSE_EXCITE_PIN);
}

/* Routine to set up ACMP - order of events source taken from -
 * http://devtools.silabs.com/dl/documentation/doxygen/5.0.0/efm32lg/html/group__ACMP.html
 * This routine enables core clock for CPU and clocks for peripherals used i.e ACMP0 and GPIO
 * Input Variables - None
 * Global Variables - None
 * Return Variables - None
 */
void setup_ACMP(void)
{
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_ACMP0, true);
	/* Configure ACMP, Initialise it to dark state where reference vdd value is
	 * 2/63 which is low so low power consumption*/
	ACMP_Init(ACMP0, &ACMP_Init_Dark);
	/* Set up ACMP PosSel to VDD, negSel is controlled by Ambient light sensor. */

	ACMP_ChannelSet(ACMP0,ACMP_LIGHT_SENSE, ACMP_VDD);
	/* Disable ACMP output to GPIO pin */
	ACMP_GPIOSetup(ACMP0, 0, false, false);
}

/*************************************************************/
/* Routine to Enable and configure ADC                      */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void setup_ADC(void)
{
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_ADC0, true);
	/* Base the ADC configuration on the default setup. */

		const ADC_Init_TypeDef ADCInit	= {

		.ovsRateSel 	= ADC_OSR_RATE, /* if adcInitSingle.resolution=adcResOVS */
		.lpfMode 		= ADC_FILTERMODE,
		.warmUpMode 	= ADC_WARMUP,
		.timebase 		= ADC_TimebaseCalc(0),
		.prescale 		= ADC_PrescaleCalc(ADC_Freq, HFPER_Freq),
		.tailgate 		= false,
		};

		ADC_Init(ADC0, &ADCInit);
		ADC_InitSingle_TypeDef ADCInitSingle = {

		.prsSel 	= ADC_PRSTrigger,      	/* Triggered by PRS CH0 */
		.acqTime 	= ADC_AcqTime,
		.reference 	= ADC_VREF,		 	/* VDD, 1.25 V,  is set at Reference-Voltage*/
		.resolution = ADC_Resolution,   		/* no oversampling */
		.input 		= adcSingleInpTemp,    	/* input = channel 4 */
		.diff 		= false,
		.prsEnable 	= false,				/* Enable PSR-trigger for ADC */
		.leftAdjust = false,
		.rep 		= true,
		};

		#ifdef	DMA_off							//if DMA is off
			#ifndef ADC_Poll					//if enabling ADC polling

				/* Setup interrupt generation on completed conversion. */
				ADCInitSingle.rep = true;
				// Clearing the interrupts
				int intFlags;
				intFlags = ADC_IntGet(ADC0);
				ADC_IntClear(ADC0, intFlags);
				ADC_IntEnable(ADC0, ADC_IF_SINGLE);
				NVIC_EnableIRQ(ADC0_IRQn);

			#else
				/* Setup interrupt generation on completed conversion. */
				ADCInitSingle.rep = false;
			#endif
		#endif
		ADC_InitSingle(ADC0, &ADCInitSingle);
}

/*************************************************************/
/* Routine to Enable and configure LETIMER                   */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void LETIMER_setup(void)
{
	/* calculate the value to be set for LETIMER0 COMP0 register,
	 * LETIMER gives an interrupt when
	 * counter value (LETIMER0_CNT) matches COMP0 or COMP1 value
	 */
	//set_sleep_mode(SLEEP_MODE_LIMIT);

	float tp = (TIME_PERIOD * (float)CMU_ClockFreqGet(cmuClock_LFA) * correction_ratio);
	float led_on = (ON_TIME * (float)CMU_ClockFreqGet(cmuClock_LFA) * correction_ratio);

	if(prescaled_two_power > 0)
	{
		LETIMER_CompareSet(LETIMER0, 0, (int)(tp/prescaled_two_power));
		LETIMER_CompareSet(LETIMER0, 1, (int)(led_on/prescaled_two_power));
	}
	else
	{
		LETIMER_CompareSet(LETIMER0, 0, (int)tp);
		LETIMER_CompareSet(LETIMER0, 1, (int)led_on);
	}

	LETIMER_RepeatSet(LETIMER0, 0, 0x01);			//Set this so that, REP0 and REP1 are not set to 0, otherwise timer would stop*/
	LETIMER_RepeatSet(LETIMER0, 1, 0x01);

	/* Initialize LETIMER to run n free mode*/
	LETIMER_Init(LETIMER0, &letimerInitFree);


	LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF); 					//* Enable LETIMER0 to interrupt CPU when count register underflows
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP1);  				//* Enable LETIMER0 to interrupt CPU when count matches COMP1 compareset value
	NVIC_EnableIRQ(LETIMER0_IRQn); 									//* Enable LETIMER0 interrupt vector in the NVIC table
}

/* Interrupt handler, on COMP1, checks current LED state indicating light or dark, if lit calls the ACMP to measure for darkness
 * if dark, calls ACMP to measure for light. Turns LED on indicating dark, off indicating light
 */

/*************************************************************/
/* Routine to Enable and configure GPIO                      */
/* Input Variables - None									 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/
void DMA_Setup(void)
{

	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_DMA, true);

	DMA_Init_TypeDef DMA_init;

	/* initialize DMA descriptor block location */
	DMA_init.controlBlock = dmaControlBlock;
	DMA_init.hprot = 0;

	DMA_Init(&DMA_init);	/* initial DMA dma Control Block descriptor location */

	call_back.cbFunc  = transferComplete;
	call_back.userPtr = NULL;
	call_back.primary = true;

	/* Setting up channel */
	DMA_CfgChannel_TypeDef  chnlCfg;

	  chnlCfg.highPri   = false;	//channel priority is high
	  chnlCfg.enableInt = true;
	  chnlCfg.select    = DMAREQ_ADC0_SINGLE;
	  chnlCfg.cb        = &call_back;

	  DMA_CfgChannel(DMA_CHANNEL_ADC, &chnlCfg);

	  DMA_CfgDescr_TypeDef    descrCfg;

	  /* Setting up channel descriptor */

	  descrCfg.dstInc  = DMA_DST_INC;
	  descrCfg.srcInc  = DMA_SRC_INC;
	  descrCfg.size    = DMA_DATA_SIZE;
	  descrCfg.arbRate = DMA_ARBITRATE;
	  descrCfg.hprot   = 0;

	  DMA_CfgDescr(DMA_CHANNEL_ADC, true, &descrCfg);


	  /* Setting flag to indicate that transfer is in progress

	   * will be cleared by call-back function. */

	  transferActive = true;

	  /* Starting transfer. Using Basic since every transfer must be initiated

	   * by the ADC. */

	  DMA_ActivateBasic(DMA_CHANNEL_ADC,true,false,(void *)ramBufferAdcData,(void *)&(ADC0->SINGLEDATA),ADCSAMPLES - 1);
	  DMA->IFC = 1 << DMA_CHANNEL_ADC;
	  DMA->IEN = 1 << DMA_CHANNEL_ADC;
	  NVIC_EnableIRQ(DMA_IRQn);
}

/* ADC0 Call Back function to close off the DMA operation and evaluate results */
/*************************************************************/
/* Callback routine for DMA's ADC channel                    */
/* Input Variables - channel number, priority, user			 */
/* Global Variables - None									 */
/* Return Variables - None									 */
/*************************************************************/

void transferComplete(unsigned int channel, bool primary, void *user)
{
	int sum;
	dmacallback++;

	DMA->IFC = 1 << DMA_CHANNEL_ADC;
	ADC0->CMD = ADC_CMD_SINGLESTOP;
	//ADC_Reset(ADC0);
	unblock_sleep_mode(EM1);

	sum = 0;
	int i;
	for (i=0; i < ADCSAMPLES; i++)
	{
		sum += ramBufferAdcData[i];
	}
		//sum = sum / ADCSAMPLES;

		temp_increment = convertToCelsius((float)sum / ADCSAMPLES);
		if ((temp_increment < temp_low ) || (temp_increment > temp_high))
		{

			LED1_on();

		}
		else
		{

			LED1_off();
		}

  transferActive = false;
  //SendToSamb11();
}

/*************************************************************/
/* Routine to convert ADC sample value into Celsius          */
/* Input Variables - ADC sample value						 */
/* Global Variables - None		 							 */
/* Return Variables - Temperature sample in Celsius			 */
/* Sleep routine written based on example from 				 */
/* Code taken from application notes for ADC - Silicon labs  */
/*************************************************************/

float convertToCelsius(int32_t adcSample)
{
  float temp;
  /* Factory calibration temperature from device information page. */
  float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
                             >> _DEVINFO_CAL_TEMP_SHIFT);

  float cal_value_0 = (float)((DEVINFO->ADC0CAL2
                               & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
                              >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

  /* Temperature gradient (from datasheet) */
  float t_grad = -6.27;

  temp = (cal_temp_0 * 1.0) - ((cal_value_0 - adcSample)  / t_grad);

  return temp;
}
/*************************************************************/
/* LETIMER interrupt handler - clear UF and COMP1 interrupts
 * when they arrive, compile time conditions for DMA on/off,
 * ACMP on/off placed here, logic for low power management
 * three periods, ACMP, DMA, ADC, IDC, power on and shut off
 * sequence defined                      					 */
/* Input Variables - None									 */
/* Global Variables - loop check variables - letimer_state,
 * led_state, x1, x2,									     */
/* Return Variables - None									 */
/*************************************************************/

void LETIMER0_IRQHandler(void)
{
    //Clear LETIMER0 COMP0 flag
	INT_Disable();

	//cb_free(cb, c_buffer);
	if(LETIMER_IntGetEnabled(LETIMER0) == LETIMER_IF_UF)
	{
		//ACMP
		LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
		//call_LESENSE_setup();
		//enable_gesture_sensor();
		#ifdef	ONBOARD_LIGHT_SENSOR

			if((ACMP0->STATUS & ACMP_STATUS_ACMPOUT))
			{
				if(led_state == 1)
				{
					LED0_off();

					ACMP_Disable(ACMP0);
					ACMP_Init(ACMP0, &ACMP_Init_Dark);
					ACMP_GPIOSetup(ACMP0, 0, false, false);
					// Set up ACMP negSel to VDD, posSel is controlled by LESENSE.
					ACMP_ChannelSet(ACMP0, ACMP_LIGHT_SENSE, ACMP_VDD);
					led_state = 0;
				}
				else
				{
					LED0_on();

					ACMP_Disable(ACMP0);
					ACMP_Init(ACMP0, &ACMP_Init_Light);
					ACMP_GPIOSetup(ACMP0, 0, false, false);
					// Set up ACMP negSel to VDD, posSel is controlled by LESENSE.
					ACMP_ChannelSet(ACMP0, ACMP_VDD,ACMP_LIGHT_SENSE);
					led_state = 1;
				}
			}
			clear();
		#endif
		//ADC
		#ifdef	DMA_off

			#ifdef ADC_Poll

				adcsamplecount=0;
				set_sleep_mode(ADC_EM);
				while(adcsamplecount < ADCSAMPLES)
				{
					ADC_Start(ADC0, adcStartSingle);
					adc_start++;
					while(ADC0->STATUS & ADC_STATUS_SINGLEACT);
					sum += ADC_DataSingleGet(ADC0);
					adcsamplecount++;
				}
				unblock_sleep_mode(ADC_EM);               // Unblocking ADC from EM1
				ADC0->CMD |= ADC_CMD_SINGLESTOP;        // Stopping the ADC0

				//sum = sum / ADCSAMPLES;
				temp_increment = convertToCelsius((float)sum / ADCSAMPLES);

				if ((temp_increment < LOWER_TEMP ) || (temp_increment > UPPER_TEMP))
				{
					x1++;
					LED1_on();

				}
				else
				{
					x2++;
					LED1_off();
				}

			#else

				set_sleep_mode(ADC_EM);
				ADC_Start(ADC0,adcStartSingle);
			#endif
		#else
			//DMA_Setup();
			//set_sleep_mode(ADC_EM);
			//ADC_Start(ADC0,ADC_Mode);

			//set_sleep_mode(EM1);
	#endif
	//set_sleep_mode(EM1);
/*
	#ifndef ONBOARD_LIGHT_SENSOR
		if (letimer_state == 0)
		{
			initialise_TSL2651();
			letimer_state = 1;
		}
		else if(letimer_state == 1)
		{
			letimer_state = 2;
		}
		else if(letimer_state == 2)
		{
			//unblock_sleep_mode(EM1);
			disable_TSL2651();
			letimer_state = 0;
		}
	#endif
*/
	}
	else
	{
		#ifdef ONBOARD_LIGHT_SENSOR

			//ACMP_Enable(ACMP0);

		#endif
		//if LETIMER_IF_COMP1 is set or enabled
		LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);

		//GPIO_PinOutClear(LED0_PORT,LED0_PIN);

		#ifdef	ONBOARD_LIGHT_SENSOR

			 //excite();
			 //while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT));

		#endif
	}
	INT_Enable();
}

void ADC0_IRQHandler(void)
{
	// Clearing the pending interrupts
		static uint32_t sum = 0;
		float sum1;
		char str[10];
		adc_start++;
		int intFlags;
		intFlags = ADC_IntGet(ADC0);
		ADC_IntClear(ADC0, intFlags);

		sum += ADC_DataSingleGet(ADC0);
		if (adcsamplecount == ADCSAMPLES)
		{
			ADC0->CMD |= ADC_CMD_SINGLESTOP;        // Stopping the ADC0
			unblock_sleep_mode(EM1);               	// Unblocking ADC from EM1
			sum1 = (sum * 1.0) / ADCSAMPLES;
			temp_increment = convertToCelsius(sum1);
			SendToSamb11();
			sum = 0;
			adcsamplecount = 0;
			if ((temp_increment < temp_low) || (temp_increment > temp_high))
			{

				LED1_on();
				#ifdef WIFI
					strcpy(command,"AT+CIPSTART=4,\"TCP\",\"184.106.153.149\",80\r\n");
					command_size = strlen(command);
					Send_Command();

					wait();

					strcpy(command,"AT+CIPSEND=4,83\r\n");
					command_size = strlen(command);
					Send_Command();

					wait();

					strcpy(command,"GET /apps/thingtweet/1/statuses/update?api_key=GCYQQHMLYUSOD8RP&status=Emergency!\r\n");
					command_size = strlen(command);
					Send_Command();

					wait();
				#endif


			}
			else
			{
				LED1_off();
				#ifdef WIFI
					strcpy(command,"AT+CIPSTART=4,\"TCP\",\"184.106.153.149\",80\r\n");
					command_size = strlen(command);
					Send_Command();

					wait();

					strcpy(command,"AT+CIPSEND=4,44\r\n");
					command_size = strlen(command);
					Send_Command();

					wait();

					strcpy(command,"GET /update?key=B8DV0508B859SGNR&field1=");
					sprintf(str,"%d",temperature);
					strcat(command,str);
					strcat(command,"\r\n");
					command_size = strlen(command);
					Send_Command();

					wait();
				#endif

			}
		}
		adcsamplecount++;


}
void setup_I2C(void)
{
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_I2C1, true);
	// Using default settings
	I2C_Init_TypeDef i2cInit = {
			  .enable	= true,                      /* Enable when init done */
			  .master	= true,						/* Set to master mode */
			  .refFreq 	= 0,                       /* Use currently configured reference clock */
			  .freq		= I2C_FREQ_STANDARD_MAX,   	/* Set to standard rate assuring being */
			                             	 	 	 /* within I2C spec */
			  .clhr 	= I2C_CLK_DUTYCYCLE,        /* Set to use 4:4 low/high duty cycle */
	  };

	  /* Enable pins at location */
	  I2C1->ROUTE = I2C_ROUTE_SDAPEN |I2C_ROUTE_SCLPEN |(0 << _I2C_ROUTE_LOCATION_SHIFT);

	  /* Initializing the I2C */
	  I2C_Init(I2C1, &i2cInit);

	  if(I2C1->STATE & I2C_STATE_BUSY)
	  {
		  I2C1->CMD = I2C_CMD_ABORT;
	  }
}

/*************************************************************/
/* i2c write and read driver function        				 */
/* Input Variables - None						 			 */
/* Global Variables - i2c_tx_buffer[], array to store slave
 * address,	register address, data 							 */
/* Return Variables - None			 						 */
/*************************************************************/
void I2C_write(void)
{
	I2C1->TXDATA = (SLAVE_ADDRESS << 1) | I2C_WRITE_FLAG;
	I2C1->CMD |= I2C_CMD_START;
	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IFC_ACK;

	I2C1->TXDATA = i2c_tx_buffer[0];  //register address
	while((I2C1->IF & I2C_IF_ACK) == 0);

	I2C1->IFC = I2C_IFC_ACK;

	I2C1->TXDATA = i2c_tx_buffer[1];  //data
	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IFC_ACK;

	I2C1->CMD = I2C_CMD_STOP;
}

/*************************************************************/
/* i2c read driver function        				             */
/* Input Variables - None						 			 */
/* Global Variables - i2c_tx_buffer[], i2c_rx_buffer[]
 * array to store slave address,register address, read data  */
/* Return Variables - None			 						 */
/*************************************************************/

void I2C_read(void)
{
	I2C1->TXDATA = (SLAVE_ADDRESS << 1) | I2C_WRITE_FLAG;

	//for(int i=0;i<300;i++);

	I2C1->CMD |= I2C_CMD_START;

	while((I2C1->IF & I2C_IF_ACK) == 0);

	I2C1->IFC = I2C_IFC_ACK;

	//for(int i=0;i<300;i++);

	I2C1->TXDATA = i2c_tx_buffer[0];  //register address

	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->CMD |= I2C_CMD_START;

	I2C1->TXDATA = (SLAVE_ADDRESS << 1) | I2C_READ_FLAG;
	while((I2C1->IF & I2C_IF_ACK) == 0);
	I2C1->IFC = I2C_IFC_ACK;
	while((I2C1->STATUS & I2C_STATUS_RXDATAV) == 0);
	//I2C1->IFC = I2C_IFC_ACK;

	//I2C1->CMD = I2C_CMD_ACK;
	I2C1->CMD = I2C_CMD_NACK;
	I2C1->CMD = I2C_CMD_STOP;
	i2c_rx_buffer[0]= I2C1->RXDATA;  //read data
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->IFC = I2C_IFC_NACK;
	//while((I2C1->IF & I2C_IF_MSTOP) == 0);
	//I2C1->IFC = I2C_IFC_MSTOP;
}

/*************************************************************/
/* Initialize tsl2651 function       				          */
/* Input Variables - None						 			  */
/* Global Variables - i2c_tx_buffer[], array to store slave
 * address,	register address, data
 * Return Variables - None			 						 */
/*************************************************************/

void initialise_TSL2651(void)
{
	/* Using PC4 (SDA) and PC5 (SCL) , PD0 GPIO power, PD1 interrupt from TSL2651  */
	  GPIO_PinModeSet(I2C1_SDASCL_PORT, I2C1_SDA_PIN, gpioModeWiredAndPullUpFilter, 0);
	  GPIO_PinModeSet(I2C1_SDASCL_PORT, I2C1_SCL_PIN, gpioModeWiredAndPullUpFilter, 0);
	  GPIO_PinModeSet(I2C1_GPIO_PORT, I2C1_GPIO_POWER_PIN, gpioModePushPull, 0);
	  GPIO_PinModeSet(I2C1_GPIO_PORT, I2C1_GPIO_INTERRUPT_PIN, gpioModeInput, 0);

		GPIO_PinOutSet(I2C1_GPIO_PORT, I2C1_GPIO_POWER_PIN);
		for(int i=0; i<1000;i++)
		{
			/*wait for power on reset time*/
		}
		//GPIO_PinOutSet(I2C1_SDASCL_PORT, I2C1_SDA_PIN);
		//GPIO_PinOutSet(I2C1_SDASCL_PORT, I2C1_SCL_PIN);
		//GPIO_PinOutSet(I2C1_GPIO_PORT, I2C1_GPIO_INTERRUPT_PIN);

		//enable_gesture_sensor();
		GPIO_IntConfig(I2C1_GPIO_PORT, I2C1_GPIO_INTERRUPT_PIN,false, true, true);   // falling edge
		NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
		NVIC_EnableIRQ(GPIO_ODD_IRQn);
		enable_gesture_sensor();

}

/*************************************************************/
/* GPIO interrupt handler for external interrupt on PD1       */
/* Input Variables - None						 			  */
/* Global Variables - i2c_tx_buffer[], array to store slave
 * address,	register address, data
 * Return Variables - None			 						 */
/*************************************************************/
void GPIO_ODD_IRQHandler(void)
{
	INT_Disable();

	GPIO_IntClear(GPIO_IntGet()); //clear all pending gpio interrupts
	GPIO->IFC = GPIO->IF;
	LED1_on();
	read_gesture();

	int count = 0,i = 0;
	i2c_tx_buffer[0] = 0xAE;
	I2C_read();
	GFLVL = i2c_rx_buffer[0];

	while(GFLVL != 0)
	{
		for(i=0;i<3000;i++);
		i2c_tx_buffer[0] = 0xFC;
		I2C_read();
		U[count] = i2c_rx_buffer[0];
		i2c_tx_buffer[0] = 0xFD;
		I2C_read();
		D[count] = i2c_rx_buffer[0];
		i2c_tx_buffer[0] = 0xFE;
		I2C_read();
		L[count] = i2c_rx_buffer[0];
		i2c_tx_buffer[0] = 0xFF;
		I2C_read();
		R[count] = i2c_rx_buffer[0];
		count++;
	}

	int PINT;
	i2c_tx_buffer[0] = 0x93;
	I2C_read();
	PINT = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = APDS9960_GCONF4;
	i2c_tx_buffer[1] = 0x02;
	I2C_write();
	LED1_off();

	INT_Enable();
}

/*************************************************************/
/* disable TSL2651 setup, disables all GPIO, TSL2651 and clears
 * all interrupts in appropriate sequence       			  */
/* Input Variables - None						 			  */
/* Global Variables - i2c_tx_buffer[], array to store slave
 * address,	register address, data
 * Return Variables - None			 						 */
/*************************************************************/
void disable_TSL2651(void)
{
	i2c_tx_buffer[0] = APDS9960_ENABLE;
	i2c_tx_buffer[1] = 0x00;
	I2C_write();

	GPIO_PinModeSet(I2C1_SDASCL_PORT, I2C1_SDA_PIN, gpioModeDisabled, 0);
	GPIO_PinModeSet(I2C1_SDASCL_PORT, I2C1_SCL_PIN, gpioModeDisabled, 0);
	GPIO_PinModeSet(I2C1_GPIO_PORT, I2C1_GPIO_INTERRUPT_PIN, gpioModeDisabled, 0);
	GPIO_PinOutClear(I2C1_GPIO_PORT, I2C1_GPIO_POWER_PIN);
	GPIO_PinModeSet(I2C1_GPIO_PORT, I2C1_GPIO_POWER_PIN, gpioModeDisabled, 0);

	//LED1_off();
	//power off the device

	// Disable and clear the interrupts
	//GPIO_ExtIntConfig(I2C1_GPIO_PORT, I2C1_GPIO_INTERRUPT_PIN,1,false, true, false);
	GPIO_IntClear(GPIO_IntGet());  // Disabling the interrupts
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	//GPIO_DriveModeSet(I2C1_GPIO_PORT, gpioDriveModeLowest);

}

void LEUART_setup(void)
{

	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_OscillatorEnable(cmuOsc_LFXO,true,true);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);

	  /* Enable CORE LE clock in order to access LE modules */
	  CMU_ClockEnable(cmuClock_CORELE, true);

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_LEUART0, true);    /* Enable LEUART1 clock */
	CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1);

	LEUART_Reset(LEUART0);

	LEUART_Init_TypeDef leuart0Init =
	{
	  .enable   = LEUART_INIT_STATUS,       /* Activate data reception on LEUn_TX pin. */
	  .refFreq  = 0,                    /* Inherit the clock frequenzy from the LEUART clock source */
	  .baudrate = LEUART_BAUD_RATE,                 /* Baudrate = 9600 bps */
	  .databits = LEUART_FRAME_BITS,      /* Each LEUART frame containes 8 databits */
	  .parity   = LEUART_PARITY,       /* No parity bits in use */
	  .stopbits = LEUART_STOPBIT,      /* Setting the number of stop bits in a frame to 2 bitperiods */
	};

	LEUART_Init(LEUART0, &leuart0Init);
	LEUART_IntClear(LEUART0, _LEUART_IF_MASK);
	GPIO_PinModeSet(LEUART0_GPIO,LEUART0_TX_PIN,gpioModePushPull,1);
	GPIO_PinModeSet(LEUART0_GPIO,LEUART0_RX_PIN,gpioModeInput,1);

	LEUART0->ROUTE = LEUART_ROUTE_RXPEN |LEUART_ROUTE_TXPEN| LEUART_ROUTE_LOCATION_LOC0;

	//GPIO_PinModeSet(LEUART0_GPIO,LEUART0_TX_PIN,gpioModePushPull,1);
	//GPIO_PinModeSet(LEUART0_GPIO,LEUART0_RX_PIN,gpioModeInput,1);

	LEUART0->IEN |= LEUART_IEN_TXC;
//	LEUART0->CTRL |= LEUART_CTRL_LOOPBK;
//	LEUART0->CTRL |= LEUART_CTRL_AUTOTRI;

	LEUART_Enable(LEUART0, leuartEnable);
	NVIC_EnableIRQ(LEUART0_IRQn);
}

/*void LEUART0_IRQHandler(void)
{
	INT_Disable();
	static n=0;
	int leuart0_interrupt = LEUART_IntGet(LEUART0);

	LEUART_IntClear(LEUART0, leuart0_interrupt);
	while((LEUART0->IF & LEUART_IF_RXDATAV)==0);
	dummyarray[n++]= (uint8_t)LEUART0->RXDATA;

	int leuart0_interrupt = LEUART_IntGet(LEUART0);
	LEUART_IntClear(LEUART0, leuart0_interrupt);

	if(bytes_to_transfer != (length_buff-1))
	{
		bytes_to_transfer++;
#ifdef LEUART_ENABLE
	#ifdef CIRCULAR_BUFFER

		LEUART0->TXDATA = cbuffer_remove(&cb,1);
	#else
 		valuetosend = arraytosend[bytes_to_transfer];
		LEUART0->TXDATA = valuetosend;
		dummyarray[n++] = LEUART0->RXDATA;
	#endif

#endif
	}
	else
	{
		bytes_to_transfer=0;
		unblock_sleep_mode(ENERGYMODE_LEUART);
	}


	INT_Enable();
}*/

void SendToSamb11(void)
{
	bytes_to_transfer = 0;
	intparttosend = (uint8_t)temp_increment;
	floatparttosend = (uint8_t)((temp_increment - intparttosend)*10);

#ifdef LEUART_DMA
	#ifdef CIRCULAR_BUFFER

		cbuffer_add(&cb,167,1);
		cbuffer_add(&cb,0x01,1);
		cbuffer_add(&cb,168,1);
		cbuffer_add(&cb,intparttosend,1);
		cbuffer_add(&cb,169,1);
		cbuffer_add(&cb,floatparttosend,1);


		while(bytes_to_transfer < length_buff)
		{
			arraytosend[bytes_to_transfer++] = cbuffer_remove(&cb,1);
		}
		Send_Values();

	#else
		arraytosend[0]=50;
		arraytosend[1]=intparttosend;
		arraytosend[2]=floatparttosend;
	#endif
#endif
	//set_sleep_mode(ENERGYMODE_LEUART);
#ifdef LEUART_ENABLE
	#ifdef CIRCULAR_BUFFER
	#else
		LEUART0->TXDATA = arraytosend[bytes_to_transfer];
	#endif

#endif
}

void reset_gesture_parameters()
{
	    gesture_data_.index = 0;
	    gesture_data_.total_gestures = 0;

	    gesture_ud_delta_ = 0;
	    gesture_lr_delta_ = 0;

	    gesture_ud_count_ = 0;
	    gesture_lr_count_ = 0;

	    gesture_near_count_ = 0;
	    gesture_far_count_ = 0;

	    gesture_state_ = 0;
	    gesture_motion_ = DIR_NONE;
}

void enable_gesture_sensor()
{
	//i2c_tx_buffer[0] = 0x80; //power off control register
	//i2c_tx_buffer[1] = 0x00; //power off control register
	//I2C_write();

/*	i2c_tx_buffer[0] = APDS9960_ENABLE; //set all enable
i2c_tx_buffer[1] = 0x7F;
I2C_write();
*/
	i2c_tx_buffer[0] = APDS9960_GPENTH; //power on control register
	i2c_tx_buffer[1] = DEFAULT_GPENTH; //power on control register
	I2C_write();

	i2c_tx_buffer[0] = APDS9960_GEXTH;
	i2c_tx_buffer[1] = DEFAULT_GEXTH;
	I2C_write();

	i2c_tx_buffer[0] = APDS9960_GCONF1;
	i2c_tx_buffer[1] = DEFAULT_GCONF1;
	I2C_write();

	i2c_tx_buffer[0] = APDS9960_GCONF2; //read gesture gain
	I2C_read();

	int val1,val2;

	val1 = DEFAULT_GGAIN;
	val1 &= 0b00000011;
	val1 = val1 << 5;
	val2 = i2c_rx_buffer[0];
	/* Shift and mask out GGAIN bits */
	val2 = (val2 >> 5) & 0b00000011;
	val2 &= 0b10011111;
	val2 |= val1;

	i2c_tx_buffer[0] = APDS9960_GCONF2;
	i2c_tx_buffer[1] = val2;	//set gesture gain
	I2C_write();

	i2c_tx_buffer[0] = APDS9960_GCONF2; //read LED DRIVE
	I2C_read();

	val1 = DEFAULT_GLDRIVE; //read led drive
	val1 &= 0b00000011;
	val1 = val1 << 3;
	val2 = i2c_rx_buffer[0];
	/* Shift and mask out GLDRIVE bits */
	val2 = (val2 >> 3) & 0b00000011;
	val2 &= 0b11100111;
	val2 |= val1;

	i2c_tx_buffer[0] = APDS9960_GCONF2; //set led drive
	i2c_tx_buffer[1] = val2;
	I2C_write();

	i2c_tx_buffer[0] = APDS9960_GCONF2; //READ GW TIME
	I2C_read();

	val1 = DEFAULT_GWTIME; //read GW TIME
	/* Set bits in register to given value WAIT TIME 39.2 MS*/
	val1 &= 0b00000111;
	val2 = i2c_rx_buffer[0];
	/* Mask out GWTIME bits */
	val2 &= 0b00000111;
	val2 &= 0b11111000;
	val2 |= val1;

	i2c_tx_buffer[0] = APDS9960_GCONF2; //set gw time
	i2c_tx_buffer[1] = val2;
	I2C_write();

	i2c_tx_buffer[0] = APDS9960_GOFFSET_U; //threshold low value to be set to 0x08
	i2c_tx_buffer[1] = DEFAULT_GOFFSET; //threshold low value to be set to 0x08
	I2C_write();


	i2c_tx_buffer[0] = APDS9960_GOFFSET_D; 	//interrupt register value to be set to 0x14 1= level interrupt, 4=persistence
	i2c_tx_buffer[1] = DEFAULT_GOFFSET;
	I2C_write();

	i2c_tx_buffer[0] = APDS9960_GOFFSET_L; 	//interrupt register value to be set to 0x14 1= level interrupt, 4=persistence
	i2c_tx_buffer[1] = DEFAULT_GOFFSET;
	I2C_write();

	i2c_tx_buffer[0] = APDS9960_GOFFSET_R; 	//interrupt register value to be set to 0x14 1= level interrupt, 4=persistence
	i2c_tx_buffer[1] = DEFAULT_GOFFSET;
	I2C_write();

	i2c_tx_buffer[0] = APDS9960_GPULSE; 	//interrupt register value to be set to 0x14 1= level interrupt, 4=persistence
	i2c_tx_buffer[1] = DEFAULT_GPULSE;
	I2C_write();

	i2c_tx_buffer[0] = APDS9960_GCONF3; 	//interrupt register value to be set to 0x14 1= level interrupt, 4=persistence
	i2c_tx_buffer[1] = DEFAULT_GCONF3;
	I2C_write();

/*
	i2c_tx_buffer[0] = APDS9960_GCONF4; //read gesture enable register
	I2C_read();

	val1 = DEFAULT_GIEN;
	val1 &= 0b00000001;
	val1 = val1 << 1;
	val2 = i2c_rx_buffer[0];
	 Shift and mask out bits
	val2 = val2 >> 1 & 0b00000001;
	val2 &= 0b11111101;
	val2 |= val1;

	i2c_tx_buffer[0] = APDS9960_GCONF4; //set GIEN TO ZERO DISABLE  INTERRUPTS
	i2c_tx_buffer[1] = val2;
*/
	i2c_tx_buffer[0] = APDS9960_GCONF4; //set GIEN GMODE TO 1 enable  INTERRUPTS
	i2c_tx_buffer[1] = 0x03;
	I2C_write();

	/* Enable gesture mode
	       Set ENABLE to 0 (power off)
	       Set WTIME to 0xFF
	       Set AUX to LED_BOOST_300
	       Enable PON, WEN, PEN, GEN in ENABLE
	 */

	i2c_tx_buffer[0] = APDS9960_WTIME; //wtime register
	i2c_tx_buffer[1] = 0xFF; //wtime vaule for enable gesture sensor
	I2C_write();

	i2c_tx_buffer[0] = APDS9960_PPULSE; //p pulse
	i2c_tx_buffer[1] = DEFAULT_GESTURE_PPULSE; //default ppulse
	I2C_write();

	i2c_tx_buffer[0] = APDS9960_CONFIG2; // led boost
	I2C_read();

	val1 = LED_BOOST_300;
	/* Set bits in register to given value */
	val1 &= 0b00000011;
	val1 = val1 << 4;
	val2 = i2c_rx_buffer[0];
	val2 = (val2 >> 4) & 0b00000011;
	val2 &= 0b11001111;
	val2 |= val1;

	i2c_tx_buffer[0] = APDS9960_CONFIG2; //write led boost value
	i2c_tx_buffer[1] = val2;
	I2C_write();

/*	i2c_tx_buffer[0] = APDS9960_GCONF4; //read gesture interrupt value
	I2C_read();

	i2c_tx_buffer[0] = APDS9960_GCONF4; //read gesture mode
	I2C_read();

	val1 = 0b00000001; //enable gmode bit
	 Set bits in register to given value
	val1 &= 0b00000001;
	val2 = i2c_rx_buffer[0];
	val2 &= 0b00000001;
	val2 &= 0b11111110;
	val2 |= val1;

	i2c_tx_buffer[0] = APDS9960_GCONF4; //write gesture mode set gesture mode
	i2c_tx_buffer[1] = val2;
	I2C_write();*/

	i2c_tx_buffer[0] = APDS9960_ENABLE; // enable power, wait, proximity and gesture bit in enable register
	i2c_tx_buffer[1] = (APDS9960_PON|APDS9960_GEN|APDS9960_WEN|APDS9960_PEN);
	I2C_write();


	//reset_gesture_parameters();
	//read_gesture();
	//enable gpio interrupts
//till here us enable gesture sensor

}
void read_gesture()
{

	int val1, val2;
	//i2c_tx_buffer[0] = APDS9960_GSTATUS;
	//I2C_read();

	//val2 = i2c_rx_buffer[0];
	/* Shift and mask out GVALID bit */
	//val2 &= APDS9960_GVALID;

//starts read gesture

#ifdef DEBUG
	int i=0;
	i2c_tx_buffer[0] = 0x80; //read enable register 0-PON, 6- GEN
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = 0xAB; //read GIEN 1 - GIEN
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = 0xAB; //read GIEN 1 - GIEN, 0 -GMODE
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = 0xA0; //read GPENTH
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = 0xA1; //read GPENTH
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = 0xA2;
	/*
	 * 	GCONFIG1<GFIFOTH> 0xA2<7:6> Gesture FIFO Threshold
	 * 	GCONFIG1<GEXMSK> 0xA2<5:2> Gesture Exit Mask
	 * 	GCONFIG1<GEXPERS> 0xA2<1:0> Gesture Exit Persistence
	 */
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = 0xA3;
	/*
	 * GCONFIG2<GGAIN> 0xA3<6:5> Gesture Gain Control
	 * GCONFIG2<GLDRIVE> 0xA3<4:3> Gesture LED Drive Strength
	 * GCONFIG2<GWTIME> 0xA3<2:0> Gesture Wait Time
	 */
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = 0x93;
	/*
	 * STATUS - PGSAT - 6
	 */
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = APDS9960_CONFIG2;
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = 0xA4;
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];
	i2c_tx_buffer[0] = 0xA5;
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];
	i2c_tx_buffer[0] = 0xA7;
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];
	i2c_tx_buffer[0] = 0xA9;
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = 0xAE; //GFVL
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = 0xAF; //GFOV - 1, GFVALID - 0
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];

	i2c_tx_buffer[0] = 0xFC; //GFOV - 1, GFVALID - 0
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];
	i2c_tx_buffer[0] = 0xFD; //GFOV - 1, GFVALID - 0
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];
	i2c_tx_buffer[0] = 0xFE; //GFOV - 1, GFVALID - 0
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];
	i2c_tx_buffer[0] = 0xFF; //GFOV - 1, GFVALID - 0
	I2C_read();
	debug_array[i++] = i2c_rx_buffer[0];



#endif

	uint8_t fifo_level = 0;
    uint8_t bytes_read = 0;
    uint8_t fifo_data[128];
    uint8_t gstatus;
    int motion;

    /* Wait some time to collect next batch of FIFO data */
    for(int i=0;i<100;i++);

	i2c_tx_buffer[0] = APDS9960_GFLVL; // read fifo level
	I2C_read();
	fifo_level = (uint8_t)i2c_rx_buffer[0];
	while(fifo_level > 0 && bytes_read<(fifo_level * 4))
	{

		i2c_tx_buffer[0] = APDS9960_GFIFO_U; // read fifo level
		I2C_read();
		fifo_data[bytes_read] = (uint8_t)i2c_rx_buffer[0];

		bytes_read++;
	}


    /* If at least 1 set of data, sort the data into U/D/L/R */
    if( bytes_read >= 4 )
    {
        for( i = 0; i < bytes_read; i += 4 )
        {
            gesture_data_.u_data[gesture_data_.index] =
                                                fifo_data[i + 0];
            gesture_data_.d_data[gesture_data_.index] =
                                                fifo_data[i + 1];
            gesture_data_.l_data[gesture_data_.index] =
                                                fifo_data[i + 2];
            gesture_data_.r_data[gesture_data_.index] =
                                                fifo_data[i + 3];
            gesture_data_.index++;
            gesture_data_.total_gestures++;
        }
    }

	uint8_t u_first = 0;
	    uint8_t d_first = 0;
	    uint8_t l_first = 0;
	    uint8_t r_first = 0;
	    uint8_t u_last = 0;
	    uint8_t d_last = 0;
	    uint8_t l_last = 0;
	    uint8_t r_last = 0;
	    int ud_ratio_first;
	    int lr_ratio_first;
	    int ud_ratio_last;
	    int lr_ratio_last;
	    int ud_delta;
	    int lr_delta;
	    i=0;


	    /* Check to make sure our data isn't out of bounds */
	    if( (gesture_data_.total_gestures <= 32) &&
	        (gesture_data_.total_gestures > 0) ) {

	        /* Find the first value in U/D/L/R above the threshold */
	        for( i = 0; i < gesture_data_.total_gestures; i++ ) {
	            if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
	                (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
	                (gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
	                (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {

	                u_first = gesture_data_.u_data[i];
	                d_first = gesture_data_.d_data[i];
	                l_first = gesture_data_.l_data[i];
	                r_first = gesture_data_.r_data[i];
	                break;
	            }
	        }
		    i=0;
		    debug_array1[i++]=u_first;
		    debug_array1[i++]=d_first;
		    debug_array1[i++]=l_first;
		    debug_array1[i++]=r_first;


	        /* Find the last value in U/D/L/R above the threshold */
	        for( i = gesture_data_.total_gestures - 1; i >= 0; i-- ) {

	            if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
	                (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
	                (gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
	                (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {

	                u_last = gesture_data_.u_data[i];
	                d_last = gesture_data_.d_data[i];
	                l_last = gesture_data_.l_data[i];
	                r_last = gesture_data_.r_data[i];
	                break;
	            }
	        }
	    }
	    i=0;
	    debug_array1[i++]=u_last;
	    debug_array1[i++]=d_last;
	    debug_array1[i++]=l_last;
	    debug_array1[i++]=r_last;

	   // Calculate the first vs. last ratio of up/down and left/right
	    ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
	    lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
	    ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
	    lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);


	     //Determine the difference between the first and last ratios
	    ud_delta = ud_ratio_last - ud_ratio_first;
	    lr_delta = lr_ratio_last - lr_ratio_first;


	     //Accumulate the UD and LR delta values
	    gesture_ud_delta_ += ud_delta;
	    gesture_lr_delta_ += lr_delta;

	     //Determine U/D gesture
	    if( gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 ) {
	        gesture_ud_count_ = 1;
	    } else if( gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 ) {
	        gesture_ud_count_ = -1;
	    } else {
	        gesture_ud_count_ = 0;
	    }

	     //Determine L/R gesture
	    if( gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 ) {
	        gesture_lr_count_ = 1;
	    } else if( gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 ) {
	        gesture_lr_count_ = -1;
	    } else {
	        gesture_lr_count_ = 0;
	    }

	     //Determine Near/Far gesture
	    if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 0) ) {
	        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) &&
	            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {

	            if( (ud_delta == 0) && (lr_delta == 0) ) {
	                gesture_near_count_++;
	            } else if( (ud_delta != 0) || (lr_delta != 0) ) {
	                gesture_far_count_++;
	            }

	            if( (gesture_near_count_ >= 10) && (gesture_far_count_ >= 2) ) {
	                if( (ud_delta == 0) && (lr_delta == 0) ) {
	                    gesture_state_ = NEAR_STATE;
	                } else if( (ud_delta != 0) && (lr_delta != 0) ) {
	                    gesture_state_ = FAR_STATE;
	                }
	                uint8_t process_gesture = 1;
		            }
	        }
	    } else {
	        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) &&
	            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {

	            if( (ud_delta == 0) && (lr_delta == 0) ) {
	                gesture_near_count_++;
	            }

	            if( gesture_near_count_ >= 10 ) {
	                gesture_ud_count_ = 0;
	                gesture_lr_count_ = 0;
	                gesture_ud_delta_ = 0;
	                gesture_lr_delta_ = 0;
	            }
	        }
	    }


	    // Return if near or far event is detected
	        if( gesture_state_ == NEAR_STATE ) {
	            gesture_motion_ = DIR_NEAR;
	            return;
	        } else if ( gesture_state_ == FAR_STATE ) {
	            gesture_motion_ = DIR_FAR;
	            return;
	        }

	         //Determine swipe direction
	        if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 0) ) {
	            gesture_motion_ = DIR_UP;
	        } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 0) ) {
	            gesture_motion_ = DIR_DOWN;
	        } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 1) ) {
	            gesture_motion_ = DIR_RIGHT;
	        } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == -1) ) {
	            gesture_motion_ = DIR_LEFT;
	        } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 1) ) {
	            if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
	                gesture_motion_ = DIR_UP;
	            } else {
	                gesture_motion_ = DIR_RIGHT;
	            }
	        } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == -1) ) {
	            if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
	                gesture_motion_ = DIR_DOWN;
	            } else {
	                gesture_motion_ = DIR_LEFT;
	            }
	        } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == -1) ) {
	            if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
	                gesture_motion_ = DIR_UP;
	            } else {
	                gesture_motion_ = DIR_LEFT;
	            }
	        } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 1) ) {
	            if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
	                gesture_motion_ = DIR_DOWN;
	            } else {
	                gesture_motion_ = DIR_RIGHT;
	            }
	        } else {
	            return;
	        }

	        return;
	        //reset_gesture_parameters();
}

