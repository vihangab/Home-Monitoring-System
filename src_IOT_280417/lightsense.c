/**************************************************************************//**
 * @file
 * @brief LESENSE demo for EFM32LG_STK3600
 * @version 5.0.0
 ******************************************************************************
 * @section License
 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "main.h"

/**************************************************************************//**
 * Macro definitions
 *****************************************************************************/

/* Default configuration for alternate excitation channel. */
#define LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF                                    \
  {                                                                             \
    false,                  /* Alternate excitation enabled.*/                  \
    lesenseAltExPinIdleDis, /* Alternate excitation pin is disabled in idle. */ \
    false                   /* Excite only for corresponding channel. */        \
  }

/*
 ACMP
#define ACMP_NEG_REF           		acmpChannelVDD
#define ACMP_THRESHOLD_LOW         (0x35)                         // Reference value for the lightsensor.
#define ACMP_THRESHOLD_HIGH        (0x3D)                              * Value works well in office light
                                                          * conditions. Might need adjustment
                                                             * for other conditions.
*/

LESENSE_ChDesc_TypeDef initLesenseCh =
  {
    .enaScanCh     = true,
    .enaPin        = false,                 /* Pin is input, no enabling needed. Separate pin is exciting the sensor. */
    .enaInt        = true,                  /* Enable interrupt for this channel. */
    .chPinExMode   = lesenseChPinExHigh,    /* Excite by pullin pin high. */
    .chPinIdleMode = lesenseChPinIdleDis,   /* During Idle, excite pin should be disabled (tri-stated). */
    .useAltEx      = true,                  /* Use alternate excite pin. */
    .shiftRes      = false,                 /* Not applicable, only for decoder operation. */
    .invRes        = false,                 /* No need to invert result. */
    .storeCntRes   = true,                  /* Not applicable, don't care really. */
    .exClk         = lesenseClkLF,          /* Using low frequency clock for timing the excitation. */
    .sampleClk     = lesenseClkLF,          /* Using low frequency clock for timing the sample instant. */
    .exTime        = 0x01,                  /* 1 LFclk cycle is enough excitation time, this depends on response time of light sensor. */
    .sampleDelay   = 0x01,                  /* Sampling should happen when excitation ends, it it happens earlier, excitation time might as well be reduced. */
    .measDelay     = 0x00,                  /* Not used here, basically only used for applications which uses the counting feature. */
    .acmpThres     = ACMP_THRESHOLD_LOW,        /* This is the analog comparator threshold setting, determines when the acmp triggers. */
    .sampleMode    = lesenseSampleModeACMP, /* Sampling acmp, not counting. */
    .intMode       = lesenseSetIntNegEdge,  /* Interrupt when voltage falls below threshold. */
    .cntThres      = 0x0000,                /* Not applicable. */
    .compMode      = lesenseCompModeLess    /* Not applicable. */
  };

LESENSE_ChDesc_TypeDef initLesenseCh1 =
  {
    .enaScanCh     = true,
    .enaPin        = false,                 /* Pin is input, no enabling needed. Separate pin is exciting the sensor. */
    .enaInt        = true,                  /* Enable interrupt for this channel. */
    .chPinExMode   = lesenseChPinExHigh,    /* Excite by pullin pin high. */
    .chPinIdleMode = lesenseChPinIdleDis,   /* During Idle, excite pin should be disabled (tri-stated). */
    .useAltEx      = true,                  /* Use alternate excite pin. */
    .shiftRes      = false,                 /* Not applicable, only for decoder operation. */
    .invRes        = false,                 /* No need to invert result. */
    .storeCntRes   = true,                  /* Not applicable, don't care really. */
    .exClk         = lesenseClkLF,          /* Using low frequency clock for timing the excitation. */
    .sampleClk     = lesenseClkLF,          /* Using low frequency clock for timing the sample instant. */
    .exTime        = 0x01,                  /* 1 LFclk cycle is enough excitation time, this depends on response time of light sensor. */
    .sampleDelay   = 0x01,                  /* Sampling should happen when excitation ends, it it happens earlier, excitation time might as well be reduced. */
    .measDelay     = 0x00,                  /* Not used here, basically only used for applications which uses the counting feature. */
    .acmpThres     = ACMP_THRESHOLD_HIGH,        /* This is the analog comparator threshold setting, determines when the acmp triggers. */
    .sampleMode    = lesenseSampleModeACMP, /* Sampling acmp, not counting. */
    .intMode       = lesenseSetIntPosEdge,  /* Interrupt when voltage falls below threshold. */
    .cntThres      = 0x0000,                /* Not applicable. */
    .compMode      = lesenseCompModeLess    /* Not applicable. */
  };

  /* GPIO
  #define LED_GPIO_PORT gpioPortE
  #define LED_GPIO_PIN  2
	 LESENSE
  #define LIGHTSENSE_CH             6
  #define LIGHTSENSE_EXCITE_PORT    gpioPortD
  #define LIGHTSENSE_EXCITE_PIN     6
  #define LIGHTSENSE_SENSOR_PORT    gpioPortC
  #define LIGHTSENSE_SENSOR_PIN     6
  #define LCSENSE_SCAN_FREQ         20
  #define LIGHTSENSE_INTERRUPT      LESENSE_IF_CH6
*/


/**************************************************************************//**
 * Interrupt handlers prototypes
 *****************************************************************************/
//void LESENSE_IRQHandler(void);

/**************************************************************************//**
 * Functions prototypes
 *****************************************************************************/
void setupCMU(void);
void setupACMP(void);
void setupLESENSE(void);
void setupGPIO(void);

/**************************************************************************//**
 * Interrupt handlers
 *****************************************************************************/
/**************************************************************************//**
 * @brief LESENSE_IRQHandler
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/
/**************************************************************************//**
 * @brief  LESENSE interrupt handler
 *****************************************************************************/
/*
void LESENSE_IRQHandler(void)
{
	  Clear interrupt flag
	  LESENSE_IntClear(LIGHTSENSE_INTERRUPT);


	  if (LESENSE_IF_CH6 & LESENSE->IF)//LESENSE_IntGetEnabled())
	   {
	     LESENSE_IntClear(LESENSE_IF_CH6);
	   }
	   Turn on user led
	  if(state == 0){
		  state = 1;
		  GPIO_PinOutSet(LED_GPIO_PORT, LED_GPIO_PIN);
		  initLesenseCh.acmpThres = ACMP_THRESHOLD_HIGH;
		  initLesenseCh.intMode = lesenseSetIntPosEdge;
		  LESENSE_ChannelConfig(&initLesenseCh, LIGHTSENSE_CH);
	  }
	  else {
		  state = 0;
		  GPIO_PinOutClear(LED_GPIO_PORT, LED_GPIO_PIN);
		  initLesenseCh.acmpThres = ACMP_THRESHOLD_LOW;
		  initLesenseCh.intMode = lesenseSetIntNegEdge;
		  LESENSE_ChannelConfig(&initLesenseCh, LIGHTSENSE_CH);
	  }
}
*/



/**************************************************************************//**
 * @brief  Setup the CMU
 *****************************************************************************/
void setupCMU(void)
{
	  /* Ensure core frequency has been updated */
	  SystemCoreClockUpdate();

	  /* ACMP */
	  CMU_ClockEnable(cmuClock_ACMP0, true);

	  /* GPIO */
	  CMU_ClockEnable(cmuClock_GPIO, true);

	/* Low energy peripherals
	 *   LESENSE
	 *   LFRCO clock must be enables prior to enabling
	 *   clock for the low energy peripherals */
	  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
	  CMU_ClockEnable(cmuClock_CORELE, true);
	  CMU_ClockEnable(cmuClock_LESENSE, true);

	  /* RTC */
	  //CMU_ClockEnable(cmuClock_RTC, true);

	  /* Disable clock source for LFB clock. */
	  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);
}


/**************************************************************************//**
 * @brief  Setup the GPIO
 *****************************************************************************/
void setupGPIO(void)
{
	  /* Configure the drive strength of the ports for the light sensor. */
	  GPIO_DriveModeSet(LIGHTSENSE_EXCITE_PORT, gpioDriveModeStandard);
	  GPIO_DriveModeSet(LIGHTSENSE_SENSOR_PORT, gpioDriveModeStandard);

	  /* Initialize the 2 GPIO pins of the light sensor setup. */
	  GPIO_PinModeSet(LIGHTSENSE_EXCITE_PORT, LIGHTSENSE_EXCITE_PIN, gpioModePushPull, 0);
	  GPIO_PinModeSet(LIGHTSENSE_SENSOR_PORT, LIGHTSENSE_SENSOR_PIN, gpioModeDisabled, 0);

	  /* Configure user led as output */
	  GPIO_PinModeSet(LED1_PORT, LED1_PIN, gpioModePushPull, 0);

}


/**************************************************************************//**
 * @brief  Setup the ACMP
 *****************************************************************************/
void setupACMP(void)
{
	  /* Configuration structure for ACMP */
	  static const ACMP_Init_TypeDef acmpInit =
	  {
	    .fullBias                 = false, /* The lightsensor is slow acting, */
	    .halfBias                 = true,  /* comparator bias current can be set to lowest setting.*/
	    .biasProg                 = 0x0,   /* Analog comparator will still be fast enough */
	    .interruptOnFallingEdge   = false, /* No comparator interrupt, lesense will issue interrupts. */
	    .interruptOnRisingEdge    = false,
	    .warmTime                 = acmpWarmTime512, /* Not applicable, lesense controls this. */
	    .hysteresisLevel          = acmpHysteresisLevel5, /* Some hysteresis will prevent excessive toggling. */
	    .inactiveValue            = false, /* Not applicable, lesense controls this. */
	    .lowPowerReferenceEnabled = false, /* Can be enabled for even lower power. */
	    .vddLevel                 = 0x00,  /* Not applicable, lesense controls this through .acmpThres value. */
	    .enable                   = false  /* Not applicable, lesense controls this. */
	  };

	  /* Initialize ACMP */
	  ACMP_Init(ACMP0, &acmpInit);
	  /* Disable ACMP0 out to a pin. */
	  ACMP_GPIOSetup(ACMP0, 0, false, false);
	  /* Set up ACMP negSel to VDD, posSel is controlled by LESENSE. */
	  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel0);
	  /* LESENSE controls ACMP thus ACMP_Enable(ACMP0) should NOT be called in order
	   * to ensure lower current consumption. */
}


/**************************************************************************//**
 * @brief  Setup the LESENSE
 *****************************************************************************/
void setupLESENSE(void)
{
  state_light = 0;
  /* LESENSE configuration structure */
  static const LESENSE_Init_TypeDef initLesense =
  {
    .coreCtrl         =
    { /* LESENSE configured for periodic scan. */
      .scanStart    = lesenseScanStartPeriodic,
      .prsSel       = lesensePRSCh0,
      .scanConfSel  = lesenseScanConfDirMap,
      .invACMP0     = false,
      .invACMP1     = false,
      .dualSample   = false,
      .storeScanRes = false,
      .bufOverWr    = true,
      .bufTrigLevel = lesenseBufTrigHalf,
      .wakeupOnDMA  = lesenseDMAWakeUpDisable,
      .biasMode     = lesenseBiasModeDutyCycle, /* Lesense should duty cycle comparator and related references etc. */
      .debugRun     = false
    },

    .timeCtrl         =
    {
      .startDelay     = 0 /* No start delay needed for this application. */
    },

    .perCtrl          =
    {  /* DAC is not needed for this application. */
      .dacCh0Data     = lesenseDACIfData,
      .dacCh0ConvMode = lesenseDACConvModeDisable,
      .dacCh0OutMode  = lesenseDACOutModeDisable,
      .dacCh1Data     = lesenseDACIfData,
      .dacCh1ConvMode = lesenseDACConvModeDisable,
      .dacCh1OutMode  = lesenseDACOutModeDisable,
      .dacPresc       = 0,
      .dacRef         = lesenseDACRefBandGap,
      .acmp0Mode      = lesenseACMPModeMuxThres, /* Allow LESENSE to control ACMP mux and reference threshold. */
      .acmp1Mode      = lesenseACMPModeMuxThres,
      .warmupMode     = lesenseWarmupModeNormal  /* Normal mode means LESENSE is allowed to dutycycle comparator and reference. */
    },

    .decCtrl          =
    { /* Decoder or statemachine not used in this code example. */
      .decInput  = lesenseDecInputSensorSt,
      .initState = 0,
      .chkState  = false,
      .intMap    = true,
      .hystPRS0  = false,
      .hystPRS1  = false,
      .hystPRS2  = false,
      .hystIRQ   = false,
      .prsCount  = true,
      .prsChSel0 = lesensePRSCh0,
      .prsChSel1 = lesensePRSCh1,
      .prsChSel2 = lesensePRSCh2,
      .prsChSel3 = lesensePRSCh3
    }
  };

  /* Channel configuration */
  /* Only one channel is configured for the lightsense application. */


  /* Alternate excitation channels configuration. */
  /* The lightsensor is excited by alternate excite channel 0. */
  static const LESENSE_ConfAltEx_TypeDef initAltEx =
  {
    .altExMap = lesenseAltExMapALTEX,
    .AltEx[0] =
    {
      .enablePin = true,
      .idleConf  = lesenseAltExPinIdleDis,
      .alwaysEx  = true
    },
    .AltEx[1] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF,
    .AltEx[2] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF,
    .AltEx[3] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF,
    .AltEx[4] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF,
    .AltEx[5] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF,
    .AltEx[6] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF,
    .AltEx[7] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF
  };

  /* Initialize LESENSE interface _with_ RESET. */
  LESENSE_Init(&initLesense, true);

  /* Configure LESENSE channel */
  LESENSE_ChannelConfig(&initLesenseCh, LIGHTSENSE_CH);

  /* Configure alternate excitation channels */
  LESENSE_AltExConfig(&initAltEx);

  /* Set scan frequency */
  LESENSE_ScanFreqSet(0, LCSENSE_SCAN_FREQ);

  /* Set clock divisor for LF clock. */
  LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_2);

  /* Enable interrupt in NVIC. */
  NVIC_EnableIRQ(LESENSE_IRQn);

  /* Start scan. */
  LESENSE_ScanStart();
}




void call_LESENSE_setup(void)
{
	  /* Disable global interrupts */
	  INT_Disable();

	  /* Enable clocks for used peripherals */
	  setupCMU();

	  /* Setup the ACMP */
	  setupACMP();

	  /* Setup the GPIO */
	  setupGPIO();

	  /* Setup the RTC */
	  //setupRTC();

	  /* setup lesense */
	  setupLESENSE();

	  /* Enable global interrupts */
	  INT_Enable();
}


