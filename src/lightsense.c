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
#include "lightsense_conf.h"

/* LESENSE channel configuration constant table. */
   LESENSE_ChAll_TypeDef initChs = LESENSE_LIGHTSENSE_SCAN_CONF;
  /* LESENSE alternate excitation channel configuration constant table. */
   LESENSE_ConfAltEx_TypeDef initAltEx = LESENSE_LIGHTSENSE_ALTEX_CONF;
  /* LESENSE central configuration constant table. */
   LESENSE_Init_TypeDef initLESENSE =
  {
    .coreCtrl =
    {
      .scanStart = lesenseScanStartPeriodic,
      .prsSel = lesensePRSCh0,
      .scanConfSel = lesenseScanConfDirMap,
      .invACMP0 = false,
      .invACMP1 = false,
      .dualSample = false,
      .storeScanRes = false,
      .bufOverWr = true,
      .bufTrigLevel = lesenseBufTrigHalf,
      .wakeupOnDMA = lesenseDMAWakeUpDisable,
      .biasMode = lesenseBiasModeDutyCycle,
      .debugRun = false
    },

    .timeCtrl =
    {
      .startDelay = 0U
    },

    .perCtrl =
    {
      .dacCh0Data = lesenseACMPThres,
      .dacCh0ConvMode = lesenseDACConvModeDisable,
      .dacCh0OutMode = lesenseDACOutModeDisable,
      .dacCh1Data = lesenseACMPThres,
      .dacCh1ConvMode = lesenseDACConvModeDisable,
      .dacCh1OutMode = lesenseDACOutModeDisable,
      .dacPresc = 0U,
      .dacRef = lesenseDACRefVdd,
      .acmp0Mode = lesenseACMPModeDisable,
      .acmp1Mode = lesenseACMPModeDisable,
      .warmupMode = lesenseWarmupModeNormal
    },

    .decCtrl =
    {
      .decInput = lesenseDecInputSensorSt,
      .initState = 0U,
      .chkState = false,
      .intMap = true,
      .hystPRS0 = false,
      .hystPRS1 = false,
      .hystPRS2 = false,
      .hystIRQ = false,
      .prsCount = true,
      .prsChSel0 = lesensePRSCh0,
      .prsChSel1 = lesensePRSCh1,
      .prsChSel2 = lesensePRSCh2,
      .prsChSel3 = lesensePRSCh3
    }
  };

/**************************************************************************//**
 * @brief  Setup the CMU
 *****************************************************************************/
void setupCMU(void)
{
  /* Ensure core frequency has been updated */
  //SystemCoreClockUpdate();

  /* Select clock source for HF clock. */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
  /* Select clock source for LFA clock. */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  /* Disable clock source for LFB clock. */
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);

  /* Enable HF peripheral clock. */
  CMU_ClockEnable(cmuClock_HFPER, true);
  /* Enable clock for GPIO. */
  CMU_ClockEnable(cmuClock_GPIO, true);
  /* Enable clock for LCD. */
  /* Enable clock for ACMP0. */
  CMU_ClockEnable(cmuClock_ACMP0, true);
  /* Enable clock for PRS. */
  CMU_ClockEnable(cmuClock_PRS, true);
  /* Enable CORELE clock. */
  CMU_ClockEnable(cmuClock_CORELE, true);
  /* Enable clock for PCNT. */
  CMU_ClockEnable(cmuClock_PCNT0, true);
  /* Enable clock on RTC. */
  //CMU_ClockEnable(cmuClock_RTC, true);
  /* Enable clock for LESENSE. */
  CMU_ClockEnable(cmuClock_LESENSE, true);
  /* Enable clock divider for LESENSE. */
  CMU_ClockDivSet(cmuClock_LESENSE, cmuClkDiv_1);
  /* Enable clock divider for RTC. */
  //CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_32768);
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

  /*GPIO setup for LED 1 pin*/
  GPIO_PinModeSet(LED0_PORT,LED0_PIN, gpioModePushPull,0);

}


/**************************************************************************//**
 * @brief  Setup the ACMP
 *****************************************************************************/
void setupACMP(void)
{
  /* ACMP configuration constant table. */
  static const ACMP_Init_TypeDef initACMP =
  {
    .fullBias = false,                 /* fullBias */
    .halfBias = true,                  /* halfBias */
    .biasProg =  0x0,                  /* biasProg */
    .interruptOnFallingEdge =  true,  /* interrupt on rising edge NA as lesense controls this*/
    .interruptOnRisingEdge =  false,   /* interrupt on falling edge NA as lesense controls this*/
    .warmTime = acmpWarmTime512,       /* 512 cycle warmup to be safe */
    .hysteresisLevel = acmpHysteresisLevel5, /* hysteresis level 5 */
    .inactiveValue = false,            /* inactive value */
    .lowPowerReferenceEnabled = true, /* low power reference */
    .vddLevel = 0x02,                  /* VDD level NA as lesense controls this*/
    .enable = false                    /* Don't request enabling. lesense controls this*/
  };

  /* Configure ACMP. */
    ACMP_Init(ACMP0, &initACMP);
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
  //excite();
	/* Initialize LESENSE interface with RESET. */
  LESENSE_Init(&initLESENSE, true);

  /* Configure scan channels. */
  LESENSE_ChannelAllConfig(&initChs);

  /* Configure alternate excitation channels. */
  LESENSE_AltExConfig(&initAltEx);

  /* Set scan frequency (in Hz). */
  (void)LESENSE_ScanFreqSet(0U, 20U);

  /* Set clock divisor for LF clock. */
  LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_1);

  /* Start scanning LESENSE channels. */
  LESENSE_ScanStart();
}


/**************************************************************************//**
 * @brief  Setup the PRS
 *****************************************************************************/
void setupPRS(void)
{
  /* Use PRS location 0 and output PRS channel 0 on GPIO PORTA0. */
  PRS->ROUTE = 0x01U;

  /* PRS channel 0 configuration. */
  PRS_SourceAsyncSignalSet(0U,
                           PRS_CH_CTRL_SOURCESEL_LESENSEL,
                           PRS_CH_CTRL_SIGSEL_LESENSESCANRES6);
}


/**************************************************************************//**
 * @brief  LESENSE interrupt handler
 *****************************************************************************/
void LESENSE_IRQHandler(void)
{
	unblock_sleep_mode(EM1);
  /* Negative edge interrupt on LESENSE CH6. */
  if (LESENSE_IF_CH6 & LESENSE_IntGetEnabled())
  {
    LESENSE_IntClear(LESENSE_IF_CH6);
  }

  if(state_light==0)
  {

	  GPIO_PinOutSet(LED0_PORT,LED0_PIN);
	  //LESENSE_ChAll_TypeDef initChs = LESENSE_LIGHTSENSE_SCAN_CONF1;
	  //LESENSE_Init(&initLESENSE, true);
	  /* Configure scan channels. */
	  //LESENSE_ChannelAllConfig(&initChs);
	  //LESENSE_ScanStart();
	  //state_light = 1;
  }
  else
  {
	  GPIO_PinOutClear(LED0_PORT,LED0_PIN);
	  LESENSE_ChAll_TypeDef initChs = LESENSE_LIGHTSENSE_SCAN_CONF;
	  LESENSE_Init(&initLESENSE, true);
	  /* Configure scan channels. */
	  LESENSE_ChannelAllConfig(&initChs);
	  LESENSE_ScanStart();
	  state_light = 0;
  }
  //clear();
}

void setupPCNT(void)
{
  /* PCNT configuration constant table. */
  static const PCNT_Init_TypeDef initPCNT =
  {
    .mode = pcntModeOvsSingle, /* Oversampling, single mode. */
    .counter = 0U, /* Counter value has been initialized to 0. */
    .top = 1, /* Counter top value. */
    .negEdge = false, /* Use positive edge. */
    .countDown = false, /* Up-counting. */
    .filter = false, /* Filter disabled. */
    .hyst = false, /* Hysteresis disabled. */
    .s1CntDir = false, /* Counter direction is given by CNTDIR. */
    .cntEvent = pcntCntEventUp, /* Regular counter counts up on upcount events. */
    .auxCntEvent = pcntCntEventNone, /* Auxiliary counter doesn't respond to events. */
    .s0PRS = pcntPRSCh0, /* PRS channel 0 selected as S0IN. */
    .s1PRS = pcntPRSCh0  /* PRS channel 0 selected as S1IN. */
  };


  /* Initialize PCNT. */
  PCNT_Init(PCNT0, &initPCNT);
  /* Enable PRS input S0 in PCNT. */
  PCNT_PRSInputEnable(PCNT0, pcntPRSInputS0, true);

  /* Enable the PCNT peripheral. */
  PCNT_Enable(PCNT0, pcntModeOvsSingle);
  /* Enable the PCNT overflow interrupt. */
  PCNT_IntEnable(PCNT0, PCNT_IEN_OF);
}

void PCNT0_IRQHandler(void)
{
	/* Overflow interrupt on PCNT0. */
	PCNT_IntClear(PCNT0, PCNT_IF_OF);
}

void call_LESENSE_setup(void)
{
   /* Setup CMU. */
  setupCMU();
  /* Setup GPIO. */
  setupGPIO();
  /* Setup ACMP. */
  setupACMP();
  /* Setup PCNT */
  setupPCNT();
  /* Setup PRS. */
  setupPRS();
  /* Setup LESENSE. */
  setupLESENSE();

  /* Enable PCNT0 interrupt in NVIC. */
  NVIC_EnableIRQ(PCNT0_IRQn);
  /* Enable interrupt in NVIC. */
  NVIC_EnableIRQ(LESENSE_IRQn);
}


