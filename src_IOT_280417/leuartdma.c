#include "leuartdma.h"

char ssid[50]="G4 7896";
char password[20]="mayflower";




/**************************************************************************//**
 * @brief  Setting up LEUART
 *****************************************************************************/
void setupLeuart(void)
{
  /* Enable peripheral clocks */
  CMU_ClockEnable(cmuClock_HFPER, true);
  /* Configure GPIO pins */
  CMU_ClockEnable(cmuClock_GPIO, true);
  /* To avoid false start, configure output as high */
  GPIO_PinModeSet(LEUART0_GPIO, LEUART0_TX_PIN, gpioModePushPull, 1);
  GPIO_PinModeSet(LEUART0_GPIO, LEUART0_RX_PIN, gpioModeInput, 0);
  LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;

  /* Enable CORE LE clock in order to access LE modules */
  CMU_ClockEnable(cmuClock_CORELE, true);
  /* Select LFXO for LEUARTs (and wait for it to stabilize) */
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_LEUART0, true);
  /* Do not prescale clock */
  CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1);
  /* Configure LEUART */
  init.enable = leuartDisable;
  LEUART_Init(LEUART0, &init);

  /* Enable pins at default location */
  LEUART0->ROUTE = LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN | LEUART_LOCATION;

  /* Set RXDMAWU to wake up the DMA controller in EM2 */
  LEUART_TxDmaInEM2Enable(LEUART0, true);
	//LEUART0->CTRL |= LEUART_CTRL_LOOPBK;
	//LEUART0->CTRL |= LEUART_CTRL_AUTOTRI;


  /* Finally enable it */
  LEUART_Enable(LEUART0, leuartEnable);

   	LEUART0->IEN |= LEUART_IEN_RXDATAV;
    NVIC_EnableIRQ(LEUART0_IRQn);

}
/**************************************************************************//**
 * @brief  Setup DMA
 *
 * @details
 *   This function initializes DMA controller.
 *   It configures the DMA channel to be used for LEUART0 transmit
 *   and receive. The primary descriptor for channel0 is configured for
 *   a single data byte transfer. For continous data reception and transmission
 *   using LEUART DMA loopmode is enabled for channel0.
 *   In the end DMA transfer cycle is configured to basicMode where
 *   the channel source address, destination address, and
 *   the transfercount per dma cycle have been specified.
 *
 *****************************************************************************/
void setupDma(void)
{
	/* DMA configuration structs */
	DMA_Init_TypeDef       dmaInit;
	DMA_CfgChannel_TypeDef channelCfg;
	DMA_CfgDescr_TypeDef   descrCfg;
	//DMA_CfgLoop_TypeDef    loopCfg;

  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
  /* Setting up channel */
  channelCfg.highPri   = false; /* Can't use with peripherals */
  channelCfg.enableInt = false; /* Interrupt not needed in loop transfer mode */
  /* Configure channel 0 */
  /*Setting up DMA transfer trigger request*/
  channelCfg.select = DMAREQ_LEUART0_TXBL;
  channelCfg.cb     = NULL;
  DMA_CfgChannel(0, &channelCfg);

  /* Setting up channel descriptor */
  /* Destination is LEUART_Tx register and doesn't move */
  descrCfg.dstInc = dmaDataIncNone;
  /* Source is LEUART_RX register and transfers 8 bits each time */
  descrCfg.srcInc = dmaDataInc1;
  descrCfg.size   = dmaDataSize1;
  /* We have time to arbitrate again for each sample */
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  /* Configure primary descriptor  */
  DMA_CfgDescr(0, true, &descrCfg);

  /* Configure loop transfer mode */
  //loopCfg.enable = true;
  //loopCfg.nMinus1 = 5;  /* Single transfer per DMA cycle*/
  //DMA_CfgLoop(0, &loopCfg);

}

void WIFI_Connect(void)
{
	Wifi_AT();
	Wifi_Reset();
	Wifi_SetMode();
	//Wifi_GetList();
	Wifi_Join();
	Wifi_SetMultiple();
}

void Wifi_AT()
{
	strcpy(command,"AT\r\n");
	command_size = strlen(command);
	Send_Command();
	wait();
}

void Wifi_Reset()
{
	strcpy(command,"AT+RST\r\n");
	command_size = strlen(command);
	Send_Command();
	wait();
}

void Wifi_SetMode()
{
	strcpy(command,"AT+CWMODE=3\r\n");
	command_size = strlen(command);
	Send_Command();
	wait();
}

void Wifi_GetList()
{
	strcpy(command,"AT+CWLAP\r\n");
	command_size = strlen(command);
	Send_Command();
	wait();
}

void Wifi_Join()
{
	strcpy(command,"AT+CWJAP=\"");
    strcat(command,ssid);
    strcat(command,"\",\"");
    strcat(command,password);
    strcat(command,"\"\r\n");
    command_size = strlen(command);
    Send_Command();
    wait();
    wait();
    wait();
}

void Wifi_SetMultiple()
{
	strcpy(command,"AT+CIPMUX=1\r\n");
    command_size = strlen(command);
    Send_Command();
    wait();
}

void Send_Command()
{
      /* Activate basic dma cycle using channel0 */
      DMA_ActivateBasic(0,
                true,
                false,
                (void *)&LEUART0->TXDATA,
                (void *)&command,
                command_size-1);
    LEUART0->TXDATA = 1;
}
void Send_Values()
{
	  for(int i=0;i<500;i++);

	/* Activate basic dma cycle using channel0 */
      DMA_ActivateBasic(0,
                true,
                false,
                (void *)&LEUART0->TXDATA,
                (void *)&arraytosend,
                (length_buff-1));
    LEUART0->TXDATA = 1;
}


void wait()
{
    for(int i=0;i<2000000;i++);
}
