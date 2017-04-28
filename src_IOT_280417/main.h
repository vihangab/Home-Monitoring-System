/*
 * Name: Vihanga Bare
   MCIOT : Bonus Assignment- Circular Buffer
   Date: April 4, 2017
 */

#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_

#include "em_device.h"
#include "em_chip.h"
#include "stdint.h"
#include "stdbool.h"
#include "em_int.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_letimer.h"
#include "em_timer.h"
#include "em_adc.h"
#include "em_dma.h"
#include "em_acmp.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "em_leuart.h"
#include "em_core.h"
#include "em_pcnt.h"
#include "em_prs.h"
#include "dmactrl.c"
#include "em_assert.h"
#include "circularbuff.h"
#include "gesturei2c.h"
#include "string.h"
#include "leuartdma.h"
#include "capsense.h"
#include "em_lesense.h"
#include "em_rtc.h"
//#include "em_rtc.h"
#include "em_dac.h"

#define EM0 0
#define EM1 1
#define EM2 2
#define EM3 3
#define EM_RESTORE 	1
#define TIME_PERIOD 20 					// TO excite ambient light sensor every TIME_PERIOD seconds
#define ON_TIME 	0.004  						// time to excite light sensor and get voltage reading - 4 msec
#define ENABLE_CALLIBRATION 1

#define LFXO_MAX_COUNT 		32769
#define ULFRCO_MAX_COUNT 	1001
#define TIMERn_MAX_COUNT 	65535
#define LETIMER_MAX_COUNT 	65535
#define LED1_PORT 			gpioPortE
#define LED1_PIN 			3
#define LED0_PORT 			gpioPortE
#define LED0_PIN 			2
#define LIGHTSENSE_EXCITE_PORT 	gpioPortD
#define LIGHTSENSE_EXCITE_PIN 	6U
#define LIGHTSENSE_SENSOR_PORT 	gpioPortC
#define LIGHTSENSE_SENSOR_PIN 	6U
#define VDD_Level_Dark 			2
#define VDD_Level_Light 		61
#define ACMP_LIGHT_SENSE 		acmpChannel6
#define ACMP_VDD 				acmpChannelVDD
#define SLEEP_MODE_LIMIT 		EM2				//limit the mode to which CPU can sleep based on peripheral

#define ACMP_WARMUP_TIME 		acmpWarmTime256
#define ACMP_HYSTERESIS 		acmpHysteresisLevel4


/* ADC Transfer Data */

#define ADCSAMPLES                  500
volatile uint16_t 					ramBufferAdcData[ADCSAMPLES];
#define DMA_CHANNEL_ADC       		0
//#define LOWER_TEMP					15
//#define UPPER_TEMP					20
#define DMA_off						//uncomment this to switch off the DMA for ADC
#define ADC_Freq					1280000
#define HFPER_Freq					0			//set to 0 to use default HFPER clock frequency i.e 14 Mhz as timebase
//#define ADC_Poll					//uncomment this to enable ADC Polling

#define ADC_EM						1						//EM1
#define ADC_OSR_RATE				adcOvsRateSel2
#define ADC_FILTERMODE				adcLPFilterBypass
#define ADC_WARMUP					adcWarmupNormal
#define ADC_PRSTrigger				adcPRSSELCh0
#define ADC_AcqTime					adcAcqTime1
#define ADC_VREF					adcRef1V25
#define ADC_Resolution				adcRes12Bit
#define ADC_Mode					adcStartSingle

#define DMA_SRC_INC 				dmaDataIncNone
#define DMA_DST_INC 				dmaDataInc2
#define DMA_DATA_SIZE 				dmaDataSize2
#define DMA_ARBITRATE 				dmaArbitrate1
#define ENERGYMODE_LEUART			EM2



// TSL2651 threshold values, ADDRESS FOR REGISTERS ON TSL2651

/*#define TSL2561_THRESHighReg 		 0x0800
#define TSL2561_THRESLowReg 		 0x000f
#define TSL2561_CONTROL_REG          0x80
#define TSL2561_TIMING_REG			 0x81
#define TSL2561_THRESLowLow_REG      0x82
#define TSL2561_THRESLowhigh_REG     0x83
#define TSL2561_THRESHighLow_REG     0x84
#define TSL2561_THRESHighHigh_REG    0x85
#define TSL2561_INTERRUPT_REG        0x86

#define TSL2561_DATA0Low			 0x8c
#define TSL2561_DATA0High			 0x8d
#define TSL2561_DATA1Low			 0x8e
#define TSL2561_DATA1High			 0x8f*/
#define LEUART_LOCATION    			0
#define LEUART0_GPIO    			gpioPortD
#define LEUART0_TX_PIN  			4
#define LEUART0_RX_PIN  			5
//#define LEUART_ENABLE				1
#define LEUART_INIT_STATUS			leuartDisable
#define LEUART_BAUD_RATE			9600
#define LEUART_FRAME_BITS			leuartDatabits8
#define LEUART_PARITY				leuartNoParity
#define LEUART_STOPBIT				leuartStopbits1
#define CIRCULAR_BUFFER_TEST		0
#define DMA_CHANNEL_LEUART0			0

#define DEBUG
#define LEUART_DMA

/* Bitmask for the currently touched channels */
  uint16_t channels_touched;

#define CIRCULAR_BUFFER
int state_light;
//#define ONBOARD_LIGHT_SENSOR
//#define WIFI

int temp_low;
int temp_high;

uint8_t debug_array[50];
uint8_t debug_array1[20];
uint8_t state;

uint8_t bytes_to_transfer;
uint8_t intparttosend;
uint8_t floatparttosend;
uint8_t arraytosend[6];
uint8_t dummyarray[6];


/* ACMP */
#define ACMP_NEG_REF           		acmpChannelVDD
#define ACMP_THRESHOLD_LOW         (0x35)                         // Reference value for the lightsensor.
#define ACMP_THRESHOLD_HIGH        (0x3D)                           /*   * Value works well in office light
                                                          * conditions. Might need adjustment
                                                             * for other conditions. */

/* GPIO */
#define LED_GPIO_PORT gpioPortE
#define LED_GPIO_PIN  2
	/* LESENSE */
#define LIGHTSENSE_CH             6
#define LIGHTSENSE_EXCITE_PORT    gpioPortD
#define LIGHTSENSE_EXCITE_PIN     6
#define LIGHTSENSE_SENSOR_PORT    gpioPortC
#define LIGHTSENSE_SENSOR_PIN     6
#define LCSENSE_SCAN_FREQ         20
#define LIGHTSENSE_INTERRUPT      LESENSE_IF_CH6


#endif



