/*
 * capsense.h
 *
 *  Created on: Apr 24, 2017
 *      Author: Priyanka
 */

#ifndef SRC_CAPSENSE_H_
#define SRC_CAPSENSE_H_

#include "main.h"


/* LESENSE number of channels possible to use, should be 16 */
#define NUM_LESENSE_CHANNELS    16

/* GPIO Port for analog comparators */
#define LESENSE_CH_PORT         gpioPortC

#define RTC_FREQ	            32768
#define CALIBRATION_INTERVAL    2

/** Scan frequency for LESENSE, how often all the pads are scanned. */
#define LESENSE_SCAN_FREQUENCY          5

/** Sample delay, how long the rc-oscillations are sampled. */
#define SAMPLE_DELAY                   30

/** Validate count is the number of consecutive scan-cycles a button needs to */
/** be in the changed state before an actual button press or release is acknowledged. */
#define VALIDATE_CNT                   10

/** Number of calibration events used to calculate threshold. */
#define NUMBER_OF_CALIBRATION_VALUES    10

static volatile uint16_t calibration_value[NUM_LESENSE_CHANNELS][NUMBER_OF_CALIBRATION_VALUES];
static volatile uint16_t buttons_pressed;
static volatile uint16_t channel_max_value[NUM_LESENSE_CHANNELS];
static volatile uint16_t channel_min_value[NUM_LESENSE_CHANNELS];

static uint16_t channels_used_mask;
static uint8_t num_channels_used;
static float channel_threshold_percent[NUM_LESENSE_CHANNELS];


unsigned int test_flag;
unsigned int captouch_interrupt_flags;

//unsigned int temp = 0;

/* Function prototypes */
static void LETOUCH_setupACMP(void);
static void LETOUCH_setupLESENSE(void);
static void LETOUCH_setupGPIO(void);
static void LETOUCH_setupCMU(void);

static uint16_t GetMaxValue(volatile uint16_t* A, uint16_t N);
static uint16_t GetMinValue(volatile uint16_t* A, uint16_t N);

void LETOUCH_Init(float sensitivity[]);
uint16_t LETOUCH_GetChannelsTouched(void);
uint16_t LETOUCH_GetChannelMaxValue(uint8_t channel);
uint16_t LETOUCH_GetChannelMinValue(uint8_t channel);
void LETOUCH_Calibration(void);

void LETOUCH_setupRTC(void);
void RTC_IRQHandler(void);
void send_status(unsigned int channels);

#endif /* SRC_CAPSENSE_H_ */
