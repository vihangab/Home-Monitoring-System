/*************************************************************

  Sleep Routine which decides which energy mode will CPU enter
  Input Variables - None
  Global Variables - sleep_block_counter
  Return Variables - None
  Sleep routine written based on example from @section License
 <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>

  *************************************************************/
#include "em_device.h"
#include "em_chip.h"
#include "stdint.h"
#include "stdbool.h"
#include "em_int.h"
#include "em_emu.h"
#include "em_cmu.h"

int sleep_block_counter[4]={0,0,0,0}; 					/*Array to set flags for selected energy mode inside the sleep routine*/
