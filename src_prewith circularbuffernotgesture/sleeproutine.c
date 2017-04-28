
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
#include "main.h"
#include "sleeproutine.h"


void sleep(void)
{
	if(sleep_block_counter[EM0] > 0)
	{
		return;
	}
	else if(sleep_block_counter[EM1] > 0)
	{
		EMU_EnterEM1();
	}
	else if(sleep_block_counter[EM2] > 0)
	{
		EMU_EnterEM2(EM_RESTORE);
	}
	else if(sleep_block_counter[EM3] > 0)
	{
		EMU_EnterEM3(EM_RESTORE);
	}
	else
	{
		EMU_EnterEM3(EM_RESTORE);
	}
	return;
}

/*************************************************************
 * BLock and Unblock Sleep Routine
 * Input Variables - int energy_mode
 * Global Variables - sleep_block_counter
 * Return Variables - None
 *  Sleep routine written based on example from @section License
    <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *************************************************************/

void set_sleep_mode(int energy_mode)
{
	INT_Disable();
	sleep_block_counter[energy_mode]++;
	INT_Enable();
}

/*************************************************************
 * Removing the restriction of the block sleep mode
 * when the peripheral is no longer required
 *************************************************************/

void unblock_sleep_mode(int energy_mode)
{
       INT_Disable();
       sleep_block_counter[energy_mode]--;
       INT_Enable();
}
