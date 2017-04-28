/*
 * Name: Vihanga Bare
   MCIOT : Bonus Assignment- Circular Buffer
   Date: April 4, 2017
 */

#include "main.h"
//#include "circularbuff.h"

void cb_init(circular_buffer *cb, uint8_t size)
{
	(cb->bufferpointer) = (uint8_t *)malloc(sizeof(uint8_t)*length_buff);
	(cb->head)=(cb->bufferpointer);
	(cb->tail)=(cb->bufferpointer);
	(cb->count)=0;
	(cb->length)=size;

}

/******************************************************************************
* This function is used to free the heap memory allocated to circular buffer
* uint8_t *cbuffer - Pointer which points to circular buffer
******************************************************************************/
void cb_free(circular_buffer *cb)
{
	free(cb->bufferpointer);
}

/******************************************************************************
* This function is used to add elements to circular buffer
* The return value can be full or available
* circular_buffer *cb - Pointer which points to circular buffer
* uint8_t * data - Pointer which points to array which contains data
                   to be stored on buffer
uint8_t values_add - This variable has the number of values to be added to buffer
********************************************************************************/

void cbuffer_add(circular_buffer *cb,uint8_t data,uint8_t values_add)
{
	while(values_add)
	{
		/*Condition to check if buffer is not full*/

		if((cb->count)!=(cb->length))
		{
			/*Put the data at location pointed by head, then increment head pointer
			  and count value*/
			*(cb->head)= data;
			(cb->head)++;
			(cb->count)++;

			/*Wrap around, if head is pointing to last location of buffer*/
			if((cb->head)>((cb->bufferpointer)+(cb->length)-1))
			{
				(cb->head)=(cb->bufferpointer);
			}
		}
		//data++;
		values_add--;
	}
}

/*******************************************************************************
* This function is used to remove elements from circular buffer
* The return value can be empty or available depending on count value
* circular_buffer *cb - Pointer which points to circular buffer
* uint8_t values_remove - This variable gives the number of values which are
			  to be removed
*******************************************************************************/

uint8_t cbuffer_remove(circular_buffer *cb,uint8_t values_remove)
{
	uint8_t data;
	while(values_remove)
	{
		/*Count value checks if buffer is empty*/
		if((cb->count)!=0)
		{
			data=*(cb->tail);

			/*After removing the element from buffer, increment tail pointer
			  and decrement count value*/
			(cb->tail)++;
			(cb->count)--;

			/*Wrap around if tail points at last location*/
			if((cb->tail)>((cb->bufferpointer)+(cb->length)-1))
			{
				(cb->tail)=(cb->bufferpointer);
			}
		}
		values_remove--;
	}
	return data;
}
