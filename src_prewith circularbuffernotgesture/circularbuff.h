/*
 * Name: Vihanga Bare
   MCIOT : Bonus Assignment- Circular Buffer
   Date: April 4, 2017
 */

#ifndef SRC_CIRCULARBUFF_H_
#define SRC_CIRCULARBUFF_H_

#include"main.h"


/*Circular buffer structure*/

typedef struct circular_buffer
{
	uint8_t *bufferpointer; //buffer pointer that points to start of buffer
	uint8_t *head;     // head of data buffer
    uint8_t *tail; // end of data buffer
    uint8_t count; //no of elements added in buffer
    uint8_t length;

}circular_buffer;

/*Buffer state structure*/

typedef enum buffer_states
{
	BUFFER_FULL,
    BUFFER_EMPTY,
    AVAILABLE
}cbuff_state;

uint8_t  length_buff;
/*Structure variable for transmitter circular buffer*/

circular_buffer cb;

/*Variable which states the length of buffer*/



/*Circular buffer functions declarations*/

/*****************************************************************************
* This function is used to initialize circular buffer
* It makes head, tail, buffer point to same location and make count 0
*****************************************************************************/

void cb_init(circular_buffer *cb,uint8_t length_buff);
void cb_free(circular_buffer *cb);
void cbuffer_add(circular_buffer *cb,uint8_t data,uint8_t values_add);
uint8_t cbuffer_remove(circular_buffer *cb,uint8_t values_remove);

#endif
