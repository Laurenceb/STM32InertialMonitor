#include <stdlib.h>
#include "buffer.h"

void Add_To_Buffer(uint16_t data,buff_type* buffer) {
	buffer->data[buffer->head++]=data;//Put data in and increment
	buffer->head%=buffer->size;
	if(buffer->head==buffer->tail)	//Buffer wraparound due to filling
		buffer->tail=(buffer->tail+1)%buffer->size;
}

uint8_t Get_From_Buffer(uint16_t* data,buff_type* buffer) {
	if(buffer->tail==buffer->head) {
		*data=0;		//Set the data to zero if this happens
		return 1;		//Error - no data in buffer
	}
	else {
		*data=buffer->data[buffer->tail];//grab a data sample from the buffer
		buffer->tail++;
		buffer->tail%=buffer->size;
		return 0;		//No error
	}
}

void Empty_Buffer(buff_type* buffer) {
	buffer->tail=buffer->head;	//Set the tail to the head, to show there is no usable data present
}

void init_buffer(buff_type* buff, uint16_t size) {
	buff->data=(uint16_t*)malloc(size*2);
	buff->size=size;
}

