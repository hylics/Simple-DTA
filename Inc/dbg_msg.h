/*

*/

#ifndef _DBG_MSG_H
#define _DBG_MSG_H

#ifndef NDEBUG
#include <stdint.h>

//debug uart messages
const uint8_t err1[]="queue handle = null\n";
const uint8_t err2[]="result data not received from queue\n";
const uint8_t msg1[]="adc_task\n";
const uint8_t msg2[]="uart_task\n";
	
#endif

#endif //_DBG_MSG_H
