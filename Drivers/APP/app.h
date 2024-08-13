#ifndef __APP_H
#define __APP_H

#include "usart.h"
#include "tim.h"
#include "bldc.h"
#include <string.h>
#include "adc.h"
#include "PID.h"


#define ADC_CHANNELS    (uint8_t)10
#define UART_NUM_BYTES		(uint8_t)28


typedef struct TX_BUFFER{

	uint16_t phasesV[3];
	uint16_t phasesI[3];
	uint16_t vinRef;
	uint16_t temperature[3];
	uint16_t potValue;
	uint16_t pwmDutyCycle;
	uint16_t rpm;
	uint16_t rotationDirection;

} TX_BUFFER;

extern TX_BUFFER tx_buffer;
extern uint16_t dataToSend[UART_NUM_BYTES];
extern const uint16_t tx_buffer_size;
extern volatile uint16_t adc_buffer[ADC_CHANNELS];


void run_app();
void guiDataTransmit(volatile Bldc_t* pDriver);


#endif	// __APP_H
