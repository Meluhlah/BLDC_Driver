#ifndef __APP_H
#define __APP_H

#include "usart.h"
#include "tim.h"
#include "bldc.h"
#include <string.h>
#include "adc.h"
#include "PID.h"


#define ADC_CHANNELS    				(uint8_t)11

// --------------------------- GUI Commands ------------------------- //
typedef enum {
    CMD_MOTOR_INIT,
    CMD_MOTOR_ALIGN,
    CMD_MOTOR_START,
    CMD_MOTOR_STOP,
    CMD_SET_MOTOR_SPEED,
    CMD_SET_MOTOR_DIRECTION,
}UartCommands_e;


typedef struct __attribute__((__packed__)){
    UartCommands_e command; // This will be treated as uint8_t (1 byte)
    uint16_t data;          // 2 bytes
} UartBufferRx_t;

typedef UartBufferRx_t UartBufferRx_t;


typedef struct UartBufferTx_t{

	uint16_t phasesV[3];
	uint16_t phasesI[3];
	uint16_t vinRef;
	uint16_t temperature[3];
	uint16_t potValue;
	uint16_t pwmDutyCycle;
	uint16_t rpm;
	uint16_t rotationDirection;
	uint32_t checkSum;

} UartBufferTx_t;

extern UartBufferTx_t uartBufferTx;
extern UartBufferRx_t uartBufferRx;

extern volatile uint16_t adc_buffer[ADC_CHANNELS];


void run_app();
void guiDataTransmit(volatile BldcHandler_t* pDriver);


#endif	// __APP_H
