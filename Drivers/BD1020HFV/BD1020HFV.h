#ifndef INC_DRIVERS_BD1020HFV_H_
#define INC_DRIVERS_BD1020HFV_H_


#include <stdint.h>


/* --------------- Datasheet -----------------
 * Accuracy = ±1.5°C @ Ta = 30°C
 * 			= ±2.5°C @ Ta = -30°C, +100°C
 * Temp Sens = -8.2mV/°C
 * Detection Temp Range = -30°C to +100°C
 */

#define REF_TEMP	(float)30
#define VREF		(float)3.3
#define RES			(float)4095.0
#define V30 		(float)(RES/VREF)*1.3
#define DELTA_1C	(float)((RES*0.0082)/VREF)


float get_temperature(uint16_t adcValue);

#endif /* INC_DRIVERS_BD1020HFV_H_ */
