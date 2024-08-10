#include "PID.h"


// #define PI_CONTROL
#define PID_CONTROL

void pidInit(PIDController* pid, float kp, float ki, float kd){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->limMax = 0.0f;
    pid->limMin = 0.0f;
    pid->differentiator = 0.0f;
    pid->integrator = 0.0f;
    pid->out = 0.0f;
    pid->prevError = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->T = SAMPLETIME;
    pid->tau = TAU;


    pid->prevError = 0.0;
}


float pidUpdate(PIDController* pid, float setpoint, float measurement){
     
     /* error term calculation*/
     float error = setpoint - measurement;

    /*  proportional term calculation*/
     float proportional = pid->kp * error;

     pid->integrator = pid->integrator + 0.5f * pid->ki * pid->T * (error + pid->prevError);
    
    /* Anti wind-up*/
    float limMinInt = 0;
    float limMaxInt = 0;

    if(pid->limMax < proportional)
        limMinInt = pid->limMin - proportional;
    else
        limMinInt = 0.0f;

    if (pid->integrator > limMaxInt)
        pid->integrator = limMaxInt;

    else if(pid->integrator < limMinInt)
        pid->integrator = limMinInt;

    pid->differentiator = ((2.0f * pid->kd) / (2.0f * pid->tau + pid->T)) * (measurement - pid->prevMeasurement)
                            + (2.0f * pid->tau - pid->T)/(2.0f * pid->tau + pid->T) * pid->differentiator;

    #ifdef PID_CONTROL
   
    pid->out = proportional + pid->integrator + pid->differentiator;

    #elif defined PI_CONTROL
    pid->out = proportional + pid->integrator;

    #endif

    if(pid->out > pid->limMax)
        pid->out = pid->limMax;

    else if(pid->out < pid->limMin)
        pid->out = pid->limMin;

    pid->prevError = error;

    pid->prevMeasurement = measurement;

    return pid->out;
}
