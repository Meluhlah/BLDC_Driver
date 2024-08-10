#ifndef __PI_CONTROLLER
#define __PI_CONTROLLER

#define SAMPLETIME  0.001   // Seconds
#define TAU         0.001   // Seconds

typedef struct{

    /*      Gains       */
    float kp;
    float ki;
    float kd;

    /*  Derivative LPF time constant*/
    float tau;

    /*  Sample Time (seconds)*/
    float T;

    float integrator;
    float prevError;        // Required for Integrator
    float differentiator;
    float prevMeasurement;  // Required for Differentiator

    /*  Anti-Windup */
    float limMax;
    float limMin;

    float out;              // Controller output



} PIDController;

void pidInit(PIDController* pid, float kp, float ki, float kd);
float pidUpdate(PIDController* pid, float setpoint, float measurement);





#endif // __PI_CONTROLLER