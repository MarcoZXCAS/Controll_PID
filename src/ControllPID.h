#ifndef ControllPID_h
#define ControllPID_h

#include "Arduino.h"

//Definizione della struttura dei dati
struct PIDController{
    double Kp, Ki, Kd;
    double integral;
    double prevInput;
    unsigned long prevTime;
    double outMin, outMax;
    bool firstRun;
};

void pidInit(PIDController &pid, double Kp, double Ki, double Kd);
void pidSetLimits(PIDController &pid, double minVal, double maxVal);
double pidCompute(PIDController &pid, double input, double setpoint);
void pidReset(PIDController &pid);

#endif