#include ""ControllPID.h"

void pidInit(PIDController &pid, double Kp, double Ki, double Kd) {
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    pid.integral = 0.0;
    pid.prevInput = 0.0;
    pid.prevTime = millis();
    pid.outMin = -255.0; // Default limits
    pid.outMax = 255.0;
    pid.firstRun = true;
}

void pidSetLimits(PIDController &pid, double minVal, double maxVal) {
    if (minVal >= maxVal) return; // Invalid limits
    pid.outMin = minVal;
    pid.outMax = maxVal;
    // Ensure integral term is within new limits
    pid.integral = constrain(pid.integral, minVal, maxVal);
}

double pidCompute(PIDController &pid, double input, double setpoint) {
  unsigned long now = millis();

  if (pid.firstRun) {
    pid.prevInput = input;
    pid.prevTime  = now;
    pid.firstRun  = false;
    return 0;
  }

  double dt = (now - pid.prevTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  pid.prevTime = now;

  double error = setpoint - input;

  double P = pid.Kp * error;

  double newIntegral = pid.integral + error * dt;
  double I = pid.Ki * newIntegral;
  if      (I > pid.outMax) I = pid.outMax;
  else if (I < pid.outMin) I = pid.outMin;
  pid.integral = (pid.Ki != 0) ? I / pid.Ki : newIntegral;

  double dInput = (input - pid.prevInput) / dt;
  double D      = -pid.Kd * dInput;
  pid.prevInput = input;

  return constrain(P + I + D, pid.outMin, pid.outMax);
}

void pidReset(PIDController &pid) {
    pid.integral = 0;
    pid.prevInput = 0;
    pid.prevTime = millis();
    pid.firstRun = true;
}