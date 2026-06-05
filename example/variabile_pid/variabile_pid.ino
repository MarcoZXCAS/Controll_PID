#include <ControlloPID.h>
PIDController pid;

double setpoint = 200.0;
double input = 0.0;

void setup() {
  Serial.begin(9600);

  pidInit(pid, 0.11, 0.2, 0.12);
  pidSetLimits(pid, -100, 100);
}

void loop() {
  double output = pidCompute(pid, input, setpoint);

  Serial.print("output: ");
  Serial.println(output);

  delay(100);
}