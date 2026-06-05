#include <ControlloPID.h>
PIDController pid;

double setpoint = 200.0;
double input = 0.0;

const int motorPin = 9; // Pin PWM per il motore

void setup() {
  Serial.begin(9600);

  pidInit(pid, 0.11, 0.2, 0.12);
  pidSetLimits(pid, 0, 256);

  pinMode(motorPin, OUTPUT);
}

void loop() {
  double output = pidCompute(pid, input, setpoint);

  Serial.print("output: ");
  Serial.println(output);

  analogWrite(motorPin, output);

  delay(1);
}