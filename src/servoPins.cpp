#include <Adafruit_PWMServoDriver.h>

// Definitions for the PWM drivers and pulse constants
int minPulse = 450;
int maxPulse = 2500;

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
