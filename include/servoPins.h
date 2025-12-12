// servoPins.h - declarations only. Definitions live in src/servoPins.cpp
#ifndef SERVO_PINS_H
#define SERVO_PINS_H

#include <Adafruit_PWMServoDriver.h>

// Assign Servo on PWM adafruit (pin numbers)
#define SERVO_PINFR1 0
#define SERVO_PINFR2 1
#define SERVO_PINFR3 2
#define SERVO_PINCR1 4
#define SERVO_PINCR2 5
#define SERVO_PINCR3 6
#define SERVO_PINBR1 8
#define SERVO_PINBR2 9
#define SERVO_PINBR3 10
#define SERVO_PINFL1 0
#define SERVO_PINFL2 1
#define SERVO_PINFL3 2
#define SERVO_PINCL1 4
#define SERVO_PINCL2 5
#define SERVO_PINCL3 6
#define SERVO_PINBL1 8
#define SERVO_PINBL2 9
#define SERVO_PINBL3 10

#define SDA_PIN 21  // ESP32 I2C SDA
#define SCL_PIN 22  // ESP32 I2C SCL

// Extern declarations for shared objects defined in a single .cpp
extern int minPulse;
extern int maxPulse;
extern Adafruit_PWMServoDriver pwm1;
extern Adafruit_PWMServoDriver pwm2;

#endif // SERVO_PINS_H