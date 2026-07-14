#pragma once

#include <Adafruit_PWMServoDriver.h>

#include "kinematics.h"

// Lowest layer: turns joint angles into PCA9685 pulses. Knows nothing about
// gaits or coordinates.
class ServoBus {
public:
  void begin();

  // Write one leg's three joint angles to its servos.
  void writeLeg(int leg, const JointAngles &j);

private:
  Adafruit_PWMServoDriver pwmRight{PCA_RIGHT_ADDR};  // legs 0-2
  Adafruit_PWMServoDriver pwmLeft{PCA_LEFT_ADDR};    // legs 3-5
};
