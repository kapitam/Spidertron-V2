#include "servo_bus.h"

#include <Wire.h>

namespace {

// Convert a servo angle (0-180 deg) to PCA9685 ticks.
uint16_t angleToTicks(double angleDeg) {
  double pulseUs = SERVO_MIN_PULSE_US +
                   (angleDeg / 180.0) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
  return pulseUs * 4096 / (1000000 / PWM_FREQ_HZ);
}

}  // namespace

void ServoBus::begin() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  pwmRight.begin();
  pwmRight.setPWMFreq(PWM_FREQ_HZ);
  pwmLeft.begin();
  pwmLeft.setPWMFreq(PWM_FREQ_HZ);
}

void ServoBus::writeLeg(int leg, const JointAngles &j) {
  // Legs 0-2 are on the right-side driver, 3-5 on the left-side driver.
  // Each leg occupies channels base+0..base+2, spaced 4 channels apart.
  Adafruit_PWMServoDriver &driver = (leg < 3) ? pwmRight : pwmLeft;
  uint8_t base = (leg % 3) * 4;

  // All three servos are mounted mirrored, hence the 180 - angle.
  driver.setPWM(base + 0, 0, angleToTicks(180 - j.coxa));
  driver.setPWM(base + 1, 0, angleToTicks(180 - j.femur));
  driver.setPWM(base + 2, 0, angleToTicks(180 - j.tibia));
}
