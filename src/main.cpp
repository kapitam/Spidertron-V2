#include <Arduino.h>

#include "body.h"
#include "config.h"
#include "gait.h"
#include "psx_input.h"

// Layering, top to bottom:
//   input (PsxInput)  ->  planner (TripodGait)  ->  body (IK + limits)
//                                                     -> servo_bus (PCA9685)
// To drive this from ROS later, replace PsxInput/TripodGait with a node that
// sets body.foot(leg) targets, checks body.poseReachable(), and calls
// body.commit().

HexapodBody body;
TripodGait gait(body);
PsxInput input;

void setup() {
  Serial.begin(115200);

  input.begin();
  body.begin();

  gait.rest();
}

void loop() {
  MoveCommand cmd = input.readMove();

  if (cmd.active) {
    gait.walk(cmd.headingRad, cmd.magnitude);
  } else {
    gait.rest();
  }
}
