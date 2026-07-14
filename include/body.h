#pragma once

#include "kinematics.h"
#include "servo_bus.h"

// The interface a motion planner talks to. A planner (the tripod gait today,
// a ROS node later) sets world-frame foot targets and calls commit(); the
// body handles IK and servo output and reports whether the pose was reachable.
class HexapodBody {
public:
  void begin();

  // World-frame target position of one foot (cm). Mutable so planners can
  // adjust targets incrementally.
  Vec3 &foot(int leg) { return feet[leg]; }

  // Solve IK for all feet and write the servos.
  // Returns false if any joint limit was hit (legs after the offending one
  // are not written, so the pose on the robot stays consistent).
  bool commit();

private:
  ServoBus servos;
  Vec3 feet[NUM_LEGS] = {};
};
