#pragma once

#include "kinematics.h"
#include "servo_bus.h"

// The interface a motion planner talks to. A planner (the tripod gait today,
// a ROS node later) sets world-frame foot targets, verifies the pose with
// poseReachable(), and applies it with commit().
class HexapodBody {
public:
  void begin();

  // World-frame target position of one foot (cm). Mutable so planners can
  // adjust targets incrementally.
  Vec3 &foot(int leg) { return feet[leg]; }

  // IK-check every current foot target without moving anything. Returns
  // false (and logs the offending leg) if any joint limit would be hit.
  bool poseReachable() const;

  // Solve IK for all feet and write the servos.
  void commit();

private:
  ServoBus servos;
  Vec3 feet[NUM_LEGS] = {};
};
