#pragma once

#include "config.h"

// Pure geometry: no hardware access, no global state. Everything a motion
// planner needs to reason about poses lives here.
//
// Coordinate convention: X/Y is the horizontal plane, Z is vertical
// (negative = below the body).

struct Vec3 {
  double x;
  double y;
  double z;
};

// Servo-space joint angles for one leg, in degrees.
struct JointAngles {
  double coxa;   // A1: hip rotation around the vertical axis
  double femur;  // A2: hip up/down
  double tibia;  // A3: knee
};

// Angle at which leg `leg` is mounted around the body (degrees).
inline double legMountAngleDeg(int leg) {
  return leg * LEG_MOUNT_STEP_DEG;
}

// Rotate a world-frame foot position into the leg's local frame.
Vec3 worldToLocal(const Vec3 &world, double mountAngleDeg);

// Inverse kinematics: foot position in the leg's local frame -> joint angles.
JointAngles solveLegIK(const Vec3 &local);

// True if all three joints are within the servo limits from config.h.
bool withinJointLimits(const JointAngles &j);
