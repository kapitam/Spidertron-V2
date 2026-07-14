#pragma once

#include "body.h"

// Tripod-gait motion planner. This is the layer to swap out for ROS later:
// a replacement planner only needs to set body.foot(leg) targets (world
// frame, cm) and call body.commit() — nothing below this file has to change.
//
// The gait alternates between two tripods:
//   tripod A = legs 0, 2, 4    tripod B = legs 1, 3, 5
// The stance tripod slides backwards on the ground (pushing the body toward
// the heading) while the swing tripod moves forwards through the air. The
// tripods swap whenever the stance tripod runs out of joint range.
class TripodGait {
public:
  explicit TripodGait(HexapodBody &body) : body(body) {}

  // All feet on a circle around the body, all on the ground.
  void rest();

  // One walking update toward headingRad with stick magnitude 0-2.
  void walk(double headingRad, double magnitude);

private:
  // One gait increment. Returns false (and undoes the position shift) if a
  // joint limit was hit.
  bool step(double headingRad, double magnitude);

  bool isStance(int leg) const;

  HexapodBody &body;
  bool stanceIsB = false;  // which tripod is currently on the ground
};
