#include "gait.h"

bool TripodGait::isStance(int leg) const {
  bool inTripodB = (leg % 2 == 1);
  return inTripodB == stanceIsB;
}

void TripodGait::rest() {
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    double rad = legMountAngleDeg(leg) * DEG_TO_RAD;
    body.foot(leg) = { REST_DISTANCE * cos(rad), REST_DISTANCE * sin(rad), GROUND_Z };
  }
  body.commit();
  delay(TRANSITION_MS);
}

bool TripodGait::step(double headingRad, double magnitude) {
  double dx = cos(headingRad) * magnitude * STEP_SCALE;
  double dy = sin(headingRad) * magnitude * STEP_SCALE;

  Vec3 previous[NUM_LEGS];
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    previous[leg] = body.foot(leg);
  }

  for (int leg = 0; leg < NUM_LEGS; leg++) {
    double sign = isStance(leg) ? -1.0 : 1.0;
    Vec3 &foot = body.foot(leg);
    foot.x += sign * dx;
    foot.y += sign * dy;
    foot.z = isStance(leg) ? GROUND_Z : AIR_Z;
  }

  // Validate the whole pose before any servo write: no partial movement.
  if (!body.poseReachable()) {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
      body.foot(leg) = previous[leg];
    }
    return false;
  }

  body.commit();
#if DEBUG_LOG
  Serial.println(stanceIsB ? "allOK Blue moved successfully"
                           : "allOK Red moved successfully");
#endif
  return true;
}

void TripodGait::walk(double headingRad, double magnitude) {
  bool ok = step(headingRad, magnitude);
  delay(10);  // stability

  if (!ok) {
    // Stance tripod ran out of range: swap tripods and take the first step
    // with the new stance group.
    stanceIsB = !stanceIsB;
    step(headingRad, magnitude);
    delay(TRANSITION_MS);
    Serial.println(stanceIsB ? "Switching to Blue" : "Switching to Red");
  }
}
