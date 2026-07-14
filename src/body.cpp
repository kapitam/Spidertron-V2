#include "body.h"

void HexapodBody::begin() {
  servos.begin();
}

bool HexapodBody::poseReachable() const {
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    Vec3 local = worldToLocal(feet[leg], legMountAngleDeg(leg));
    JointAngles j = solveLegIK(local);

    if (!withinJointLimits(j)) {
      Serial.println("----------constraint tripped----------");
      Serial.print("Constrained Leg : ");
      Serial.print(leg);
      Serial.print(" A1 : ");
      Serial.print(j.coxa);
      Serial.print(" A2 : ");
      Serial.print(j.femur);
      Serial.print(" A3 : ");
      Serial.println(j.tibia);
      return false;
    }
  }
  return true;
}

void HexapodBody::commit() {
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    Vec3 local = worldToLocal(feet[leg], legMountAngleDeg(leg));
    servos.writeLeg(leg, solveLegIK(local));
  }
}
