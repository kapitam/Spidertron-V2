#include "body.h"

void HexapodBody::begin() {
  servos.begin();
}

bool HexapodBody::commit() {
  bool limitHit = false;

  for (int leg = 0; leg < NUM_LEGS; leg++) {
    Vec3 local = worldToLocal(feet[leg], legMountAngleDeg(leg));
    JointAngles j = solveLegIK(local);

#if DEBUG_LOG
    Serial.print("Leg (Angle ");
    Serial.print(legMountAngleDeg(leg));
    Serial.print("°): Local X: ");
    Serial.print(local.x, 6);
    Serial.print(" Y: ");
    Serial.print(local.y, 6);
    Serial.print(" Z: ");
    Serial.println(local.z, 6);
    Serial.print("A1 : ");
    Serial.print(j.coxa);
    Serial.print(" A2 : ");
    Serial.print(j.femur);
    Serial.print(" A3 : ");
    Serial.println(j.tibia);
#endif

    if (!limitHit) {
      servos.writeLeg(leg, j);
    }
    if (!withinJointLimits(j)) {
      limitHit = true;
    }
  }

  return !limitHit;
}
