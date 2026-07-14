#include "kinematics.h"

#include <math.h>

Vec3 worldToLocal(const Vec3 &world, double mountAngleDeg) {
  double rad = mountAngleDeg * DEG_TO_RAD;

  Vec3 local;
  local.x =  world.x * cos(rad) + world.y * sin(rad);
  local.y = -world.x * sin(rad) + world.y * cos(rad);
  local.z = world.z;
  return local;
}

JointAngles solveLegIK(const Vec3 &local) {
  double n = sqrt(local.x * local.x + local.y * local.y) - COXA_OFFSET;
  double l = sqrt(local.z * local.z + n * n);

  JointAngles j;
  j.coxa = atan2(local.y, local.x) * RAD_TO_DEG + 90;

  j.tibia = acos(((FEMUR_LENGTH * FEMUR_LENGTH) + (TIBIA_LENGTH * TIBIA_LENGTH) - (l * l)) /
                 (2 * FEMUR_LENGTH * TIBIA_LENGTH)) * RAD_TO_DEG;

  double b = acos(((l * l) + (FEMUR_LENGTH * FEMUR_LENGTH) - (TIBIA_LENGTH * TIBIA_LENGTH)) /
                  (2 * l * FEMUR_LENGTH)) * RAD_TO_DEG;
  j.femur = atan2(local.z, n) * RAD_TO_DEG + b + 90;
  if (j.femur > 180) {
    j.femur = 360 - j.femur;
  }
  return j;
}

bool withinJointLimits(const JointAngles &j) {
  return j.coxa  >= A1_MIN && j.coxa  <= A1_MAX &&
         j.femur >= A2_MIN && j.femur <= A2_MAX &&
         j.tibia >= A3_MIN && j.tibia <= A3_MAX;
}
