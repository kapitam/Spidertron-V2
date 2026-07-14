#include "kinematics.h"

#include <math.h>

Vec3 worldToLocal(const Vec3 &world, double mountAngleDeg) {
  double rad = mountAngleDeg * DEG_TO_RAD;

  double x = world.x - LEG_OFFSET * cos(rad);
  double z = world.z - LEG_OFFSET * sin(rad);

  Vec3 local;
  local.x = x * cos(rad) - z * sin(rad);
  local.z = x * sin(rad) + z * cos(rad);
  local.y = world.y;
  return local;
}

JointAngles solveLegIK(const Vec3 &local) {
  double n = sqrt(local.z * local.z + local.x * local.x) - COXA_OFFSET;
  double l = sqrt(local.y * local.y + n * n);

  JointAngles j;
  j.coxa = atan2(local.z, local.x) * RAD_TO_DEG + 90;

  j.tibia = acos(((FEMUR_LENGTH * FEMUR_LENGTH) + (TIBIA_LENGTH * TIBIA_LENGTH) - (l * l)) /
                 (2 * FEMUR_LENGTH * TIBIA_LENGTH)) * RAD_TO_DEG;

  double b = acos(((l * l) + (FEMUR_LENGTH * FEMUR_LENGTH) - (TIBIA_LENGTH * TIBIA_LENGTH)) /
                  (2 * l * FEMUR_LENGTH)) * RAD_TO_DEG;
  j.femur = atan2(local.y, n) * RAD_TO_DEG + b + 90;
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
