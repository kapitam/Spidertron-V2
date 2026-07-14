// TEMPORARY Wokwi verification build.
// Compares the refactored kinematics against the remote working commit's
// math (copied verbatim from ee77e62) and runs a short gait smoke test.
// This file is restored to the real main.cpp after the test.

#include <Arduino.h>
#include <math.h>

#include "body.h"
#include "config.h"
#include "gait.h"
#include "kinematics.h"

// ===== Reference implementation, verbatim from commit ee77e62 =====
namespace reference {

struct Leg { double x, y, z; };
struct Angle { float A1, A2, A3; };

const double J2L = 8.8;
const double J3L = 15.5;
const double D = 5.5;

Leg worldToLocal(const Leg &world, double legAngleDeg) {
  double legAngleRad = legAngleDeg * DEG_TO_RAD;
  Leg local;
  local.x = (world.x * cos(legAngleRad)) + (world.y * sin(legAngleRad));
  local.y = -(world.x * sin(legAngleRad)) + (world.y * cos(legAngleRad));
  local.z = world.z;
  return local;
}

Angle coordinateToAngle(double X, double Y, double Z) {
  Angle target;
  double N = sqrt((Y * Y) + (X * X)) - D;
  double L = sqrt((Z * Z) + (N * N));

  target.A1 = atan2(Y, X) * (180 / PI) + 90;
  target.A3 = acos(((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L)) * (180 / PI);
  double B = acos(((L * L) + (J2L * J2L) - (J3L * J3L)) / (2 * L * J2L)) * (180 / PI);
  double A = atan2(Z, N) * (180 / PI);
  target.A2 = A + B + 90;

  if (target.A2 > 180) {
    target.A2 = 360 - target.A2;
  }
  return target;
}

}  // namespace reference

HexapodBody body;
TripodGait gait(body);

int compared = 0;
int failures = 0;
int nanPoses = 0;
double maxErrDeg = 0;

void comparePose(int leg, double wx, double wy, double wz) {
  Vec3 local = worldToLocal({wx, wy, wz}, legMountAngleDeg(leg));
  JointAngles j = solveLegIK(local);

  reference::Leg refLocal = reference::worldToLocal({wx, wy, wz}, leg * 60.0);
  reference::Angle ref = reference::coordinateToAngle(refLocal.x, refLocal.y, refLocal.z);

  compared++;

  bool newNan = isnan(j.coxa) || isnan(j.femur) || isnan(j.tibia);
  bool refNan = isnan(ref.A1) || isnan(ref.A2) || isnan(ref.A3);
  if (newNan || refNan) {
    nanPoses++;
    if (newNan != refNan) {
      failures++;
      Serial.printf("NaN mismatch leg %d at (%.2f, %.2f, %.2f)\n", leg, wx, wy, wz);
    }
    return;
  }

  double err = max(fabs(j.coxa - ref.A1),
                   max(fabs(j.femur - ref.A2), fabs(j.tibia - ref.A3)));
  if (err > maxErrDeg) maxErrDeg = err;
  if (err > 0.001) {  // reference stores floats, so allow rounding noise
    failures++;
    Serial.printf("MISMATCH leg %d at (%.2f, %.2f, %.2f): "
                  "new(%.4f, %.4f, %.4f) ref(%.4f, %.4f, %.4f)\n",
                  leg, wx, wy, wz, j.coxa, j.femur, j.tibia,
                  ref.A1, ref.A2, ref.A3);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== IK parity test: refactor vs remote working commit ===");

  // Sweep world-frame foot positions around each leg's rest position.
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    double restRad = legMountAngleDeg(leg) * DEG_TO_RAD;
    double rx = REST_DISTANCE * cos(restRad);
    double ry = REST_DISTANCE * sin(restRad);

    for (double dx = -6; dx <= 6; dx += 1.5) {
      for (double dy = -6; dy <= 6; dy += 1.5) {
        comparePose(leg, rx + dx, ry + dy, GROUND_Z);
        comparePose(leg, rx + dx, ry + dy, AIR_Z);
      }
    }
  }

  Serial.printf("Compared %d poses (%d unreachable in both), max error %.6f deg\n",
                compared, nanPoses, maxErrDeg);
  Serial.println(failures == 0 ? "IK PARITY: PASSED" : "IK PARITY: FAILED");

  // Gait smoke test: walk forward from rest, watch for tripod switches.
  Serial.println("=== Gait smoke test: walking heading 0, magnitude 1 ===");
  body.begin();
  gait.rest();
  for (int i = 0; i < 150; i++) {
    gait.walk(0.0, 1.0);
  }
  Serial.println("GAIT TEST DONE");
}

void loop() {}
