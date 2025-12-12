// SPLIT UP CODE INTO BITE SIZES

#include <SPI.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PSX.h>

#include <servoPins.h>
#include <controller.h>
#include <logicStruct.h>
#include <logicVaribles.h>
#include "servoMovement.cpp"

void setup() {
  Serial.begin(115200);

  psx.setupPins(DATA_PIN, CMD_PIN, ATT_PIN, CLOCK_PIN, 10);
  psx.config(PSXMODE_ANALOG);

  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);

  Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C
}

// constraint values for when a motor has went too far, also switches the leg groups
void constraintReached(double servo1, double servo2, double servo3) {
  if (servo1 > 120 || servo2 > 180 || servo3 > 100 || servo1 < 30 || servo2 < 110 || servo3 < 44) {
    constraint = true;
    Serial.println("----------constraint Tripped----------");
  }
}


void CoordinateToAngle(int leg, double X, double Y, double Z, bool legInAir) {

  Angle target;

  // Calculate the length of the hypotenuse
  double N = sqrt((Y * Y) + (X * X)) - D;
  double L = sqrt((Z * Z) + (N * N));

  target.A1 = atan2(Y,X) * (180 / PI) + 90;


  // Calculate the angles using inverse kinematics
  target.A3 = acos(((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L)) * (180 / PI);
  double B = acos(((L * L) + (J2L * J2L) - (J3L * J3L)) / (2 * L * J2L)) * (180 / PI);
  double A = atan2(Z, N) * (180 / PI);
  target.A2 = A + B + 90;


  if (target.A2 > 180) {
  target.A2 = 360 - target.A2;
  }

  if (!legInAir) {
    constraintReached(target.A1, target.A2, target.A3);
  }
  Serial.print("Leg : ");
  Serial.print(leg);
  Serial.print(" A1 : ");
  Serial.print(target.A1);
  Serial.print(" A2 : ");
  Serial.print(target.A2);
  Serial.print(" A3 : ");
  Serial.println(target.A3);

  if (!constraint) {
    UpdatePosition(leg, target.A1, target.A2, target.A3);
      }
}


// Pure IK validator that computes angles but does NOT command servos.
// Returns true if the computed angles are within allowed ranges (i.e. no constraint tripped).
bool computeAnglesInternal(int leg, double X, double Y, double Z, Angle &target, bool legInAir) {
  // Calculate the length of the hypotenuse
  double N = sqrt((Y * Y) + (X * X)) - D;
  double L = sqrt((Z * Z) + (N * N));

  target.A1 = atan2(Y, X) * (180 / PI) + 90;

  // Defensive clamping for acos arguments to avoid NaN from FP errors
  double acos3_arg = ((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L);
  if (acos3_arg > 1.0) acos3_arg = 1.0;
  if (acos3_arg < -1.0) acos3_arg = -1.0;
  target.A3 = acos(acos3_arg) * (180 / PI);

  double acosB_arg = 0.0;
  if (L > 1e-8) {
    acosB_arg = ((L * L) + (J2L * J2L) - (J3L * J3L)) / (2 * L * J2L);
    if (acosB_arg > 1.0) acosB_arg = 1.0;
    if (acosB_arg < -1.0) acosB_arg = -1.0;
  } else {
    // Degenerate case: set a safe default to avoid division by zero
    acosB_arg = 1.0;
  }
  double B = acos(acosB_arg) * (180 / PI);
  double A = atan2(Z, N) * (180 / PI);
  target.A2 = A + B + 90;

  if (target.A2 > 180) {
    target.A2 = 360 - target.A2;
  }

  // If the leg is considered on the ground, run constraint checks (same as original behavior)
  if (!legInAir) {
    constraintReached(target.A1, target.A2, target.A3);
  }

  // Return success if no constraint was tripped
  return !constraint;
}

Leg worldToLocal(const Leg &world, double legAngleDeg, double offset) {
  Leg local;
  
  // Convert leg angle from degrees to radians
  double legAngleRad = legAngleDeg * DEG_TO_RAD;
  
  // X Y Logic
  local.x = (world.x  * cos(legAngleRad)) + (world.y  * sin(legAngleRad));
  local.y = -(world.x  * sin(legAngleRad)) + (world.y  * cos(legAngleRad));
  
  // Y remains unchanged
  local.z = world.z;
  
  return local;
}

void updateLegCoordinates(int legAngleDeg, double x, double y, double z, bool legInAir) {
  Leg worldLegPos = { x, y, z };  // World coordinates
  Leg localCoord = worldToLocal(worldLegPos, legAngleDeg, legOffset);
  int leg = legAngleDeg / 60; // this is range 0-5
  // Serial.print("Leg (Angle ");
  // Serial.print(legAngleDeg);
  // Serial.print("°, Leg: ");
  // Serial.print(leg);
  // Serial.print(") ");
  // Serial.print("Local X: ");
  // Serial.print(localCoord.x, 6); // Print with higher precision
  // Serial.print(" Y: ");
  // Serial.print(localCoord.y, 6);
  // Serial.print(" Z: ");
  // Serial.println(localCoord.z, 6);
  CoordinateToAngle(leg, localCoord.x, localCoord.y, localCoord.z, legInAir);
}

// Function to set legs to their rest positions.
// Here we assume a circular arrangement for simplicity.
void setRestPositions() {
  float angles[6] = {0, 60, 120, 180, 240, 300};  // degrees for each leg
  leg1.x = REST_DISTANCE * cos(angles[0] * DEG_TO_RAD);
  leg1.y = REST_DISTANCE * sin(angles[0] * DEG_TO_RAD);
  
  leg2.x = REST_DISTANCE * cos(angles[1] * DEG_TO_RAD);
  leg2.y = REST_DISTANCE * sin(angles[1] * DEG_TO_RAD);
  
  leg3.x = REST_DISTANCE * cos(angles[2] * DEG_TO_RAD);
  leg3.y = REST_DISTANCE * sin(angles[2] * DEG_TO_RAD);
  
  leg4.x = REST_DISTANCE * cos(angles[3] * DEG_TO_RAD);
  leg4.y = REST_DISTANCE * sin(angles[3] * DEG_TO_RAD);
  
  leg5.x = REST_DISTANCE * cos(angles[4] * DEG_TO_RAD);
  leg5.y = REST_DISTANCE * sin(angles[4] * DEG_TO_RAD);
  
  leg6.x = REST_DISTANCE * cos(angles[5] * DEG_TO_RAD);
  leg6.y = REST_DISTANCE * sin(angles[5] * DEG_TO_RAD);
  
  // Optionally, set all legs to a default z (e.g., all on the ground)
  leg1.z = GROUND_Z;
  leg2.z = GROUND_Z;
  leg3.z = GROUND_Z;
  leg4.z = GROUND_Z;
  leg5.z = GROUND_Z;
  leg6.z = GROUND_Z;
  // Serial.println("within reset postion function (leg: 1, 2, ...)");
  // Serial.print("Z :"); Serial.print(leg1.z); Serial.print(" "); Serial.print(leg2.z); Serial.print(" "); Serial.print(leg3.z);
  // Serial.print(" "); Serial.print(leg4.z); Serial.print(" "); Serial.print(leg5.z); Serial.print(" "); Serial.println(leg6.z);
  // Serial.print("X :"); Serial.print(leg1.x); Serial.print(" "); Serial.print(leg2.x); Serial.print(" "); Serial.print(leg3.x);
  // Serial.print(" "); Serial.print(leg4.x); Serial.print(" "); Serial.print(leg5.x); Serial.print(" "); Serial.println(leg6.x);
  // Serial.print("Y :"); Serial.print(leg1.y); Serial.print(" "); Serial.print(leg2.y); Serial.print(" "); Serial.print(leg3.y);
  // Serial.print(" "); Serial.print(leg4.y); Serial.print(" "); Serial.print(leg5.y); Serial.print(" "); Serial.println(leg6.y);

  // Write coordinates
  updateLegCoordinates(0, leg1.x, leg1.y, leg1.z, false);
  updateLegCoordinates(60, leg2.x, leg2.y, leg2.z, false);
  updateLegCoordinates(120, leg3.x, leg3.y, leg3.z, false);
  updateLegCoordinates(180, leg4.x, leg4.y, leg4.z, false);
  updateLegCoordinates(240, leg5.x, leg5.y, leg5.z, false);
  updateLegCoordinates(300, leg6.x, leg6.y, leg6.z, false);
}

void movered(float moveRad, float movemagnitude) {
  // Do not reset constraint here; only validate new positions first
  // Calculate movement vector components (scaled by fastnessMultiplier).
  float dx = cos(moveRad) * movemagnitude * fastnessMultiplier;
  float dy = sin(moveRad) * movemagnitude * fastnessMultiplier;

  // Create tentative positions (do not modify globals yet)
  Leg t1 = leg1; 
  Leg t2 = leg2; 
  Leg t3 = leg3; 
  Leg t4 = leg4; 
  Leg t5 = leg5; 
  Leg t6 = leg6;

  // Apply deltas to tentative positions
  t1.x -= dx; t1.y -= dy; t1.z = GROUND_Z; // red on ground
  t3.x -= dx; t3.y -= dy; t3.z = GROUND_Z;
  t5.x -= dx; t5.y -= dy; t5.z = GROUND_Z;

  t2.x += dx; t2.y += dy; t2.z = AIR_Z; // blue in air
  t4.x += dx; t4.y += dy; t4.z = AIR_Z;
  t6.x += dx; t6.y += dy; t6.z = AIR_Z;

  // Validate all six legs using computeAnglesInternal before committing any servo writes
  Angle a1, a2, a3, a4, a5, a6;
  // Temporarily clear constraint flag; computeAnglesInternal will set it if needed
  bool prevConstraint = constraint;
  constraint = false;
  bool ok1 = computeAnglesInternal(0, t1.x, t1.y, t1.z, a1, false);
  bool ok2 = computeAnglesInternal(1, t2.x, t2.y, t2.z, a2, true);
  bool ok3 = computeAnglesInternal(2, t3.x, t3.y, t3.z, a3, false);
  bool ok4 = computeAnglesInternal(3, t4.x, t4.y, t4.z, a4, true);
  bool ok5 = computeAnglesInternal(4, t5.x, t5.y, t5.z, a5, false);
  bool ok6 = computeAnglesInternal(5, t6.x, t6.y, t6.z, a6, true);

  bool allOk = ok1 && ok2 && ok3 && ok4 && ok5 && ok6;

  if (allOk) {
    // commit tentative positions and apply servo updates
    leg1 = t1; leg2 = t2; leg3 = t3; leg4 = t4; leg5 = t5; leg6 = t6;
    UpdatePosition(0, a1.A1, a1.A2, a1.A3);
    UpdatePosition(1, a2.A1, a2.A2, a2.A3);
    UpdatePosition(2, a3.A1, a3.A2, a3.A3);
    UpdatePosition(3, a4.A1, a4.A2, a4.A3);
    UpdatePosition(4, a5.A1, a5.A2, a5.A3);
    UpdatePosition(5, a6.A1, a6.A2, a6.A3);
  } else {
    // If validation failed, restore previous constraint state and do not change positions
    constraint = true; // signal that a move was rejected
    // Keep global legs unchanged (no partial movement)
  }
}

void moveblue(float moveRad, float movemagnitude) {
  // Calculate movement vector components (scaled by fastnessMultiplier).
  float dx = cos(moveRad) * movemagnitude * fastnessMultiplier;
  float dy = sin(moveRad) * movemagnitude * fastnessMultiplier;

  // Create tentative positions (do not modify globals yet)
  Leg t1 = leg1; 
  Leg t2 = leg2; 
  Leg t3 = leg3; 
  Leg t4 = leg4; 
  Leg t5 = leg5; 
  Leg t6 = leg6;

  // Apply deltas to tentative positions (reverse signs from movered)
  t1.x += dx; t1.y += dy; t1.z = AIR_Z; // red in air
  t3.x += dx; t3.y += dy; t3.z = AIR_Z;
  t5.x += dx; t5.y += dy; t5.z = AIR_Z;

  t2.x -= dx; t2.y -= dy; t2.z = GROUND_Z; // blue on ground
  t4.x -= dx; t4.y -= dy; t4.z = GROUND_Z;
  t6.x -= dx; t6.y -= dy; t6.z = GROUND_Z;

  // Validate all six legs using computeAnglesInternal before committing any servo writes
  Angle a1, a2, a3, a4, a5, a6;
  bool prevConstraint = constraint;
  constraint = false;
  bool ok1 = computeAnglesInternal(0, t1.x, t1.y, t1.z, a1, true);
  bool ok2 = computeAnglesInternal(1, t2.x, t2.y, t2.z, a2, false);
  bool ok3 = computeAnglesInternal(2, t3.x, t3.y, t3.z, a3, true);
  bool ok4 = computeAnglesInternal(3, t4.x, t4.y, t4.z, a4, false);
  bool ok5 = computeAnglesInternal(4, t5.x, t5.y, t5.z, a5, true);
  bool ok6 = computeAnglesInternal(5, t6.x, t6.y, t6.z, a6, false);

  bool allOk = ok1 && ok2 && ok3 && ok4 && ok5 && ok6;

  if (allOk) {
    // commit tentative positions and apply servo updates
    leg1 = t1; leg2 = t2; leg3 = t3; leg4 = t4; leg5 = t5; leg6 = t6;
    UpdatePosition(0, a1.A1, a1.A2, a1.A3);
    UpdatePosition(1, a2.A1, a2.A2, a2.A3);
    UpdatePosition(2, a3.A1, a3.A2, a3.A3);
    UpdatePosition(3, a4.A1, a4.A2, a4.A3);
    UpdatePosition(4, a5.A1, a5.A2, a5.A3);
    UpdatePosition(5, a6.A1, a6.A2, a6.A3);
  } else {
    // If validation failed, restore previous constraint state and do not change positions
    constraint = true; // signal that a move was rejected
  }
}

// control magnitude and direction cal
Dir controller(int x, int y) {
  Dir calculated;
  calculated.rad = atan2(y, x);
  calculated.mag = sqrt(x*x + y*y) / 128;
  return calculated;
}

// switch leg logic with constraint and delay
void movementXY(float moveRad, float movemagnitude) {
  if (legnowblue == false) {
    movered(moveRad, movemagnitude);
    delay(10); // stability
    //Serial.println("Red moving");
    if (constraint) {
      legnowblue = true;
      constraint = false;
      moveblue(moveRad, movemagnitude);
      delay(transdur);
    }
  } else {
    moveblue(moveRad, movemagnitude);
    delay(10); // stability
    //Serial.println("Blue moving");
    if (constraint) {
      legnowblue = false;
      constraint = false;
      movered(moveRad, movemagnitude);
      delay(transdur);
    }
  }
}

// rotation logic
void movementRot(float magnitude) {
  // insert stuff here later if want
}

void loop() {
  //start psx
  PSXerror = psx.read(PSXdata);

  if(PSXerror == PSXERROR_SUCCESS) {
  //Serial.print("success");
  } else {
  //Serial.print("No success reading data. Check connections and timing.");
  }

  int LeftX = PSXdata.JoyLeftX - 128;
  int LeftY = PSXdata.JoyLeftY - 128;
  int RightX = PSXdata.JoyRightX - 128;
  int RightY= PSXdata.JoyRightY - 128;
  //end psx

  //Serial.print("test controller LeftX: ");
  //Serial.print(LeftX);
  //Serial.print(" LeftY: ");
  //Serial.println(LeftY);
  if (abs(LeftX) + abs(LeftY) < 15) {
    setRestPositions();
    delay(transdur);
  } else {
    movementXY(controller(LeftX, LeftY).rad, controller(LeftX, LeftY).mag);
  }
}