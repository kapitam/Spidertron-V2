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
#include <servoMovement.h>

void CoordinateToAngle(int leg, double X, double Y, double Z) {

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

  Serial.print("Updating POS Leg : ");
  Serial.print(leg);
  Serial.print(" A1 : ");
  Serial.print(target.A1);
  Serial.print(" A2 : ");
  Serial.print(target.A2);
  Serial.print(" A3 : ");
  Serial.println(target.A3);

  UpdatePosition(leg, target.A1, target.A2, target.A3);
  
}


// Pure IK validator that computes angles but does NOT command servos.
// Returns true if the computed angles are within allowed ranges (i.e. no constraint tripped).
bool computeAnglesInternal(int leg, double X, double Y, double Z) {
  // Calculate the length of the hypotenuse

  float A1, A2, A3;
  double N = sqrt((Y * Y) + (X * X)) - D;
  double L = sqrt((Z * Z) + (N * N));

  A1 = atan2(Y, X) * (180 / PI) + 90;

  // Defensive clamping for acos arguments to avoid NaN from FP errors
  double acos3_arg = ((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L);
  if (acos3_arg > 1.0) acos3_arg = 1.0;
  if (acos3_arg < -1.0) acos3_arg = -1.0;
  A3 = acos(acos3_arg) * (180 / PI);

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
  A2 = A + B + 90;

  if (A2 > 180) {
    A2 = 360 - A2;
  }



  // run constraint checks (same as original behavior)
    if (A1 > servo1max || A2 > servo2max || A3 > servo3max ||
        A1 < servo1min || A2 < servo2min || A3 < servo3min) {
        constraint = true;
        Serial.println("----------constraint Tripped----------");
          Serial.print("Constrained Leg : ");
          Serial.print(leg);
          Serial.print(" A1 : ");
          Serial.print(A1);
          Serial.print(" A2 : ");
          Serial.print(A2);
          Serial.print(" A3 : ");
          Serial.println(A3);
        
      return false; // Constraint tripped
    } else {
      return true; // All good
    }
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

bool updateLegCoordinates(int legAngleDeg, double x, double y, double z, bool updateServosOrCheck) {
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
  if (updateServosOrCheck) {
    CoordinateToAngle(leg, localCoord.x, localCoord.y, localCoord.z);
  } else {
    int OK;
    OK = computeAnglesInternal(leg, localCoord.x, localCoord.y, localCoord.z);
    return OK;
  }
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




  // int leg1x = REST_DISTANCE * cos(angles[0] * DEG_TO_RAD);
  // int leg1y = REST_DISTANCE * sin(angles[0] * DEG_TO_RAD);
  // int leg2x = REST_DISTANCE * cos(angles[1] * DEG_TO_RAD);
  // int leg2y = REST_DISTANCE * sin(angles[1] * DEG_TO_RAD);
  // int leg3x = REST_DISTANCE * cos(angles[2] * DEG_TO_RAD);
  // int leg3y = REST_DISTANCE * sin(angles[2] * DEG_TO_RAD);
  // int leg4x = REST_DISTANCE * cos(angles[3] * DEG_TO_RAD);
  // int leg4y = REST_DISTANCE * sin(angles[3] * DEG_TO_RAD);
  // int leg5x = REST_DISTANCE * cos(angles[4] * DEG_TO_RAD);
  // int leg5y = REST_DISTANCE * sin(angles[4] * DEG_TO_RAD);
  // int leg6x = REST_DISTANCE * cos(angles[5] * DEG_TO_RAD);
  // int leg6y = REST_DISTANCE * sin(angles[5] * DEG_TO_RAD);

  // Serial.println("Check Rest:");
  // Serial.print("LEG  1: "); Serial.print(leg1x); Serial.print(", "); Serial.println(leg1y);
  // Serial.print("LEG  2: "); Serial.print(leg2x); Serial.print(", "); Serial.println(leg2y);
  // Serial.print("LEG  3: "); Serial.print(leg3x); Serial.print(", "); Serial.println(leg3y);
  // Serial.print("LEG  4: "); Serial.print(leg4x); Serial.print(", "); Serial.println(leg4y);
  // Serial.print("LEG  5: "); Serial.print(leg5x); Serial.print(", "); Serial.println(leg5y);
  // Serial.print("LEG  6: "); Serial.print(leg6x); Serial.print(", "); Serial.println(leg6y);



  

  Serial.println("Check World Cooridates:");
  Serial.print("LEG  1: "); Serial.print(leg1.x); Serial.print(", "); Serial.print(leg1.y); Serial.print(", "); Serial.println(leg1.z);
  Serial.print("LEG  2: "); Serial.print(leg2.x); Serial.print(", "); Serial.print(leg2.y); Serial.print(", "); Serial.println(leg2.z);
  Serial.print("LEG  3: "); Serial.print(leg3.x); Serial.print(", "); Serial.print(leg3.y); Serial.print(", "); Serial.println(leg3.z);
  Serial.print("LEG  4: "); Serial.print(leg4.x); Serial.print(", "); Serial.print(leg4.y); Serial.print(", "); Serial.println(leg4.z);
  Serial.print("LEG  5: "); Serial.print(leg5.x); Serial.print(", "); Serial.print(leg5.y); Serial.print(", "); Serial.println(leg5.z);
  Serial.print("LEG  6: "); Serial.print(leg6.x); Serial.print(", "); Serial.print(leg6.y); Serial.print(", "); Serial.println(leg6.z);

  // Write coordinates
  updateLegCoordinates(0, leg1.x, leg1.y, leg1.z, true);
  updateLegCoordinates(60, leg2.x, leg2.y, leg2.z, true);
  updateLegCoordinates(120, leg3.x, leg3.y, leg3.z, true);
  updateLegCoordinates(180, leg4.x, leg4.y, leg4.z, true);
  updateLegCoordinates(240, leg5.x, leg5.y, leg5.z, true);
  updateLegCoordinates(300, leg6.x, leg6.y, leg6.z, true);

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
  bool ok1 = updateLegCoordinates(0, leg1.x, leg1.y, leg1.z, false);
  bool ok2 = updateLegCoordinates(60, leg2.x, leg2.y, leg2.z, false);
  bool ok3 = updateLegCoordinates(120, leg3.x, leg3.y, leg3.z, false);
  bool ok4 = updateLegCoordinates(180, leg4.x, leg4.y, leg4.z, false);
  bool ok5 = updateLegCoordinates(240, leg5.x, leg5.y, leg5.z, false);
  bool ok6 = updateLegCoordinates(300, leg6.x, leg6.y, leg6.z, false);

  Serial.println("World Cooridates:");
  Serial.print("LEG  1: "); Serial.print(leg1.x); Serial.print(", "); Serial.print(leg1.y); Serial.print(", "); Serial.println(leg1.z);
  Serial.print("LEG  2: "); Serial.print(leg2.x); Serial.print(", "); Serial.print(leg2.y); Serial.print(", "); Serial.println(leg2.z);
  Serial.print("LEG  3: "); Serial.print(leg3.x); Serial.print(", "); Serial.print(leg3.y); Serial.print(", "); Serial.println(leg3.z);
  Serial.print("LEG  4: "); Serial.print(leg4.x); Serial.print(", "); Serial.print(leg4.y); Serial.print(", "); Serial.println(leg4.z);
  Serial.print("LEG  5: "); Serial.print(leg5.x); Serial.print(", "); Serial.print(leg5.y); Serial.print(", "); Serial.println(leg5.z);
  Serial.print("LEG  6: "); Serial.print(leg6.x); Serial.print(", "); Serial.print(leg6.y); Serial.print(", "); Serial.println(leg6.z);

  bool allOk = ok1 && ok2 && ok3 && ok4 && ok5 && ok6;

  if (allOk) {
    // commit tentative positions and apply servo updates
    leg1 = t1; leg2 = t2; leg3 = t3; leg4 = t4; leg5 = t5; leg6 = t6;

    Serial.println("allOK Red moved successfully");

  updateLegCoordinates(0, leg1.x, leg1.y, leg1.z, true);
  updateLegCoordinates(60, leg2.x, leg2.y, leg2.z, true);
  updateLegCoordinates(120, leg3.x, leg3.y, leg3.z, true);
  updateLegCoordinates(180, leg4.x, leg4.y, leg4.z, true);
  updateLegCoordinates(240, leg5.x, leg5.y, leg5.z, true);
  updateLegCoordinates(300, leg6.x, leg6.y, leg6.z, true);

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
  bool ok1 = updateLegCoordinates(0, leg1.x, leg1.y, leg1.z, false);
  bool ok2 = updateLegCoordinates(60, leg2.x, leg2.y, leg2.z, false);
  bool ok3 = updateLegCoordinates(120, leg3.x, leg3.y, leg3.z, false);
  bool ok4 = updateLegCoordinates(180, leg4.x, leg4.y, leg4.z, false);
  bool ok5 = updateLegCoordinates(240, leg5.x, leg5.y, leg5.z, false);
  bool ok6 = updateLegCoordinates(300, leg6.x, leg6.y, leg6.z, false);

  Serial.println("World Cooridates:");
  Serial.print("LEG  1: "); Serial.print(leg1.x); Serial.print(", "); Serial.print(leg1.y); Serial.print(", "); Serial.println(leg1.z);
  Serial.print("LEG  2: "); Serial.print(leg2.x); Serial.print(", "); Serial.print(leg2.y); Serial.print(", "); Serial.println(leg2.z);
  Serial.print("LEG  3: "); Serial.print(leg3.x); Serial.print(", "); Serial.print(leg3.y); Serial.print(", "); Serial.println(leg3.z);
  Serial.print("LEG  4: "); Serial.print(leg4.x); Serial.print(", "); Serial.print(leg4.y); Serial.print(", "); Serial.println(leg4.z);
  Serial.print("LEG  5: "); Serial.print(leg5.x); Serial.print(", "); Serial.print(leg5.y); Serial.print(", "); Serial.println(leg5.z);
  Serial.print("LEG  6: "); Serial.print(leg6.x); Serial.print(", "); Serial.print(leg6.y); Serial.print(", "); Serial.println(leg6.z);

  bool allOk = ok1 && ok2 && ok3 && ok4 && ok5 && ok6;

  if (allOk) {
    // commit tentative positions and apply servo updates
    leg1 = t1; leg2 = t2; leg3 = t3; leg4 = t4; leg5 = t5; leg6 = t6;

    Serial.println("allOK Blue moved successfully");

  updateLegCoordinates(0, leg1.x, leg1.y, leg1.z, true);
  updateLegCoordinates(60, leg2.x, leg2.y, leg2.z, true);
  updateLegCoordinates(120, leg3.x, leg3.y, leg3.z, true);
  updateLegCoordinates(180, leg4.x, leg4.y, leg4.z, true);
  updateLegCoordinates(240, leg5.x, leg5.y, leg5.z, true);
  updateLegCoordinates(300, leg6.x, leg6.y, leg6.z, true);

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

void setup() {
  Serial.begin(115200);

  psx.setupPins(DATA_PIN, CMD_PIN, ATT_PIN, CLOCK_PIN, 10);
  psx.config(PSXMODE_ANALOG);

  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);

  Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C
  setRestPositions();
}

void loop() {
  //start psx
  PSXerror = psx.read(PSXdata);

  if(PSXerror == PSXERROR_SUCCESS) {
  Serial.println("controller success");
  } else {
  Serial.print("No success reading data. Check connections and timing.");
  }

  int LeftX = PSXdata.JoyLeftX - 128;
  int LeftY = PSXdata.JoyLeftY - 128;
  int RightX = PSXdata.JoyRightX - 128;
  int RightY= PSXdata.JoyRightY - 128;
  //end psx

  Serial.print(",");
  Serial.print(LeftX);
  Serial.print(",");
  Serial.print(LeftY);
  Serial.print(",");
  Serial.print(RightX);
  Serial.print(",");
  Serial.println(RightY);

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