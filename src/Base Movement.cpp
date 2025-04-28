
// MAJOR NOTE: DUE TO THE CONSTRAINT FUNCTION NOT BEING PROPERLY CONFIGURED, WHAT HAPPENS IS THAT THE HEXAPOD WILL RUN LEG
// UPDATE CODE UNTIL A LEG HITS THE LIMIT. THIS CAUSES ALL THE NON UPDATED LEGS TO NOT PROCEED WITH UPDATES, BUT WONT REVERT
// OLD LEG POSITIONS OTHER THEN TE LEG WHERE CONSTRAINT WAS REACHED.
// THIS MEANS THAT THE HEXAPOD WILL HAVE TO BE SET BACK TO REST POSITION EVERY ONCE IN A WHILE TO PREVENT LEG DRIFT.


#include <SPI.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PSX.h>

#define DATA_PIN   25   // White wire
#define CMD_PIN    26  // Black wire
#define ATT_PIN    32  // Gray wire
#define CLOCK_PIN  33  // black wire

//Assign Servo on PWM adafruit
#define SERVO_PINFR1 0
#define SERVO_PINFR2 1
#define SERVO_PINFR3 2
#define SERVO_PINCR1 4
#define SERVO_PINCR2 5
#define SERVO_PINCR3 6
#define SERVO_PINBR1 8
#define SERVO_PINBR2 9
#define SERVO_PINBR3 10
#define SERVO_PINFL1 0
#define SERVO_PINFL2 1
#define SERVO_PINFL3 2
#define SERVO_PINCL1 4
#define SERVO_PINCL2 5
#define SERVO_PINCL3 6
#define SERVO_PINBL1 8
#define SERVO_PINBL2 9
#define SERVO_PINBL3 10

const int minPulse = 450;
const int maxPulse = 2500; 

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

PSX psx;

// Variables to hold the controller state and error code
PSX::PSXDATA PSXdata;
int PSXerror;

// Struct for leg coordinates
struct Leg {
  double x;
  double y;
  double z;
};

// Struct for leg angles
struct Angle {
  float A1;
  float A2;
  float A3;
};

// Struct Direction and Mag
struct Dir {
  double mag;
  double rad;
};

const double J2L = 10.7;  // Length of the femur
const double J3L = 16.8;  // Length of the tibia
const double D = 6.5;     // Offset length from the start of femur to axis
const double legOffset = 9.83;  // From center of hexapod

// Global leg objects labeled leg1 ... leg6
Leg leg1, leg2, leg3, leg4, leg5, leg6;

// Adjustable variables
const float REST_DISTANCE = 17.0;   // Distance from center for rest position (adjust as needed)
const float GROUND_Z = -8.0;          // Z coordinate for legs on the ground
const float AIR_Z = 0.0;            // Z coordinate for legs in the air
// this is calculated with the magnitude function range of 0-2, the units in cm
// the speed at dx and dy is updated is in 10ms, therefore to get speed of 0-2 cm/s this must be 0.01
float fastnessMultiplier = 0.1;      // multiplier
bool legnowblue = false;
bool constraint = false;
int transdur = 200; // time taken to transition in ms

void setup() {
  Serial.begin(115200);

  Serial.println("Begin");

  psx.setupPins(DATA_PIN, CMD_PIN, ATT_PIN, CLOCK_PIN, 10);
  psx.config(PSXMODE_ANALOG);

  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);
}

double translator(double angle) {
  double temp = minPulse + (angle / 180.0) * (maxPulse - minPulse);
  return temp * 4096 / (1000000 / 50); // Convert to PWM ticks
}

void UpdatePosition(int leg, double A1, double A2, double A3) {
  
  // translate leg angles into servo angles
  A1 = translator(180 - A1);
  A2 = translator(A2);
  A3 = translator(180 - A3);
  //Serial.println(leg);
  switch (leg) {
    case  0:
      pwm1.setPWM(SERVO_PINFR1, 0, A1);
      pwm1.setPWM(SERVO_PINFR2, 0, A2);
      pwm1.setPWM(SERVO_PINFR3, 0, A3);
      //Serial.println("leg 0");
      break;
    case  1:
      pwm1.setPWM(SERVO_PINCR1, 0, A1);
      pwm1.setPWM(SERVO_PINCR2, 0, A2);
      pwm1.setPWM(SERVO_PINCR3, 0, A3);
      //Serial.println("leg 1");
      break;
    case 2:
      pwm1.setPWM(SERVO_PINBR1, 0, A1);
      pwm1.setPWM(SERVO_PINBR2, 0, A2);
      pwm1.setPWM(SERVO_PINBR3, 0, A3);
      //Serial.println("leg 2");
      break;
    case 3:
      pwm2.setPWM(SERVO_PINFL1, 0, A1);
      pwm2.setPWM(SERVO_PINFL2, 0, A2);
      pwm2.setPWM(SERVO_PINFL3, 0, A3);
      //Serial.println("leg 3");
      break;
    case 4:
      pwm2.setPWM(SERVO_PINCL1, 0, A1);
      pwm2.setPWM(SERVO_PINCL2, 0, A2);
      pwm2.setPWM(SERVO_PINCL3, 0, A3);
      ///Serial.println("leg 4");
      break;
    case 5:
      pwm2.setPWM(SERVO_PINBL1, 0, A1);
      pwm2.setPWM(SERVO_PINBL2, 0, A2);
      pwm2.setPWM(SERVO_PINBL3, 0, A3);
      //Serial.println("leg 5");
      break;
    default:
      break;
  } 
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
  constraint = false;
// Calculate movement vector components (scaled by fastnessMultiplier).
  float dx = cos(moveRad) * movemagnitude * fastnessMultiplier;
  float dy = sin(moveRad) * movemagnitude * fastnessMultiplier;
  
  // Define two leg groups:
  // Red group: legs 1, 3, 5
  leg1.x -= dx;
  leg1.y -= dy;
  
  leg3.x -= dx;
  leg3.y -= dy;
  
  leg5.x -= dx;
  leg5.y -= dy;
  
  // Blue group: legs 2, 4, 6
  leg2.x += dx;
  leg2.y += dy;
  
  leg4.x += dx;
  leg4.y += dy;
  
  leg6.x += dx;
  leg6.y += dy;
  
  // Red group (on ground)
  leg1.z = GROUND_Z;
  leg3.z = GROUND_Z;
  leg5.z = GROUND_Z;
  
  // Blue group (in air)
  leg2.z = AIR_Z;
  leg4.z = AIR_Z;
  leg6.z = AIR_Z;

  // Write coordinates
  updateLegCoordinates(0, leg1.x, leg1.y, leg1.z, false);
  updateLegCoordinates(60, leg2.x, leg2.y, leg2.z, true);
  updateLegCoordinates(120, leg3.x, leg3.y, leg3.z, false);
  updateLegCoordinates(180, leg4.x, leg4.y, leg4.z, true);
  updateLegCoordinates(240, leg5.x, leg5.y, leg5.z, false);
  updateLegCoordinates(300, leg6.x, leg6.y, leg6.z, true);

  // reset amount servo coordinate back after constraint reached
  if (constraint == true) {
    // Define two leg groups:
    // Red group: legs 1, 3, 5
    leg1.x += dx;
    leg1.y += dy;
    
    leg3.x += dx;
    leg3.y += dy;
    
    leg5.x += dx;
    leg5.y += dy;
    
    // Blue group: legs 2, 4, 6
    leg2.x -= dx;
    leg2.y -= dy;
    
    leg4.x -= dx;
    leg4.y -= dy;
    
    leg6.x -= dx;
    leg6.y -= dy;
  }
}

void moveblue(float moveRad, float movemagnitude) {
  constraint = false;
  // Calculate movement vector components (scaled by fastnessMultiplier).
  float dx = cos(moveRad) * movemagnitude * fastnessMultiplier;
  float dy = sin(moveRad) * movemagnitude * fastnessMultiplier;
  
  // Define two leg groups:
  // Red group: legs 1, 3, 5
  leg1.x += dx;
  leg1.y += dy;
  
  leg3.x += dx;
  leg3.y += dy;
  
  leg5.x += dx;
  leg5.y += dy;
  
  // Blue group: legs 2, 4, 6
  leg2.x -= dx;
  leg2.y -= dy;
  
  leg4.x -= dx;
  leg4.y -= dy;
  
  leg6.x -= dx;
  leg6.y -= dy;
  
  // Red group (on ground)
  leg1.z = AIR_Z;
  leg3.z = AIR_Z;
  leg5.z = AIR_Z;
  
  // Blue group (in air)
  leg2.z = GROUND_Z;
  leg4.z = GROUND_Z;
  leg6.z = GROUND_Z;

  // Write coordinates
  updateLegCoordinates(0, leg1.x, leg1.y, leg1.z, true);
  updateLegCoordinates(60, leg2.x, leg2.y, leg2.z, false);
  updateLegCoordinates(120, leg3.x, leg3.y, leg3.z, true);
  updateLegCoordinates(180, leg4.x, leg4.y, leg4.z, false);
  updateLegCoordinates(240, leg5.x, leg5.y, leg5.z, true);
  updateLegCoordinates(300, leg6.x, leg6.y, leg6.z, false);

  // reset amount servo coordinate back after constraint reached
  if (constraint == true) {
    // Define two leg groups:
    // Red group: legs 1, 3, 5
    leg1.x -= dx;
    leg1.y -= dy;
    
    leg3.x -= dx;
    leg3.y -= dy;
    
    leg5.x -= dx;
    leg5.y -= dy;
    
    // Blue group: legs 2, 4, 6
    leg2.x += dx;
    leg2.y += dy;
    
    leg4.x += dx;
    leg4.y += dy;
    
    leg6.x += dx;
    leg6.y += dy;
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