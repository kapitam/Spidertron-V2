// MAIN CODE EDIT, PLEASE CHECK Dwag

const double J2L = 8.8;  // Length of the femur
const double J3L = 15.5;  // Length of the tibia
const double D = 5.5;     // Offset length from the start of femur to axis
const double legOffset = 9.83;  // From center of hexapod

// Adjustable variables
const float REST_DISTANCE = 21.0;   // Distance from center for rest position (adjust as needed)
const float GROUND_Z = -3.0;          // Z coordinate for legs on the ground
const float AIR_Z = 0.0;            // Z coordinate for legs in the air

// this is calculated with the magnitude function range of 0-2, the units in cm
// the speed at dx and dy is updated is in 10ms, therefore to get speed of 0-2 cm/s this must be 0.01
float fastnessMultiplier = 0.01;     // multiplier
int transdur = 200; // time taken to transition in ms

int servo1min = 45;
int servo1max = 135;
int servo2min = 110;
int servo2max = 180;
int servo3min = 5;
int servo3max = 90;
