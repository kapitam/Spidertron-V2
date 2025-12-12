// MAIN CODE EDIT, PLEASE CHECK Dwag

const double J2L = 10.7;  // Length of the femur
const double J3L = 16.8;  // Length of the tibia
const double D = 6.5;     // Offset length from the start of femur to axis
const double legOffset = 9.83;  // From center of hexapod

// Adjustable variables
const float REST_DISTANCE = 17.0;   // Distance from center for rest position (adjust as needed)
const float GROUND_Z = -8.0;          // Z coordinate for legs on the ground
const float AIR_Z = 0.0;            // Z coordinate for legs in the air

// this is calculated with the magnitude function range of 0-2, the units in cm
// the speed at dx and dy is updated is in 10ms, therefore to get speed of 0-2 cm/s this must be 0.01
float fastnessMultiplier = 0.1;      // multiplier
int transdur = 200; // time taken to transition in ms