#include <PSX.h>

#define DATA_PIN   25   // White wire
#define CMD_PIN    26  // Black wire
#define ATT_PIN    32  // Gray wire
#define CLOCK_PIN  33  // black wire

PSX psx;

// Variables to hold the controller state and error code
PSX::PSXDATA PSXdata;
int PSXerror;

// Struct Direction and Mag
struct Dir {
  double mag;
  double rad;
};