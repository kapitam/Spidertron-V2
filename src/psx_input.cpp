#include "psx_input.h"

#include <math.h>

#include "config.h"

void PsxInput::begin() {
  psx.setupPins(PSX_DATA_PIN, PSX_CMD_PIN, PSX_ATT_PIN, PSX_CLOCK_PIN, 10);
  psx.config(PSXMODE_ANALOG);
}

MoveCommand PsxInput::readMove() {
  int error = psx.read(data);
  if (error != PSXERROR_SUCCESS) {
    Serial.print("No success reading data. Check connections and timing.");
  }

  // Center the sticks around 0 (raw range is 0-255).
  int x = data.JoyLeftX - 128;
  int y = data.JoyLeftY - 128;

#if DEBUG_LOG
  Serial.print("LeftX: ");
  Serial.print(x);
  Serial.print(" LeftY: ");
  Serial.println(y);
#endif

  MoveCommand cmd;
  cmd.active = (abs(x) + abs(y)) >= STICK_DEADZONE;
  cmd.headingRad = atan2(y, x);
  cmd.magnitude = sqrt(x * x + y * y) / 128.0;
  return cmd;
}
