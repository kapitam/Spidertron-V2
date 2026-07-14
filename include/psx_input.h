#pragma once

#include <PSX.h>

// High-level motion request extracted from the operator input. This is the
// same shape of command a ROS teleop topic would provide, so main.cpp does
// not care where it came from.
struct MoveCommand {
  bool active;        // false = stick in deadzone, robot should rest
  double headingRad;  // direction of travel
  double magnitude;   // 0-2 (full deflection on both axes ~= 1.4)
};

class PsxInput {
public:
  void begin();
  MoveCommand readMove();

private:
  PSX psx;
  PSX::PSXDATA data;
};
