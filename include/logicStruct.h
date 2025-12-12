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

// Global leg objects labeled leg1 ... leg6
Leg leg1, leg2, leg3, leg4, leg5, leg6;

// Movement logic
bool legnowblue = false;
bool constraint = false;