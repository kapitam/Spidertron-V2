#pragma once

#include <Arduino.h>

// ===================== Debug =====================

// Optional serial logging (controller CSV, gait status). Constraint trips
// are always printed.
#define DEBUG_LOG 1

// ===================== PSX controller wiring =====================

constexpr int PSX_DATA_PIN  = 25;  // white wire
constexpr int PSX_CMD_PIN   = 26;  // black wire
constexpr int PSX_ATT_PIN   = 32;  // gray wire
constexpr int PSX_CLOCK_PIN = 33;  // blue wire

// Stick deflection (|x| + |y|, raw units) below which the robot rests.
constexpr int STICK_DEADZONE = 15;

// ===================== Servo hardware =====================

constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;

constexpr int SERVO_MIN_PULSE_US = 450;
constexpr int SERVO_MAX_PULSE_US = 2500;
constexpr int PWM_FREQ_HZ        = 50;

// One PCA9685 per side of the body.
constexpr uint8_t PCA_RIGHT_ADDR = 0x40;  // legs 0-2 (front/center/back right)
constexpr uint8_t PCA_LEFT_ADDR  = 0x41;  // legs 3-5 (front/center/back left)

// ===================== Body geometry (cm) =====================

constexpr int    NUM_LEGS           = 6;
constexpr double LEG_MOUNT_STEP_DEG = 60.0;  // angular spacing between legs

constexpr double FEMUR_LENGTH = 8.8;
constexpr double TIBIA_LENGTH = 15.5;
constexpr double COXA_OFFSET  = 5.5;  // from start of femur to its axis

// Joint angle limits (degrees). A pose that violates any of these is
// unreachable and, while walking, triggers a tripod swap.
constexpr double A1_MIN = 45,   A1_MAX = 135;  // coxa
constexpr double A2_MIN = 110,  A2_MAX = 180;  // femur
constexpr double A3_MIN = 5,    A3_MAX = 90;   // tibia

// ===================== Gait tuning =====================

constexpr float REST_DISTANCE = 21.0;  // foot distance from center at rest
constexpr float GROUND_Z      = -3.0;  // Z of feet on the ground
constexpr float AIR_Z         = 0.0;   // Z of feet in the air (swing)
constexpr int   TRANSITION_MS = 200;   // pause when tripods swap

// Stick magnitude is 0-2; positions update every ~10 ms, so 0.01 gives a
// walking speed of roughly 0-2 cm/s.
constexpr float STEP_SCALE = 0.01;
