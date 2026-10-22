#include "vex.h"
#include "auton.h"

using namespace vex;

// ===================== TUNABLES (set these on the field) =====================
static const double TOP_DEG = 540.0;     // lift top soft limit (deg)
static const double BOTTOM_DEG = -1.0;   // lift bottom soft limit (deg)
static const int LIFT_SPEED = 60;        // percent, auton can push harder
static const int DRIVE_SPEED = 60;       // percent

// Waypoints: measure these on the real field, in order.
static const double TO_GOAL1_IN = 24.0;  // start -> goal 1
static const double GOAL1_TO_GOAL2_IN = 30.0; // goal 1 -> goal 2
static const double PARK_BACK_IN = 12.0; // final reposition off the wall

// TODO: define your intake/outtake motors in main.cpp, then uncomment:
// extern motor intake;
// extern motor_group outtake;

// Globals defined in main.cpp
extern motor dr4bR;
extern motor dr4bL;
extern motor_group dr4b;
extern drivetrain drive;

// ---- Lift helpers ----
static void raiseToTop() {
  dr4b.setVelocity(LIFT_SPEED, percent);
  dr4b.spinToPosition(TOP_DEG, rotationUnits::deg, true);
}

static void lowerToBottom() {
  dr4bR.setVelocity(LIFT_SPEED, percent);
  dr4bL.setVelocity(LIFT_SPEED, percent);
  dr4bR.spin(reverse);
  dr4bL.spin(reverse);
  while (dr4bR.position(rotationUnits::deg) > BOTTOM_DEG ||
         dr4bL.position(rotationUnits::deg) > BOTTOM_DEG) {
    if (dr4bR.position(rotationUnits::deg) <= BOTTOM_DEG) dr4bR.stop();
    if (dr4bL.position(rotationUnits::deg) <= BOTTOM_DEG) dr4bL.stop();
    wait(20, msec);
  }
  dr4b.stop(); // hold
}

// ---- Scoring helpers (fill in once intake motors exist) ----
static void scorePins(int count) {
  // Placeholder: spin outtake to drop `count` pins into the goal.
  wait(200, msec); // remove when real scoring is in
}

static void collectPins() {
  // Placeholder: run intake while driving over pins.
  wait(200, msec); // remove when real intake is in
}

// ===================== AUTONOMOUS =====================
// Strategy: Autonomous Win Point needs 6+ pin halves scored on our side,
// 2+ goals with 2+ pins each, and no robot touching the wall at the end.
// Auton is 15 seconds: keep every segment tight.
void runAuton() {
  dr4b.setStopping(hold);
  drive.setDriveVelocity(DRIVE_SPEED, percent);
  drive.setTurnVelocity(DRIVE_SPEED, percent);

  // --- Goal 1: drive, lift, score 3 pins ---
  collectPins();
  drive.driveFor(forward, TO_GOAL1_IN, inches, true);
  raiseToTop();
  scorePins(3);
  lowerToBottom();

  // --- Goal 2: drive, lift, score 3 pins ---
  drive.driveFor(forward, GOAL1_TO_GOAL2_IN, inches, true);
  raiseToTop();
  scorePins(3);
  lowerToBottom();

  // --- Park: back off the wall so AWP condition 3 holds ---
  drive.driveFor(reverse, PARK_BACK_IN, inches, true);
}
