#include "vex.h"
#include "auton.h"

using namespace vex;

// ===================== TUNABLES (set these on the field) =====================
static const double TOP_DEG = 540.0;     // lift top soft limit (deg)
static const double BOTTOM_DEG = 0.0;   // lift bottom soft limit (deg)
static const int LIFT_SPEED = 60;        // percent, auton can push harder
static const int DRIVE_SPEED = 60;       // percent

// Waypoints: measure these on the real field, in order.
static const double TO_GOAL1_IN = 30.0;  // start -> goal 1
static const double GOAL1_TO_GOAL2_IN = 60.0; // goal 1 -> goal 2
static const double PARK_BACK_IN = 12.0; // final reposition off the wall

// TODO: define your intake/outtake motors in main.cpp, then uncomment:
// extern motor intake;
// extern motor_group outtake;

// Globals defined in main.cpp
extern motor dr4bR;
extern motor dr4bL;
extern motor outtake;
extern motor intake;
extern motor_group dr4b;
extern motor_group motorGroupR;
extern motor_group motorGroupL;
extern drivetrain drive;

// ---- Lift helpers ----
static void raiseToTop() {
  dr4b.setVelocity(LIFT_SPEED, percent);
  dr4b.spinToPosition(TOP_DEG, rotationUnits::deg, true);
}

static void lowerToBottom() {
  dr4b.setVelocity(LIFT_SPEED, percent);
  dr4b.spinToPosition(BOTTOM_DEG, rotationUnits::deg, true);
  dr4b.stop(hold);
}

// ---- Scoring helpers (fill in once intake motors exist) ----
static void scorePins(int count) {
  outtake.setVelocity(100, percent);
  outtake.spin(forward);
  wait(200*count,msec);
  outtake.stop();
}

static void collectPins() {
  intake.setVelocity(100, percent);
  intake.spin(forward); // No wait-time because intake is continuously moving throughout the autonomous period.
}
static void gotoPins() {
  drive.driveFor(forward, 40.5, inches, true);
  drive.turnFor(left,90,degrees,true);
}

// }
// ===================== AUTONOMOUS =====================
// Strategy: Autonomous Win Point needs 6+ pin halves scored on our side,
// 2+ goals with 2+ pins each, and no robot touching the wall at the end.
// Auton is 15 seconds: keep every segment tight.
void runAuton() {
  dr4b.resetPosition();
  dr4b.setStopping(hold);
  drive.setDriveVelocity(DRIVE_SPEED, percent);
  drive.setTurnVelocity(DRIVE_SPEED, percent);

  // --- Goal 1: drive, lift, score 3 pins ---
  collectPins();
  gotoPins();
  drive.driveFor(forward, TO_GOAL1_IN, inches, true);
  raiseToTop();
  scorePins(3);

  dr4b.setVelocity(LIFT_SPEED, percent);
  dr4b.spinToPosition(BOTTOM_DEG, rotationUnits::deg, false);
  drive.turnFor(right,180,degrees,true);

  drive.driveFor(forward, GOAL1_TO_GOAL2_IN, inches, true);
  dr4b.spinToPosition(TOP_DEG, rotationUnits::deg, true);
  scorePins(3);

  lowerToBottom();

  // --- Park: back off the wall so AWP condition 3 holds ---
  drive.driveFor(reverse, PARK_BACK_IN, inches, true);
}