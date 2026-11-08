#include "vex.h"
#include "auton.h"

using namespace vex;

// ===================== TUNABLES (set these on the field) =====================
static const double TOP_DEG = 1280.0;    // lift top soft limit (deg)
static const double BOTTOM_DEG = 0.0;   // lift bottom soft limit (deg)
static const int LIFT_SPEED = 60;        // percent, auton can push harder
static const int LIFT_TIMEOUT_MS = 4500; // longer than a full lift move, so only a stall hits it
static const int AIM_SPEED = 50;         // percent
static const int DRIVE_SPEED = 60;       // percent
static const int TURN_SPEED = 60;        // percent

// Outtake aim, in outtake_move degrees from its starting position.
// Read these off the controller screen (line 3) while aiming with Y/B.
static const double AIM_GOAL1_DEG = 0.0;
static const double AIM_GOAL2_DEG = 0.0;
static const double AIM_PICKUP_DEG = 0.0; // 0 = where it starts, so start at the pickup angle
static const int LOAD_MS = 500;           // time at the bottom for a pin to reach the outtake
static const int SCORE_MS = 400;          // time the outtake spins the opposite way to score
static const int OUTTAKE_SPEED = 60;      // percent
// Swap these two if the outtake pushes the pin out at pickup
static const directionType OUTTAKE_HOLD_DIR = directionType::rev;
static const directionType OUTTAKE_SCORE_DIR = directionType::fwd;

// Waypoints: measure these on the real field, in order.
static const double TO_PINS_IN = 40.5;   // start -> pin line, then turn to goal 1
static const double TO_GOAL1_IN = 30.0;  // start -> goal 1
static const double BACK_OFF_IN = 6.0;   // reverse off goal 1 so the lift clears it
static const double GOAL1_TO_GOAL2_IN = 60.0; // goal 1 -> goal 2
static const double PARK_BACK_IN = 12.0; // final reposition off the wall

// Headings: 0 = direction the robot starts facing, clockwise is positive.
// The intake is on the back, so the robot starts with its back facing the pins.
// The outtake is on the front, so both goals are approached driving forward.
static const double HEADING_GOAL1 = 90.0;
static const double HEADING_GOAL2 = -90.0;

// Inches per motor revolution: external gear ratio * wheel travel.
// Confirm with a tape measure by commanding a 48 inch drive.
static const double IN_PER_MOTOR_REV = 0.6 * 8.639;

// Motion control gains
static const double TURN_KP = 0.7;
static const double TURN_KD = 1.0;
static const double TURN_MIN_SPEED = 5.0;     // percent
static const double TURN_TOLERANCE_DEG = 1.5;
static const int TURN_SETTLE_MS = 100;
static const double DRIVE_KP = 5.0;           // percent per inch remaining
static const double DRIVE_MIN_SPEED = 12.0;   // percent
static const double DRIVE_START_SPEED = 15.0; // percent
static const double DRIVE_RAMP_PER_LOOP = 4.0; // percent per loop
static const double DRIVE_TOLERANCE_IN = 0.25;
static const double HEADING_KP = 1.5;         // percent per degree of error
static const int LOOP_MS = 20;

// Globals defined in main.cpp
extern brain Brain;
extern motor outtake_move;
extern motor outtake;
extern motor intake;
extern motor_group dr4b;
extern motor_group motorGroupR;
extern motor_group motorGroupL;
extern inertial imu;

// ---- Drive helpers ----
// Same motor mapping as driver_control: positive rotation is clockwise.
static void setDrive(double forwardSpeed, double clockwiseSpeed) {
  motorGroupL.spin(forward, forwardSpeed - clockwiseSpeed, percent);
  motorGroupR.spin(forward, forwardSpeed + clockwiseSpeed, percent);
}

static void stopDrive() {
  motorGroupL.stop(brake);
  motorGroupR.stop(brake);
}

// Turns to an absolute heading. The sign of targetDeg picks the side,
// because imu.rotation() keeps counting past 360 instead of wrapping.
static void turnTo(double targetDeg, int timeoutMs) {
  timer elapsed;
  double previousError = targetDeg - imu.rotation(degrees);
  int settledMs = 0;

  while (elapsed.time(msec) < timeoutMs && settledMs < TURN_SETTLE_MS) {
    double error = targetDeg - imu.rotation(degrees);
    double output = TURN_KP * error + TURN_KD * (error - previousError);
    previousError = error;

    if (fabs(error) <= TURN_TOLERANCE_DEG) {
      settledMs += LOOP_MS;
      output = 0;
    } else {
      settledMs = 0;
      if (fabs(output) < TURN_MIN_SPEED) output = copysign(TURN_MIN_SPEED, error);
      if (fabs(output) > TURN_SPEED) output = copysign(TURN_SPEED, output);
    }

    setDrive(0, output);
    wait(LOOP_MS, msec);
  }
  stopDrive();
}

// Drives a distance while holding a heading. Negative inches drive in reverse.
static void driveStraight(double inches, double headingDeg, int timeoutMs) {
  motorGroupL.resetPosition();
  motorGroupR.resetPosition();

  timer elapsed;
  double direction = inches < 0 ? -1.0 : 1.0;
  double target = fabs(inches);
  double rampSpeed = DRIVE_START_SPEED;

  while (elapsed.time(msec) < timeoutMs) {
    double motorRevs = (motorGroupL.position(rev) + motorGroupR.position(rev)) / 2.0;
    double remaining = target - direction * motorRevs * IN_PER_MOTOR_REV;
    if (remaining <= DRIVE_TOLERANCE_IN) break;

    double speed = DRIVE_KP * remaining;
    if (speed > rampSpeed) speed = rampSpeed;
    if (speed > DRIVE_SPEED) speed = DRIVE_SPEED;
    if (speed < DRIVE_MIN_SPEED) speed = DRIVE_MIN_SPEED;
    rampSpeed += DRIVE_RAMP_PER_LOOP;

    setDrive(direction * speed, HEADING_KP * (headingDeg - imu.rotation(degrees)));
    wait(LOOP_MS, msec);
  }
  stopDrive();
}

// ---- Lift helpers ----
static void liftTo(double targetDeg) {
  dr4b.setVelocity(LIFT_SPEED, percent);
  dr4b.spinToPosition(targetDeg, rotationUnits::deg, false);
}

static void aimTo(double targetDeg) {
  outtake_move.setVelocity(AIM_SPEED, percent);
  outtake_move.spinToPosition(targetDeg, rotationUnits::deg, false);
}

// Gives up at the timeout so a stalled lift can't use up the whole auton.
static void waitLift(int timeoutMs) {
  timer elapsed;
  while (elapsed.time(msec) < timeoutMs && !(dr4b.isDone() && outtake_move.isDone())) {
    wait(LOOP_MS, msec);
  }
}

// ---- Scoring helpers ----
// The outtake spins one way to pull a pin in and keep it, then the opposite way to score it.
static void holdPin() {
  outtake.setVelocity(OUTTAKE_SPEED, percent);
  outtake.spin(OUTTAKE_HOLD_DIR);
}

static void releasePin() {
  outtake.setVelocity(OUTTAKE_SPEED, percent);
  outtake.spin(OUTTAKE_SCORE_DIR);
  wait(SCORE_MS, msec);
  outtake.stop(); // stays stopped at the goal so the pin isn't pulled back in
}

static void collectPins() {
  intake.setVelocity(100, percent);
  intake.spin(forward); // No wait-time because intake is continuously moving throughout the autonomous period.
  holdPin();
}

// Pins only transfer from the intake with the lift fully down and the
// outtake at its pickup angle, so this has to finish before every lift.
static void loadPin() {
  holdPin();
  liftTo(BOTTOM_DEG);
  aimTo(AIM_PICKUP_DEG);
  waitLift(LIFT_TIMEOUT_MS);
  wait(LOAD_MS, msec);
}

static void gotoPins() {
  driveStraight(-TO_PINS_IN, 0.0, 2500); // reverse so the rear intake leads
  turnTo(HEADING_GOAL1, 1200);
}

// ===================== AUTONOMOUS =====================
// Strategy: score one pin in each of 2 goals on our side, then get off the wall.
// The outtake carries one pin per lift, and a Goal only holds one nested pin (SC2).
// The Autonomous Win Point needs 6+ pin halves, so a partner or Cup stacks must add the rest.
// The lift and outtake_move must start in the same position every match.
void runAuton() {
  timer autonTimer;

  waitUntil(!imu.isCalibrating());
  imu.setRotation(0, degrees);
  levelDr4b();
  dr4b.resetPosition();
  outtake_move.resetPosition();
  dr4b.setStopping(hold);
  outtake_move.setStopping(hold);
  motorGroupL.setStopping(brake);
  motorGroupR.setStopping(brake);

  // --- Goal 1: collect, approach while lifting, score 1 pin ---
  collectPins();
  gotoPins();
  loadPin();
  liftTo(TOP_DEG);
  aimTo(AIM_GOAL1_DEG);
  driveStraight(TO_GOAL1_IN, HEADING_GOAL1, 2000);
  waitLift(LIFT_TIMEOUT_MS);
  releasePin();

  // --- Back off so the lift clears goal 1, then lower during the turn ---
  driveStraight(-BACK_OFF_IN, HEADING_GOAL1, 800);
  liftTo(BOTTOM_DEG);
  aimTo(AIM_PICKUP_DEG);
  turnTo(HEADING_GOAL2, 1800);
  loadPin();

  // --- Goal 2: approach while lifting, score 1 pin ---
  liftTo(TOP_DEG);
  aimTo(AIM_GOAL2_DEG);
  driveStraight(GOAL1_TO_GOAL2_IN - BACK_OFF_IN, HEADING_GOAL2, 3000);
  waitLift(LIFT_TIMEOUT_MS);
  releasePin();

  // --- Park: back off the wall so AWP condition 3 holds ---
  intake.stop(); // stop first so reversing doesn't pull in more pins
  driveStraight(-PARK_BACK_IN, HEADING_GOAL2, 1000);
  liftTo(BOTTOM_DEG);
  aimTo(AIM_PICKUP_DEG);
  waitLift(LIFT_TIMEOUT_MS);

  Brain.Screen.printAt(10, 40, "Auton time: %.2f s", autonTimer.time(msec) / 1000.0);
}
