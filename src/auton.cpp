#include "vex.h"
#include "auton.h"

using namespace vex;

// ===================== TUNABLES (set these on the field) =====================
static const double TOP_DEG = 720.0;     // lift top soft limit (deg). Keep in sync with LIFT_TOP_DEG in main.cpp
static const double BOTTOM_DEG = 0.0;    // lift bottom target (deg). 0 = where it started, so spinToPosition can always finish
                                         // (-1 like driver control could stall against the hardstop until LIFT_TIMEOUT_MS)
static const int LIFT_SPEED = 100;       // percent. 4 lift moves at 60% would blow past the 15 s auton
static const int LIFT_TIMEOUT_MS = 4500; // longer than a full lift move, so only a stall hits it
static const int LIFT_START_MS = 300;    // how long the lift goes up at the start of auton, before driving
static const int AIM_SPEED = 50;         // percent
static const int DRIVE_SPEED = 60;       // percent
static const int TURN_SPEED = 60;        // percent

static const double AIM_PICKUP_DEG = 0.0; // 0 = where it starts, so start at the pickup angle
static const int LOAD_MS = 500;           // time at the bottom for a pin to reach the outtake
static const int SCORE_MS = 400;          // time the outtake spins the opposite way to score
static const int OUTTAKE_SPEED = 60;      // percent
// Swap these two if the outtake pushes the pin out at pickup
static const directionType OUTTAKE_HOLD_DIR = directionType::rev;
static const directionType OUTTAKE_SCORE_DIR = directionType::rev;

// ===================== ROUTINES (one per starting spot) =====================
// Each starting spot gets its own numbers, so tune each one on the field.
// Distances are in inches. Headings: 0 = direction the robot starts facing, clockwise is positive.
// The intake is on the back, so the robot starts with its back facing the pins.
// The outtake is on the front, so both goals are approached driving forward.
// Aim angles are outtake_move degrees: read them off the controller screen (line 3) while aiming with Y/B.
// Right side starts are the left side mirrored, so their headings are flipped.
struct AutonRoutine {
  const char *name;
  double toPinsIn;       // start -> pin line, driven in reverse (intake first), then turn to goal 1
  double toGoal1In;      // start -> goal 1, driving forward (max distance; frontSensor stops it sooner)
  double backOffIn;      // reverse off goal 1 so the lift clears it
  double goal1ToGoal2In; // goal 1 -> goal 2
  double parkBackIn;     // final reposition off the wall
  double headingGoal1;
  double headingGoal2;
  double aimGoal1Deg;
  double aimGoal2Deg;
  int goal1Tag;          // AprilTag ID on goal 1 (-1 = any tag, nearest the center). Read IDs off the brain in driver control
  int goal2Tag;          // AprilTag ID on goal 2
};

//                name          toPins toGoal1 backOff g1->g2 park  hdgG1   hdgG2  aimG1 aimG2 tagG1 tagG2
static const AutonRoutine ROUTINES[] = {
  {"Red Left",   40.5,  30.0,   6.0,   60.0, 12.0,  90.0,  -90.0,  0.0,  0.0,  -1,   -1},
  {"Red Right",  40.5,  30.0,   6.0,   60.0, 12.0, -90.0,   90.0,  0.0,  0.0,  -1,   -1},
  {"Blue Left",  40.5,  30.0,   6.0,   60.0, 12.0,  90.0,  -90.0,  0.0,  0.0,  -1,   -1},
  {"Blue Right", 40.5,  30.0,   6.0,   60.0, 12.0, -90.0,   90.0,  0.0,  0.0,  -1,   -1},
};
static const int NUM_ROUTINES = sizeof(ROUTINES) / sizeof(ROUTINES[0]);

// Routine used when nothing was picked and the robot is NOT on a real field
// (Timed Run, or a competition switch). Timed Run enables the robot right away,
// so there's no time to tap the picker. On a real field, no pick = no auton.
// 0 = Red Left, 1 = Red Right, 2 = Blue Left, 3 = Blue Right
static const int TIMED_RUN_ROUTINE = 0;

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

// ---- Sensors (all optional: if one isn't plugged in, auton falls back to timing/encoders) ----
// Tune these with the live readouts on the brain screen during driver control.
static const double PIN_DETECT_MM = 60.0; // pinSensor reads less than this when a pin is in the outtake
static const int LOAD_TIMEOUT_MS = 1500;  // max wait for a pin to reach the outtake
static const double GOAL_SCORE_IN = 6.0;  // frontSensor distance to a goal when lined up to score

// ---- AI Vision goal line-up (forward-facing camera on a side bar, AprilTags on the goals) ----
// The camera image is 320 px wide. These are the tag's x when the outtake is lined up on the goal.
// The camera is on a side bar, off to one side of the outtake, so the right x changes with distance:
// set each one with the robot parked where it does that line-up (just after turning toward
// that goal), aimed at the goal by hand. Read "Tag ... x:" off line 2 of the controller.
static const int TAG_TARGET_X_GOAL1 = 160;
static const int TAG_TARGET_X_GOAL2 = 160;
static const double TAG_TURN_KP = 0.25;       // percent turn speed per pixel off center
static const double TAG_MIN_SPEED = 5.0;      // percent
static const double TAG_MAX_SPEED = 30.0;     // percent
static const int TAG_TOLERANCE_PX = 6;
static const int TAG_SETTLE_MS = 100;
static const int TAG_FIND_MS = 300;           // give up if no tag is seen this long (camera blocked/unplugged)
static const int TAG_LINEUP_TIMEOUT_MS = 800;

// Globals defined in main.cpp
extern brain Brain;
extern controller Controller1;
extern competition Competition;
extern motor outtake_move;
extern motor outtake;
extern motor intake;
extern motor_group dr4b;
extern motor_group motorGroupR;
extern motor_group motorGroupL;
extern inertial imu;
extern distance pinSensor;
extern distance frontSensor;
extern aivision aiVision;

// ---- Pin possession (SG6: a robot may hold at most one pin) ----
static bool intakeWanted = false;

static bool hasPin() {
  return pinSensor.installed() && pinSensor.objectDistance(mm) < PIN_DETECT_MM;
}

// Runs the intake only while it's wanted AND the outtake is empty, so the robot
// never ends up holding 2 pins. Called every loop in the helpers below.
// Without a pin sensor, hasPin() is always false and the intake just follows intakeWanted.
static void updateIntake() {
  if (intakeWanted && !hasPin()) {
    intake.spin(forward, 100, percent);
  } else {
    intake.stop();
  }
}

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
    updateIntake();
    wait(LOOP_MS, msec);
  }
  stopDrive();
}

// Drives a distance while holding a heading. Negative inches drive in reverse.
// stopAtGoal: when driving forward, stop GOAL_SCORE_IN from whatever frontSensor sees.
// The encoder distance is still the max, so set it a few inches past the goal.
static void driveStraight(double inches, double headingDeg, int timeoutMs, bool stopAtGoal = false) {
  motorGroupL.resetPosition();
  motorGroupR.resetPosition();

  timer elapsed;
  double direction = inches < 0 ? -1.0 : 1.0;
  double target = fabs(inches);
  double rampSpeed = DRIVE_START_SPEED;

  while (elapsed.time(msec) < timeoutMs) {
    double motorRevs = (motorGroupL.position(rev) + motorGroupR.position(rev)) / 2.0;
    double remaining = target - direction * motorRevs * IN_PER_MOTOR_REV;
    if (stopAtGoal && frontSensor.installed() && frontSensor.isObjectDetected()) {
      remaining = fmin(remaining, frontSensor.objectDistance(distanceUnits::in) - GOAL_SCORE_IN);
    }
    if (remaining <= DRIVE_TOLERANCE_IN) break;

    double speed = DRIVE_KP * remaining;
    if (speed > rampSpeed) speed = rampSpeed;
    if (speed > DRIVE_SPEED) speed = DRIVE_SPEED;
    if (speed < DRIVE_MIN_SPEED) speed = DRIVE_MIN_SPEED;
    rampSpeed += DRIVE_RAMP_PER_LOOP;

    setDrive(direction * speed, HEADING_KP * (headingDeg - imu.rotation(degrees)));
    updateIntake();
    wait(LOOP_MS, msec);
  }
  stopDrive();
}

// ---- AI Vision helpers ----
static bool tagResults[2] = {false, false}; // did each goal's line-up work
static int tagLineUps = 0;

// Finds the AprilTag with this ID (-1 = whichever tag is nearest targetX).
// Returns its x in the camera image, or -1 if it isn't seen.
static int findTagX(int tagId, int targetX) {
  int count = aiVision.takeSnapshot(aivision::ALL_TAGS);
  int bestX = -1;
  for (int i = 0; i < count; i++) {
    int x = aiVision.objects[i].centerX;
    if (tagId >= 0 && aiVision.objects[i].id != tagId) continue;
    if (bestX < 0 || abs(x - targetX) < abs(bestX - targetX)) bestX = x;
  }
  return bestX;
}

// Turns in place until the goal's AprilTag is centered, and returns the heading to drive at.
// If the camera isn't plugged in, can't see the tag, or runs out of time,
// it returns fallbackHeading so auton runs the same as without the camera.
static double lineUpOnTag(int tagId, int targetX, double fallbackHeading) {
  if (!aiVision.installed()) return fallbackHeading;

  timer elapsed;
  int settledMs = 0;
  int lastSeenMs = 0;
  bool seen = false;

  while (elapsed.time(msec) < TAG_LINEUP_TIMEOUT_MS && settledMs < TAG_SETTLE_MS) {
    int x = findTagX(tagId, targetX);
    if (x < 0) {
      setDrive(0, 0); // lost it: stop and wait a moment instead of spinning blind
      settledMs = 0;
      if (elapsed.time(msec) - lastSeenMs > TAG_FIND_MS) break;
    } else {
      seen = true;
      lastSeenMs = elapsed.time(msec);
      double error = x - targetX; // tag right of target -> turn clockwise
      double output = 0;
      if (fabs(error) <= TAG_TOLERANCE_PX) {
        settledMs += LOOP_MS;
      } else {
        settledMs = 0;
        output = TAG_TURN_KP * error;
        if (fabs(output) < TAG_MIN_SPEED) output = copysign(TAG_MIN_SPEED, error);
        if (fabs(output) > TAG_MAX_SPEED) output = copysign(TAG_MAX_SPEED, output);
      }
      setDrive(0, output);
    }
    updateIntake();
    wait(LOOP_MS, msec);
  }
  stopDrive();

  bool linedUp = seen && settledMs >= TAG_SETTLE_MS;
  tagResults[tagLineUps++ % 2] = linedUp; // shown next to the auton time at the end
  return linedUp ? imu.rotation(degrees) : fallbackHeading;
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
    updateIntake();
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
  intakeWanted = true; // stays on for the auton, but updateIntake() pauses it while a pin is held
  updateIntake();
  holdPin();
}

// Pins only transfer from the intake with the lift fully down and the
// outtake at its pickup angle, so this has to finish before every lift.
static void loadPin() {
  holdPin();
  liftTo(BOTTOM_DEG);
  aimTo(AIM_PICKUP_DEG);
  waitLift(LIFT_TIMEOUT_MS);

  if (pinSensor.installed()) {
    // Wait until the pin actually arrives instead of guessing a time
    timer elapsed;
    while (!hasPin() && elapsed.time(msec) < LOAD_TIMEOUT_MS) {
      updateIntake();
      wait(LOOP_MS, msec);
    }
  } else {
    wait(LOAD_MS, msec);
  }
  updateIntake(); // pin is in: intake stops so a second pin can't come in
}

// The intake is on the back, so drive in reverse to run it into the pins.
static void gotoPins(const AutonRoutine &r) {
  driveStraight(-r.toPinsIn, 0.0, 2500);
  turnTo(r.headingGoal1, 1200);
}

// ===================== AUTON SELECTOR =====================
// Before the match, tap a routine on the brain screen. The pick shows on the
// brain and the controller. With nothing picked, auton does nothing.
static int selectedRoutine = -1;
static bool autonStarted = false;

// Screen is 480 x 240. Four buttons in a 2 x 2 grid below a status line.
static const int BTN_W = 230, BTN_H = 85, BTN_GAP = 10, BTN_TOP = 50;

static void buttonRect(int i, int &x, int &y) {
  x = BTN_GAP / 2 + (i % 2) * (BTN_W + BTN_GAP);
  y = BTN_TOP + (i / 2) * (BTN_H + BTN_GAP);
}

static void drawSelector() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(mono20);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(black);
  if (selectedRoutine < 0) {
    Brain.Screen.printAt(10, 30, "TAP AN AUTON (none = skip)");
  } else {
    Brain.Screen.printAt(10, 30, true, "Auton: %s", ROUTINES[selectedRoutine].name);
  }

  for (int i = 0; i < NUM_ROUTINES; i++) {
    int x, y;
    buttonRect(i, x, y);
    bool isRed = (i < 2);
    Brain.Screen.setPenColor(i == selectedRoutine ? yellow : white);
    Brain.Screen.setPenWidth(i == selectedRoutine ? 5 : 1);
    Brain.Screen.setFillColor(isRed ? red : blue);
    Brain.Screen.drawRectangle(x, y, BTN_W, BTN_H);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(x + 20, y + BTN_H / 2 + 7, false, "%s", ROUTINES[i].name);
  }
  Brain.Screen.setPenWidth(1);
  Brain.Screen.setFillColor(black);

  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Auton: %-12s", selectedRoutine < 0 ? "NONE" : ROUTINES[selectedRoutine].name);
}

void runAutonSelector() {
  // Timed Run (or a match already underway): don't draw over the auton's screen
  if (autonStarted || Competition.isEnabled()) return;
  drawSelector();
  bool wasPressing = false;
  while (!autonStarted && !Competition.isEnabled()) {
    bool pressing = Brain.Screen.pressing();
    if (pressing && !wasPressing) {
      int tx = Brain.Screen.xPosition();
      int ty = Brain.Screen.yPosition();
      for (int i = 0; i < NUM_ROUTINES; i++) {
        int x, y;
        buttonRect(i, x, y);
        if (tx >= x && tx < x + BTN_W && ty >= y && ty < y + BTN_H) {
          selectedRoutine = i;
          drawSelector();
        }
      }
    }
    wasPressing = pressing;
    wait(LOOP_MS, msec);
  }
}

// ===================== AUTONOMOUS =====================
// Being rebuilt one step at a time. Right now: raise the lift briefly, drive forward to the first goal, and score the preloaded pin.
// The lift and outtake_move must start in the same position every match.
void runAuton() {
  int routine = selectedRoutine;
  // Only a real field skips when nothing is picked. The brain's Timed Run may report
  // itself as a competition switch, so a switch isn't treated as a match.
  bool timedRunDefault = (routine < 0 && !Competition.isFieldControl());
  if (timedRunDefault) routine = TIMED_RUN_ROUTINE;

  autonStarted = true; // stops the selector so it doesn't draw over auton output

  
  const AutonRoutine &r = ROUTINES[routine];
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(10, 20, true, "Auton: %s%s", r.name, timedRunDefault ? " (default, nothing picked)" : "");
  timer autonTimer;

  waitUntil(!imu.isCalibrating());
  imu.setRotation(0, degrees);
  dr4b.resetPosition();
  outtake_move.resetPosition();
  dr4b.setStopping(hold);
  outtake_move.setStopping(hold);
  motorGroupL.setStopping(brake);
  motorGroupR.setStopping(brake);

  // Start: facing forward (outtake facing out) with a pin already in the outtake.

  // --- Raise the lift a little before moving ---
  outtake_move.spin(forward, LIFT_SPEED, percent);
  wait(LIFT_START_MS, msec);
  outtake_move.stop(); // holds there (stopping mode is hold)

  // --- Drive forward to the first goal ---
  // Straight ahead (heading 0). If frontSensor is plugged in, it stops GOAL_SCORE_IN from the goal.
  driveStraight(10, 0.0, 1000, true);

  // --- Score the first pin ---
  outtake.spin(reverse, 100, percent);

  Brain.Screen.printAt(10, 40, "Auton time: %.2f s", autonTimer.time(msec) / 1000.0);
}
