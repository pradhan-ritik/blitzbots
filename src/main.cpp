/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ritik                                                     */
/*    Created:      9/12/2026, 11:22:44 AM                                    */
/*    Description:  V5 Split-Arcade (Reversed Motors + 75% Max Velocity)     */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "auton.h"

using namespace vex;

competition Competition;

// Global instance of brain
brain Brain;
// Controller
controller Controller1 = controller(primary);

// NOTE: the ports below need to be changed to match how the robot is currently wired
// Motor definitions (Reversed direction booleans)
motor motorR1 = motor(PORT5, ratio6_1, true);
motor motorR2 = motor(PORT14, ratio6_1, true);
motor motorL1 = motor(PORT9, ratio6_1, false);
motor motorL2 = motor(PORT6, ratio6_1, false);
motor dr4bR = motor(PORT16, ratio36_1, false);
motor dr4bL = motor(PORT17, ratio36_1, true);
motor intake = motor(PORT19, ratio18_1, false);
motor outtake = motor(PORT20, ratio18_1, true);
motor outtake_move = motor(PORT10, ratio18_1, false);
inertial imu = inertial(PORT3);
// Motor groups
motor_group motorGroupR = motor_group(motorR1, motorR2);
motor_group motorGroupL = motor_group(motorL1, motorL2);
motor_group dr4b = motor_group(dr4bR, dr4bL);

// Drivetrain setup
drivetrain drive(motorGroupL, motorGroupR, 8.639, 12.75, 8.5, distanceUnits::in, 0.6);
const int DR4B_R_TRIM_MS = 200; // how long dr4bR is raised at startup to level it with dr4bL
bool preAutonRunning = false; // true while comp() is still running pre_auton

// ===================== DR4B DRIVER TUNABLES (motor degrees, same as line 3 "Lift") =====================
const double LIFT_TOP_DEG = 1280.0;      // measured before the 12:84 rebuild: re-measure at the top hardstop
const double LIFT_BOTTOM_DEG = 0.0;      // stop powering down at the bottom hardstop
const double LIFT_REST_ZONE_DEG = 30.0;  // this close to the bottom with no button: unpower and rest on the hardstop
const double LIFT_SLOW_ZONE_DEG = 140.0; // within this many degrees of the bottom, lower slowly
const int LIFT_UP_SPEED = 100;           // percent. Lifting fights gravity
const int LIFT_DOWN_SPEED = 60;          // percent
const int LIFT_DOWN_SLOW_SPEED = 30;     // percent. Near the bottom so it doesn't slam
const int LIFT_MAX_TORQUE = 100;         // percent. Mentor suggested 55-60 to protect motors once the lift is strong enough

// Drive one side of the DR4B. dir: +1 = up, -1 = down, 0 = hold.
// Each side checks its own encoder, so both sides stop at the same height.
void driveLiftSide(motor &m, int dir) {
    double pos = m.position(rotationUnits::deg);
    if (dir > 0 && pos < LIFT_TOP_DEG) {
        m.spin(forward, LIFT_UP_SPEED, percent);
    } else if (dir < 0 && pos > LIFT_BOTTOM_DEG) {
        int speed = (pos < LIFT_BOTTOM_DEG + LIFT_SLOW_ZONE_DEG) ? LIFT_DOWN_SLOW_SPEED : LIFT_DOWN_SPEED;
        m.spin(reverse, speed, percent);
    } else if (pos < LIFT_REST_ZONE_DEG) {
        m.stop(coast); // at the bottom: let it sit on the hardstop so the motors stay cool
    } else {
        m.stop(hold); // anywhere else: hold against gravity
    }
}

// Levels the right side once. Motors can't move while the field has the robot
// disabled, so if pre_auton is too early this runs at the start of auton instead.
void levelDr4b() {
  static bool leveled = false;
  if (leveled || !Competition.isEnabled()) return;

  dr4b.setStopping(hold);
  dr4bR.setVelocity(20, percent);
  dr4bR.spin(forward);
  wait(DR4B_R_TRIM_MS, msec);
  dr4bR.stop();
  leveled = true;
}

void pre_auton() {
  // Level the right side first, so both sides zero at the same height
  levelDr4b();

  // Reset motor positions and set stopping modes
  dr4b.resetPosition();
  motorGroupL.setStopping(coast);
  motorGroupR.setStopping(coast);
  outtake_move.setStopping(hold);

  // Robot must stay still for about 2 seconds while this runs
  imu.calibrate();
  while (imu.isCalibrating()) {
    wait(20, msec);
  }
}

void driver_control() {
    // With no field connected this starts right away, so let pre_auton finish first
    while (preAutonRunning) {
        wait(20, msec);
    }

    motorGroupL.setStopping(coast); // not a sudden stop, but moves forward with the momentum it has before coming to a rest.
    motorGroupR.setStopping(coast); // not a sudden stop, but moves forward with the momentum it has before coming to a rest.
    dr4b.setStopping(hold); // if not being adjusted by the controller, the dr4b will not fall due to gravity, rather hold.
    dr4b.setMaxTorque(LIFT_MAX_TORQUE, percent);
    int deadband = 5;

    while (true) {
        // --- READ JOYSTICKS & SCALE TO 75% MAX SPEED ---
        int forwardSpeed = static_cast<int>(Controller1.Axis3.position() * 0.75); // y Axis (max 100, min -100) -> 0.75 to maintain control over the robot and not have a volatile shift over a small touch.
        int turnSpeed    = static_cast<int>(Controller1.Axis1.position() * 0.75); // x axis

        // Telemetry display on Controller Screen
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("Fwd Vel:  %d%%   ", forwardSpeed); // move to line 1 to print fwd motion for testing

        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("Turn Vel: %d%%   ", turnSpeed); // move to line 2 to print fwd motion for testing

        Controller1.Screen.setCursor(3, 1);
        Controller1.Screen.print("Aim:%.0f Lift:%.0f   ", outtake_move.position(rotationUnits::deg), dr4b.position(rotationUnits::deg)); // line 3 is for finding the auton aim and lift angles

        // Split-Arcade calculation
        int leftVelocity  = forwardSpeed - turnSpeed;
        int rightVelocity = forwardSpeed + turnSpeed;
        // This is known as Split-Arcade, rather than Tank. 
        // They can both used but are for different driver preferences. 
        // The Tank uses one axis to move right and one to move left. 
        // However, the split-arcade uses one axis for the direction and the other for the speed.
        // Easier to understand conceptually considering velocity in the frame of a vector instead.

        // Clamp total combined motor velocity strictly between -75% and +75%
        if (leftVelocity < -75)   leftVelocity = -75;
        if (leftVelocity > 75)  leftVelocity = 75;
        if (rightVelocity < -75)  rightVelocity = -75;
        if (rightVelocity > 75) rightVelocity = 75;

        // sets max and min to 75 and -75 respectively from the original extremes of 100 and -100. Mainly to prevent damage and other issues.

        // --- DRIVE LEFT MOTORS ---
        if (abs(leftVelocity) < deadband) {
            motorGroupL.stop();
        } else {
            motorGroupL.setVelocity(leftVelocity, percent);
            motorGroupL.spin(forward);
        }
        

        // --- DRIVE RIGHT MOTORS ---
        if (abs(rightVelocity) < deadband) {
            motorGroupR.stop();
        } else {
            motorGroupR.setVelocity(rightVelocity, percent);
            motorGroupR.spin(forward);
        }
        // deadband (currently set to 5) ensures that a negligible touch on the controller will not impact the position of the robot.
        
        // --- DR4B: R1 = up, R2 = down, neither = hold ---
        int liftDir = 0;
        if (Controller1.ButtonR1.pressing()) liftDir = 1;
        else if (Controller1.ButtonR2.pressing()) liftDir = -1;
        driveLiftSide(dr4bR, liftDir);
        driveLiftSide(dr4bL, liftDir);

        // Brain screen: lift diagnostics. ~2.5 A while stuck = motors maxed out (mechanical problem).
        Brain.Screen.printAt(10, 40, "R  pos %5.0f  amps %4.2f  temp %3.0fC   ", dr4bR.position(rotationUnits::deg), dr4bR.current(currentUnits::amp), dr4bR.temperature(temperatureUnits::celsius));
        Brain.Screen.printAt(10, 70, "L  pos %5.0f  amps %4.2f  temp %3.0fC   ", dr4bL.position(rotationUnits::deg), dr4bL.current(currentUnits::amp), dr4bL.temperature(temperatureUnits::celsius));


        // intake
        if (Controller1.ButtonL1.pressing()) {
            intake.setVelocity(100, percent);
            intake.spin(forward);
        }
        else if (Controller1.ButtonL2.pressing()) {
            intake.setVelocity(100, percent);
            intake.spin(reverse);
        } else {
            intake.stop();
        }

        // outtake: hold X to score a pin, hold A to keep a pin in. Both stop when released.
        if (Controller1.ButtonX.pressing()) {
            outtake.setVelocity(100, percent);
            outtake.spin(forward);
        }

        else if (Controller1.ButtonA.pressing()) {
            outtake.setVelocity(100, percent);
            outtake.spin(reverse);
        }

        else {
            outtake.stop();
        }

        if (Controller1.ButtonY.pressing()) {
            outtake_move.setVelocity(100, percent);
            outtake_move.spin(forward);
        }

        else if (Controller1.ButtonB.pressing()) {
            outtake_move.setVelocity(100, percent);
            outtake_move.spin(reverse);
        }

        else {
            outtake_move.stop();
        }

        // Brief delay to give time to the brain scheduler
        wait(20, msec); // basically used to not overload the brain with too many commands at one given moment in time
    }
}

void comp() {
    preAutonRunning = true;
    Competition.autonomous(runAuton);
    Competition.drivercontrol(driver_control);
    
    // Run the pre-autonomous function
    pre_auton();
    preAutonRunning = false;
    
    // Prevent main from exiting with an infinite loop
    while (true) {
        wait(100,msec);
    }
}

int main() {
    // Driver control only (no competition switch or field needed):
    // comment out comp() below and uncomment these two lines.
    // pre_auton();
    // driver_control();

    comp();
    return 0;
}
