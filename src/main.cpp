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
bool preAutonRunning = false; // true while comp() is still running pre_auton

void pre_auton() {
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
    int deadband = 5;
    int liftSpeed = 70; // May be adjusted up to 60% (in the range 40-60, depending on driver preference)
    bool outtakeHolding = false; // true while the outtake is spinning to keep a pin
    bool lastButtonA = false; // used to toggle once per press of A

    while (true) {
        // --- READ JOYSTICKS & SCALE TO 75% MAX SPEED ---
        int forwardSpeed = static_cast<int>(Controller1.Axis3.position() * 0.75); // y Axis (max 100, min -100) -> 0.75 to maintain control over the robot and not have a volatile shift over a small touch.
        int turnSpeed    = static_cast<int>(Controller1.Axis1.position() * 0.75); // x axis

        // Telemetry display on Controller Screen
        // TEMP dr4b debug: each lift motor's angle (line 1) and current draw in amps (line 2)
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("R:%.0f L:%.0f      ", dr4bR.position(rotationUnits::deg), dr4bL.position(rotationUnits::deg));

        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("RA:%.1f LA:%.1f    ", dr4bR.current(currentUnits::amp), dr4bL.current(currentUnits::amp));

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
        
        // dr4b
        // TEMP direction test: Up arrow spins only dr4bR forward, Right arrow spins only dr4bL forward.
        // Tap briefly. Each should move its side of the lift UP. Remove once the lift works.
        if (Controller1.ButtonUp.pressing()) {
            dr4bR.setVelocity(20, percent);
            dr4bR.spin(forward);
            dr4bL.stop();
        }

        else if (Controller1.ButtonRight.pressing()) {
            dr4bL.setVelocity(20, percent);
            dr4bL.spin(forward);
            dr4bR.stop();
        }

        else if (Controller1.ButtonR1.pressing()) {
            dr4b.setVelocity(liftSpeed, percent); // makes sure that the dr4b only goes to a max of 40%
            dr4b.spin(forward);
        }

        else if (Controller1.ButtonR2.pressing()) {
            dr4b.setVelocity(liftSpeed, percent);
            dr4b.spin(reverse);
        }


        else {
            // Hold position actively against gravity even if the controller is not being physically touched at the moment
            dr4b.stop();
        }
        

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

        // outtake: A toggles holding a pin, X scores it
        bool buttonA = Controller1.ButtonA.pressing();
        if (buttonA && !lastButtonA) {
            outtakeHolding = !outtakeHolding;
        }
        lastButtonA = buttonA;

        if (Controller1.ButtonX.pressing()) {
            outtakeHolding = false; // stays stopped after scoring so the pin isn't pulled back in
            outtake.setVelocity(100, percent);
            outtake.spin(forward);
        }

        else if (outtakeHolding) {
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
    // TEMP: shows which build is running, so an old program can't be mistaken for this one
    Brain.Screen.setFont(fontType::mono40);
    Brain.Screen.printAt(20, 80, "LIFT TEST BUILD");
    Controller1.Screen.clearScreen();

    // Driver control only (no competition switch or field needed):
    // comment out comp() below and uncomment these two lines.
    pre_auton();
    driver_control();

    // comp();
    return 0;
}
