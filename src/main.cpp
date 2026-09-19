/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ritik                                                     */
/*    Created:      9/12/2026, 11:22:44 AM                                    */
/*    Description:  V5 Split-Arcade (Reversed Motors + 75% Max Velocity)     */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// Global instance of brain
brain Brain;

// Controller
controller Controller1 = controller(primary);

// Motor definitions (Reversed direction booleans)
motor motorR1 = motor(PORT5, ratio6_1, true);
motor motorR2 = motor(PORT20, ratio6_1, true);
motor motorL1 = motor(PORT9, ratio6_1, false);
motor motorL2 = motor(PORT6, ratio6_1, false);
motor dr4bR = motor(PORT16, ratio36_1, false);
motor dr4bL = motor(PORT17, ratio36_1, true);

// Motor groups
motor_group motorGroupR = motor_group(motorR1, motorR2);
motor_group motorGroupL = motor_group(motorL1, motorL2);
motor_group dr4b = motor_group(dr4bR, dr4bL);

// Drivetrain setup
drivetrain drive(motorGroupL, motorGroupR, 8.639, 12.75, 8.5, distanceUnits::in, 0.6);

int main() {
    motorGroupL.setStopping(coast); // not a sudden stop, but moves forward with the momentum it has before coming to a rest.
    motorGroupR.setStopping(coast); // not a sudden stop, but moves forward with the momentum it has before coming to a rest.
    dr4b.setStopping(hold); // if not being adjusted by the controller, the dr4b will not fall due to gravity, rather hold.
    int deadband = 5;
    int liftSpeed = 40; // May be adjusted up to 60% (in the range 40-60, depending on driver preference)
    double BOTTOM_LIMIT_DEG = -1.0; // Deg threshold near bottom rest for dr4b
    double TOP_LIMIT_DEG = 540.0;  // Deg threshold near top extension for dr4b

    while (true) {
        // --- READ JOYSTICKS & SCALE TO 75% MAX SPEED ---
        int forwardSpeed = static_cast<int>(Controller1.Axis3.position() * 0.75); // y Axis (max 100, min -100) -> 0.75 to maintain control over the robot and not have a volatile shift over a small touch.
        int turnSpeed    = static_cast<int>(Controller1.Axis1.position() * 0.75); // x axis

        // Telemetry display on Controller Screen
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("Fwd Vel:  %d%%   ", forwardSpeed); // move to line 1 to print fwd motion for testing

        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("Turn Vel: %d%%   ", turnSpeed); // move to line 2 to print fwd motion for testing

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
        double currentPosition = dr4b.position(rotationUnits::deg);

        if (Controller1.ButtonR1.pressing()) {
            if (currentPosition >= TOP_LIMIT_DEG) {
                dr4b.stop(); // Prevent moving up past maximum height in degrees
            } else {
                dr4b.setVelocity(liftSpeed, percent); // makes sure that the dr4b only goes to a max of 40%
                dr4b.spin(forward);
            }
        } 

        else if (Controller1.ButtonR2.pressing()) {
            if (currentPosition <= BOTTOM_LIMIT_DEG) {
                dr4b.stop(); // Prevent moving down past minimum height in degrees
            } else {
                dr4b.setVelocity(liftSpeed, percent);
                dr4b.spin(reverse);
                // Same logic as ButtonR1, except that it is for the reversed movement.
            }
        } 

        else {
            // Hold position actively against gravity even if the controller is not being physically touched at the moment
            dr4b.stop();
        }

        // Brief delay to give time to the brain scheduler
        wait(20, msec); // basically used to not overload the brain with too many commands at one given moment in time
    }
}