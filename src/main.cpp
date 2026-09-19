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
    motorGroupL.setStopping(coast);
    motorGroupR.setStopping(coast);

    int deadband = 5;
    int liftSpeed = 40;
    double BOTTOM_LIMIT_DEG = -1.0;    // Deg threshold near bottom rest
    double TOP_LIMIT_DEG = 540.0;  // Deg threshold near top extension

    while (true) {
        // --- READ JOYSTICKS & SCALE TO 75% MAX SPEED ---
        int forwardSpeed = static_cast<int>(Controller1.Axis3.position() * 0.75); 
        int turnSpeed    = static_cast<int>(Controller1.Axis1.position() * 0.75); 

        // Telemetry display on Controller Screen
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("Fwd Vel:  %d%%   ", forwardSpeed);

        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("Turn Vel: %d%%   ", turnSpeed);

        // Split-Arcade calculation
        int leftVelocity  = forwardSpeed - turnSpeed;
        int rightVelocity = forwardSpeed + turnSpeed;

        // Clamp total combined motor velocity strictly between -75% and +75%
        if (leftVelocity < -75)   leftVelocity = -75;
        if (leftVelocity > 75)  leftVelocity = 75;
        if (rightVelocity < -75)  rightVelocity = -75;
        if (rightVelocity > 75) rightVelocity = 75;

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

        
        // dr4b
        double currentPosition = dr4b.position(rotationUnits::deg);

        if (Controller1.ButtonR1.pressing()) {
            // Prevent moving UP past maximum height in degrees
            if (currentPosition >= TOP_LIMIT_DEG) {
                dr4b.stop();
            } else {
                dr4b.setVelocity(liftSpeed, percent);
                dr4b.spin(forward);
            }
        } 

        else if (Controller1.ButtonR2.pressing()) {
            // Prevent moving DOWN past minimum height in degrees
            if (currentPosition <= BOTTOM_LIMIT_DEG) {
                dr4b.stop();
            } else {
                dr4b.setVelocity(liftSpeed, percent);
                dr4b.spin(reverse);
            }
        } 

        else {
            // Hold position actively against gravity
            dr4b.stop();
        }

        // Brief delay to give time to the brain scheduler
        wait(20, msec);
    }
}