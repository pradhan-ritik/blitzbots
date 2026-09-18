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
motor motorR1 = motor(PORT11, ratio6_1, true);
motor motorR2 = motor(PORT12, ratio6_1, true);
motor motorL1 = motor(PORT13, ratio6_1, false);
motor motorL2 = motor(PORT14, ratio6_1, false);

// Motor groups
motor_group motorGroupR = motor_group(motorR1, motorR2);
motor_group motorGroupL = motor_group(motorL1, motorL2);

// Drivetrain setup
drivetrain drive(motorGroupL, motorGroupR, 8.639, 12.75, 8.5, distanceUnits::in, 0.6);

int main() {
    motorGroupL.setStopping(coast);
    motorGroupR.setStopping(coast);

    int deadband = 5;

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
        int leftVelocity  = forwardSpeed + turnSpeed;
        int rightVelocity = forwardSpeed - turnSpeed;

        // Clamp total combined motor velocity strictly between -75% and +75%
        if (leftVelocity > 75)   leftVelocity = 75;
        if (leftVelocity < -75)  leftVelocity = -75;
        if (rightVelocity > 75)  rightVelocity = 75;
        if (rightVelocity < -75) rightVelocity = -75;

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

        // Brief delay to give time to the brain scheduler
        wait(20, msec);
    }
}