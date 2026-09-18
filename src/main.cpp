/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ritik                                                     */
/*    Created:      9/12/2026, 11:22:44 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here
motor motorR1 = motor(PORT11, ratio6_1, false);
motor motorR2 = motor(PORT12, ratio6_1, false);
motor motorL1 = motor(PORT13, ratio6_1, true);
motor motorL2 = motor(PORT14, ratio6_1, true);
motor_group motorGroupR = motor_group(motorR1, motorR2);
motor_group motorGroupL = motor_group(motorL1, motorL2);
// Left Group, Right Group, Wheel Travel (in), Track Width (in), Wheel Base (in) units (in) gear ratio (36/60)
drivetrain drive(motorGroupL, motorGroupR, 8.639, 12.75, 8.5, distanceUnits::in, 0.6);


int main() {

    Brain.Screen.printAt( 10, 50, "Hello V5" );
    drive.setDriveVelocity(100, percent);
   
    while(1) {
        drive.drive(forward);
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
