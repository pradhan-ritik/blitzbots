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
motor motor1 = motor(PORT11, ratio18_1, false);
motor motor2 = motor(PORT12, ratio18_1, false);

int main() {

    Brain.Screen.printAt( 10, 50, "Hello V5" );
   
    while(1) {
        motor1.spin(forward);
        motor2.spin(forward);
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
