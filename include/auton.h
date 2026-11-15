#pragma once
// Autonomous routines: score 1 pin in each of 2 goals,
// using the inertial sensor for turns and straight drives.
// There is one routine per starting spot (Red/Blue, Left/Right).
void runAuton();

// Brain screen auton picker. Runs until the robot is enabled, so call it
// from comp() after pre_auton(). Tap a routine to select it.
void runAutonSelector();
