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
motor motorL1 = motor(PORT8, ratio6_1, false); 
motor motorL2 = motor(PORT6, ratio6_1, false); 
motor dr4bR = motor(PORT16, ratio36_1, false); 
motor dr4bL = motor(PORT17, ratio36_1, true); 
motor intake = motor(PORT19, ratio6_1, false); 
motor outtake = motor(PORT2, ratio18_1, true); 
motor outtake_move = motor(PORT10, ratio18_1, false); 
inertial imu = inertial(PORT3); 

// Sensors. PORTS ARE PLACEHOLDERS until the build team mounts them.
// Code checks installed() and skips any sensor that isn't plugged in.
distance pinSensor = distance(PORT11);   // points at the pin held in the outtake
distance frontSensor = distance(PORT12); // points forward (outtake side) at goals

// AI Vision colors: PLACEHOLDERS (id, red, green, blue, hue range, saturation range).
// Replace with the values from the AI Vision Utility once the sensor is mounted.
aivision::colordesc RED_PIN(1, 200, 30, 40, 10.0, 0.2);
aivision::colordesc BLUE_PIN(2, 30, 70, 200, 10.0, 0.2);
aivision::colordesc YELLOW_PIN(3, 220, 200, 40, 10.0, 0.2);
// ALL_TAGS: also detect AprilTags (used to line up on goals in auton)
aivision aiVision(PORT13, RED_PIN, BLUE_PIN, YELLOW_PIN, aivision::ALL_TAGS);
// Motor groups
motor_group motorGroupR = motor_group(motorR1, motorR2);
motor_group motorGroupL = motor_group(motorL1, motorL2);
motor_group dr4b = motor_group(dr4bR, dr4bL);

// Drivetrain setup
drivetrain drive(motorGroupL, motorGroupR, 8.639, 12.75, 8.5, distanceUnits::in, 0.6);
bool preAutonRunning = false; // true while comp() is still running pre_auton

// ===================== DR4B DRIVER TUNABLES (motor degrees, same as line 3 "Lift") =====================
const double LIFT_TOP_DEG = 720.0;       // stop going up past this. Keep in sync with TOP_DEG in auton.cpp
const double LIFT_BOTTOM_DEG = -1.0;     // stop going down past this. Keep in sync with BOTTOM_DEG in auton.cpp
const double LIFT_REST_ZONE_DEG = 30.0;  // this close to the bottom with no button: unpower and rest on the hardstop
const double LIFT_SLOW_ZONE_DEG = 140.0; // within this many degrees of the bottom, lower slowly
const int LIFT_UP_SPEED = 100;           // percent. Lifting fights gravity
const int LIFT_DOWN_SPEED = 60;          // percent
const int LIFT_DOWN_SLOW_SPEED = 30;     // percent. Near the bottom so it doesn't slam
const double OUTTAKE_MOVE_PRESET_DEG = 100.0; // up arrow sends outtake_move here (same numbers as "Aim" on controller line 3)
const int LIFT_MAX_TORQUE = 100;         // percent. Mentor suggested 55-60 to protect motors once the lift is strong enough

// NOTE: driveLiftSide() is not called right now. Driver control uses the simple
// dr4b.spin() version below (no soft limits). Keep this for when limits are added back.
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

void pre_auton() {
  // Reset motor positions and set stopping modes
  dr4b.resetPosition();
  motorGroupL.setStopping(coast);
  motorGroupR.setStopping(coast);
  outtake_move.setStopping(hold);
  aiVision.tagDetection(true); // AprilTags are off by default

  // Robot must stay still for about 2 seconds while this runs
  imu.calibrate();
  while (imu.isCalibrating()) {
    wait(20, msec);
  }
}
// Cubic blend: small stick movements stay gentle, full stick still gives full speed.
// k = 0 is linear, k = 1 is fully cubic. Raise it for a softer center.
double curve(double input, double k) {
    double x = input / 100.0;
    return 100.0 * (k * x * x * x + (1.0 - k) * x);
}

void driver_control() {
    // With no field connected this starts right away, so let pre_auton finish first
    while (preAutonRunning) {
        wait(20, msec);
    }

    motorGroupL.setStopping(coast); // not a sudden stop, but moves forward with the momentum it has before coming to a rest.
    motorGroupR.setStopping(coast); // not a sudden stop, but moves forward with the momentum it has before coming to a rest.
    dr4b.setStopping(hold); // if not being adjusted by the controller, the dr4b will not fall due to gravity, rather hold.
    dr4b.setVelocity(50, percent); // lift speed for R1/R2 below (was a stray line at file level, which doesn't compile)
    dr4b.setMaxTorque(LIFT_MAX_TORQUE, percent);
    // Erase the auton selector buttons (keeps the auton name/time lines at the top)
    Brain.Screen.setPenColor(black);
    Brain.Screen.setFillColor(black);
    Brain.Screen.drawRectangle(0, 45, 480, 195);
    Brain.Screen.setPenColor(white);

    int deadband = 5;
    bool outtakeForward = false; // false = reverse (hold pin), true = forward (score)
    bool aWasPressed = false;    // for detecting a new A press, not a held one
    bool outtakeStopped = false; // set by the down arrow, cleared by A
    bool outtakeMovePreset = false; // true while outtake_move is going to / holding the up-arrow angle
    while (true) {
        // --- READ JOYSTICKS & SCALE TO 75% MAX SPEED ---
        int forwardSpeed = static_cast<int>(curve(Controller1.Axis3.position(), 0.6) * 0.75); // y Axis (max 100, min -100) -> 0.75 to maintain control over the robot and not have a volatile shift over a small touch.
        int turnSpeed    = static_cast<int>(curve(Controller1.Axis1.position(), 0.7) * 0.5); // x axis

        // Telemetry display on Controller Screen
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("Fwd:%d%% Turn:%d%%     ", forwardSpeed, turnSpeed); // drive speeds for testing

        // Line 2: AprilTag nearest the center of the camera.
        // Use it to find each goal's tag ID and TAG_TARGET_X_GOAL1/2 (auton.cpp)
        Controller1.Screen.setCursor(2, 1);
        int tagCount = aiVision.installed() ? aiVision.takeSnapshot(aivision::ALL_TAGS) : 0;
        int tagId = -1, tagX = -1;
        for (int i = 0; i < tagCount; i++) {
            int x = aiVision.objects[i].centerX;
            if (tagId < 0 || abs(x - 160) < abs(tagX - 160)) {
                tagId = aiVision.objects[i].id;
                tagX = x;
            }
        }
        if (tagId >= 0) Controller1.Screen.print("Tag id:%d x:%d     ", tagId, tagX);
        else Controller1.Screen.print("Tag: none          ");

        Controller1.Screen.setCursor(3, 1);
        Controller1.Screen.print("Aim:%.0f Lift:%.0f   ", outtake_move.position(rotationUnits::deg), dr4b.position(rotationUnits::deg)); // line 3 is for finding the auton aim and lift angles

        // Split-Arcade calculation
        int leftVelocity  = forwardSpeed - turnSpeed;
        int rightVelocity = forwardSpeed + turnSpeed;
        // This is known as Split-Arcade, rather than Tank. 
        // They can both used but are for different driver preferences. 
        // The Tank uses one axis to move right and one to move left. 
        // However, the split-arcade uses one axis for the direction and the other for the s\peed.
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
        if (Controller1.ButtonR1.pressing()) {
            dr4b.spin(forward);   // lift goes up
        } else if (Controller1.ButtonR2.pressing()) {
            dr4b.spin(reverse);   // lift goes down
        } else {
            dr4b.stop();          // holds position
        }

        // Brain screen: lift diagnostics. ~2.5 A while stuck = motors maxed out (mechanical problem).
        Brain.Screen.printAt(10, 100, "R  pos %5.0f  amps %4.2f  temp %3.0fC   ", dr4bR.position(rotationUnits::deg), dr4bR.current(currentUnits::amp), dr4bR.temperature(temperatureUnits::celsius));
        Brain.Screen.printAt(10, 130, "L  pos %5.0f  amps %4.2f  temp %3.0fC   ", dr4bL.position(rotationUnits::deg), dr4bL.current(currentUnits::amp), dr4bL.temperature(temperatureUnits::celsius));

        // Sensor readouts for tuning PIN_DETECT_MM, GOAL_SCORE_IN and the vision colors (auton.cpp / above)
        if (pinSensor.installed()) Brain.Screen.printAt(10, 160, "Pin sensor: %5.0f mm   ", pinSensor.objectDistance(mm));
        if (frontSensor.installed()) Brain.Screen.printAt(10, 190, "Front: %5.1f in   ", frontSensor.objectDistance(inches));
        if (aiVision.installed()) {
            int reds = aiVision.takeSnapshot(RED_PIN);
            int blues = aiVision.takeSnapshot(BLUE_PIN);
            int yellows = aiVision.takeSnapshot(YELLOW_PIN);
            Brain.Screen.printAt(10, 220, "Vision sees R:%d B:%d Y:%d   ", reds, blues, yellows);
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

        // outtake: always spinning. Press A to flip direction (and restart it if stopped).
        // Starts in reverse (keeps a pin in); forward scores it. Down arrow stops it.
        bool aPressed = Controller1.ButtonA.pressing();
        if (aPressed && !aWasPressed) {
            outtakeForward = !outtakeForward;
            outtakeStopped = false;
        }
        aWasPressed = aPressed;
        if (Controller1.ButtonDown.pressing()) {
            outtakeStopped = true;
        }
        if (outtakeStopped) {
            outtake.stop();
        } else {
            outtake.setVelocity(100, percent);
            outtake.spin(outtakeForward ? forward : reverse);
        }

        // outtake_move: Y/B move it by hand. Up arrow sends it to OUTTAKE_MOVE_PRESET_DEG
        // and it keeps going there until it arrives or Y/B is pressed.
        if (Controller1.ButtonY.pressing()) {
            outtakeMovePreset = false;
            outtake_move.setVelocity(100, percent);
            outtake_move.spin(forward);
        }

        else if (Controller1.ButtonB.pressing()) {
            outtakeMovePreset = false;
            outtake_move.setVelocity(100, percent);
            outtake_move.spin(reverse);
        }

        else if (Controller1.ButtonUp.pressing()) {
            if (!outtakeMovePreset) {
                outtakeMovePreset = true;
                outtake_move.setVelocity(100, percent);
                outtake_move.spinToPosition(OUTTAKE_MOVE_PRESET_DEG, rotationUnits::deg, false);
            }
        }

        else if (!outtakeMovePreset) {
            outtake_move.stop(); // holds position (stopping mode is hold)
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

    // Pick the auton on the brain screen. Returns once the robot is enabled.
    runAutonSelector();

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