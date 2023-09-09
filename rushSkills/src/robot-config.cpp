#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
//all drivetrain motors
motor fl = motor(PORT2, ratio18_1, true);
motor bl = motor(PORT7, ratio18_1, false);
motor fr = motor(PORT9, ratio18_1, false);
motor br = motor(PORT10, ratio18_1, true);
inertial DrivetrainInertial = inertial(PORT20);
motor_group leftDrive = motor_group(fl, bl);
motor_group rightDrive = motor_group(fr, br);
//smart drivetrain configured with motor groups and inertial sensor
smartdrive Drivetrain = smartdrive(leftDrive, rightDrive, DrivetrainInertial, 319.19, 320, 40, mm, 1);
//Controllers configured
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
//all necessary system motors configured
motor flywheel = motor(PORT18, ratio6_1, false);
motor intake = motor(PORT17, ratio18_1, true);
motor jankPush = motor(PORT19, ratio18_1, false);
motor roller = motor(PORT16, ratio36_1, false);
//expansion system configured
digital_out expa1 = digital_out(Brain.ThreeWirePort.A);
digital_out expa2 = digital_out(Brain.ThreeWirePort.B);
//optical sensor configured
optical iSeeColour = optical(PORT1);
//encoders for tracking wheels configured
encoder rtrack = encoder(Brain.ThreeWirePort.G);
encoder ltrack = encoder(Brain.ThreeWirePort.C);

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
   Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  DrivetrainInertial.calibrate();
  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}