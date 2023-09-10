#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
motor fl = motor(PORT2, ratio18_1, true);
motor bl = motor(PORT7, ratio18_1, false);
motor fr = motor(PORT9, ratio18_1, false);
motor br = motor(PORT10, ratio18_1, true);
inertial DrivetrainInertial = inertial(PORT20);
motor_group leftDrive = motor_group(fl, bl);
motor_group rightDrive = motor_group(fr, br);
smartdrive Drivetrain = smartdrive(leftDrive, rightDrive, DrivetrainInertial, 319.19, 320, 40, mm, 1);
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
motor flywheel = motor(PORT18, ratio6_1, false);
motor intake = motor(PORT17, ratio18_1, true);
motor push = motor(PORT19, ratio18_1, false);
motor roller = motor(PORT16, ratio36_1, false);
digital_out expa1 = digital_out(Brain.ThreeWirePort.A);
digital_out expa2 = digital_out(Brain.ThreeWirePort.B);
optical iSeeColour = optical(PORT1);
encoder rtrack = encoder(Brain.ThreeWirePort.G);//H
encoder ltrack = encoder(Brain.ThreeWirePort.C);//D
encoder btrack = encoder(Brain.ThreeWirePort.E);//F
// VEXcode gg
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