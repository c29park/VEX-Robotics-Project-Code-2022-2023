/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\user                                             */
/*    Created:      Thu Nov 03 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// fl                   motor         10              
// bl                   motor         7               
// fr                   motor         1               
// br                   motor         2               
// Controller1          controller                    
// roller               motor         16              
// expa                 motor         18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

competition Competition;

void pre_auton(){
  vexcodeInit();
  expa1.set(false);
  expa2.set(false);
  flywheel.setVelocity(500, pct);
  intake.setVelocity(250, pct);
  push.setVelocity(10, pct);
  push.setPosition(0,deg);
  Brain.resetTimer();
  ltrack.resetRotation();
  rtrack.resetRotation();
  btrack.resetRotation();
  //previousL = previousR = previousB = 0;
}



void autonomous() {
  l();
  


}

void usercontrol() {
  enableDrivePD = false;
  Brain.resetTimer();
  ltrack.resetRotation();
  rtrack.resetRotation();
  btrack.resetRotation();
  X = 31.375;
  Y = 12.25;
  theta = M_PI;
  //previousL = previousR = previousB = 0;
  Drive(0, 0);
  fl.setStopping(coast);
  bl.setStopping(coast);
  fr.setStopping(coast);
  br.setStopping(coast);
  push.setStopping(brake);
  while(1){
    //DriverControlsWithSwap();
    DriverControls(1);
    wait(10, msec);
    Intakes();
    Flywheel();
    Shoot();
    expansion();
    Roller();
    vib();
    // Controller1.Screen.print(flywheel.velocity(rpm));
    // Controller1.Screen.newLine();
    // Controller2.Screen.print(flywheel.velocity(rpm));
    // Controller2.Screen.newLine();
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while(true){
    // Controller1.Screen.print(flywheel.velocity(rpm));
    // Controller1.Screen.newLine();
    // Controller2.Screen.print(flywheel.velocity(rpm));
    // Controller2.Screen.newLine();
    // Controller2.Screen.print(DrivetrainInertial.heading());
    // Controller2.Screen.newLine();
  
    wait(100, msec);
  }
}
