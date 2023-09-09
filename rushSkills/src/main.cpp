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

//preprocessed code 
void pre_auton(){
  //turns off the expansion systems
  expa1.set(false);
  expa2.set(false);
  //sets the velocities of the motors based on percent
  flywheel.setVelocity(500, pct);
  intake.setVelocity(250, pct);
  push.setVelocity(10, pct);
  vexcodeInit();
  //sets the current disk loader's position as default
  push.setPosition(0,deg);
}

//autonomous mode
void autonomous() {
  
  Skills();
}

//manual user control mode
void usercontrol() {
  //deactivates PD thread
  enableDrivePD = false;
  Drive(0, 0);
  //sets stopping modes and velocities for all wheel motors
  fl.setStopping(coast);
  bl.setStopping(coast);
  fr.setStopping(coast);
  br.setStopping(coast);
  fl.setVelocity(70, pct);
  bl.setVelocity(70, pct);
  fr.setVelocity(70, pct);
  br.setVelocity(70, pct);
  push.setStopping(brake);
  //forever loop that runs during the whole usercontrol period
  while(1){
    // call all control methods for systems
    DriverControls();
    wait(10, msec);
    Intakes();
    Flywheel();
    Shoot();
    expansion();
    Roller();
    vib();
  }
}

//main method that activates all the modes 
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while(true){//controller 
    Controller1.Screen.print(flywheel.velocity(rpm));
    Controller1.Screen.newLine();
   
    Controller2.Screen.print(DrivetrainInertial.heading());
    Controller2.Screen.newLine();
    wait(100, msec);
  }
}
