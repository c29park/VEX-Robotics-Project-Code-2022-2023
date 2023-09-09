#include "vex.h"

//spinning function for usercontrol
void Drive(int left, int right){
  fl.spin(forward, left, pct);
  bl.spin(forward, left, pct);
  fr.spin(forward, right, pct);
  br.spin(forward, right, pct);
}

//Driver Control
int driveCap = 1;
bool toggle = false;
bool shot = false;

//4 motor mecanum drive control code
void DriverControls(){
  int X2 =0, Y1 =0, X1 = 0, threshold = 1;//threshold is to control the deadband on the axes of the controller
  int LY = Controller1.Axis3.value();
  int RX = Controller1.Axis4.value(); 
  int PX = Controller1.Axis1.value();
  if(abs(LY) > threshold)
    Y1 = LY;
  else 
    Y1 =0;
  if(abs(RX) > threshold)
    X1 = RX;
  else 
    X1 =0;
  if(abs(PX) > threshold) 
    X2 = PX;
  else 
    X2 = 0;

  fr.spin(fwd, (Y1 - X2 - X1), pct);
  fl.spin(fwd, (Y1 + X2 + X1), pct);
  br.spin(fwd, (Y1 - X2 + X1), pct);
  bl.spin(fwd, (Y1 + X2 - X1), pct);
  //calculates where to go and controls the spin by inputs on the controller axes
}




void Flywheel(){//toggle switch for activating the flywheel
  if(Controller2.ButtonX.pressing() && !toggle){
    toggle = true; 
    wait(100, msec);
  }
  else if(Controller2.ButtonX.pressing() && toggle){
    toggle = false;
    wait(100, msec);
  }
  if(toggle){
    flywheel.spin(fwd, 10.8, volt);
  }else{
    flywheel.stop();
  }
}

void Roller(){// Roller spinning function
  if(Controller2.ButtonR1.pressing()){
    roller.spin(fwd);
  }
  else if(Controller2.ButtonR2.pressing()){
    roller.spin(reverse);
  }
  else if(Controller2.ButtonRight.pressing() && !(16000000 <= iSeeColour.color() && iSeeColour.color() <= 17000000)){//based on optical sensor hue values
      roller.spin(reverse);
    }
  else{
    roller.stop();
  }
}
//disk collector function
void Intakes(){
  if(Controller2.ButtonL1.pressing()){
    intake.spin(fwd);
  }
  else if(Controller2.ButtonL2.pressing()){
    intake.spin(reverse);
  }
  else{
    intake.stop();
  }
}
//shooting disk function that uses the disk loader
void Shoot(){
  if(Controller2.ButtonUp.pressing()){
    push.spin(reverse);
    wait(150, msec);
    shot = true;
  }
  else if(Controller2.ButtonDown.pressing()){
    push.spin(fwd);
  }
  else{//resets the loader's position to default
    push.spinToPosition(0, deg, true);
    push.stop();
    if(shot){
      flywheel.stop();
      shot = false;
      toggle = false;
    }
  }
}

//expansion system function
void expansion(){
  if(Controller2.ButtonB.pressing()){
    //when button pressed activates the expansion pneumatics
    expa1.set(true);
    expa2.set(true);
  }
}

//vibration function that tells when the flywheel's velocity is good enough for the disks to travel high in a trajectory
void vib(){
  if(flywheel.velocity(rpm) >= 430 && !Controller2.ButtonUp.pressing()){
    Controller1.rumble("-");
    Controller2.rumble("-");
  }

}

