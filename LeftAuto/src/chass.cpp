#include "vex.h"

int yMax = 100;
int hMax = 100;

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

void DriverControls(float mult){
  // int LY = Controller1.Axis3.value();
  // int RX = Controller1.Axis1.value();
  // int PX = Controller.1
  // if (RX < 15 && RX > -15 && RX != 0) {
  //  RX = (abs(RX) / RX)* 10;
  // }

  // int lSpeed = LY + RX*0.85;
  // int rSpeed = LY - RX*0.85;

  // Drive(lSpeed * mult, rSpeed * mult);
  int X2 =0, Y1 =0, X1 = 0, threshold = 15;
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

  fr.spin(fwd, (Y1 - X2 - X1)*mult, pct);
  fl.spin(fwd, (Y1 + X2 + X1)*mult, pct);
  br.spin(fwd, (Y1 - X2 + X1)*mult, pct);
  bl.spin(fwd, (Y1 + X2 - X1)*mult, pct);
}




void Flywheel(){
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

void Roller(){
  if(Controller2.ButtonR1.pressing()){
    roller.spin(fwd);
  }
  else if(Controller2.ButtonR2.pressing()){
    roller.spin(reverse);
  }
  else if(Controller2.ButtonRight.pressing() && !(16000000 <= iSeeColour.color() && iSeeColour.color() <= 17000000)){
      roller.spin(reverse);
    }
  else{
    roller.stop();
  }
}

void Intakes(){
  if(Controller2.ButtonL1.pressing()){
    intake.spin(fwd);
    //roller.spin(fwd);
  }
  else if(Controller2.ButtonL2.pressing()){
    intake.spin(reverse);
   // roller.spin(reverse);
  }
  // else if(Controller1.ButtonR1.pressing()){
  //   roller.spin(fwd);
  // }
  // else if(Controller1.ButtonR2.pressing()){
   
  // }
  else{
    intake.stop();
    //roller.stop();
  }
}

void Shoot(){
  if(Controller2.ButtonUp.pressing()){
    push.spin(reverse);
    wait(150, msec);
    shot = true;
  }
  else if(Controller2.ButtonDown.pressing()){
    push.spin(fwd);
  }
  else{
    push.spinToPosition(0, deg, true);
    push.stop();
    if(shot){
      flywheel.stop();
      shot = false;
      toggle = false;
    }
  }
}

void expansion(){
  if(Controller2.ButtonB.pressing()){
    expa1.set(true);
    expa2.set(true);
  }
}

void vib(){
  if(flywheel.velocity(rpm) >= 430 && !Controller2.ButtonUp.pressing()){
    Controller1.rumble("-");
    Controller2.rumble("-");
  }

}


 
double Kp = 0.07;
double Ki = 0.0;
double Kd = 0.11;

int desiredSpeed = 0;

int errorSpeed;
int prevErrorSpeed = 0;
int totalError;
int Derivative;
double prevPos=0;
double Power =0;
bool enableFlywheelPID = true;
bool resetFSensors = false;
int flywheelPID() {
  while(enableFlywheelPID){
    //reseting the sensor values mhm before we get down to business
    // Controller1.Screen.print("hi");
    if(resetFSensors){
      resetFSensors = false;//switch off 
      flywheel.resetPosition();
    }
    //getting the position of the motors
    double vel = flywheel.velocity(rpm);
    
    //////////////////////////////////////////////////Lateral Movement PID...
    //Get the average
    //now comes the fun part
    //Proportional 
    errorSpeed = desiredSpeed - vel; //displacement or pos
    //Derivative
    Derivative = errorSpeed - prevErrorSpeed;//takes the rate of change of the displacement, speed
    
    //totalError += errorSpeed;
    //Integral 
    // if(abs(error) < integralBound){ //if the distance is less than the bound
    //   totalError += error;
    // }else{
    //   totalError =0; //reset it to 0 
    // }

    // //let's cap the integral 
    // totalError = abs(totalError) > maxIntegral ? sign(totalError) * maxIntegral : totalError;
    //"control of the motors" statement
    Power += errorSpeed*Kp + Derivative *Kd;
  
    //////////////////////////////////////////////////////////////////////SPIN THEM ALL
    flywheel.spin(fwd, Power, rpm);
    prevErrorSpeed = errorSpeed; //updates the prevError
    task::sleep(20); 
  }
  return 1;
}

void PID(double a, double t){//driving straight function
// don't use this function to drive for like very short distances(e.g. 6 inches or even 11)
  resetFSensors = true;//resets the motor positions
  desiredSpeed = a;// parameter a is the motor spinning value in degrees 
  //last time 1790 was the value for the robot to go from the very end to the mid point of the field
  // to use the value above about 1600 tho, use the overloaded function below and adjust the d value 
  //(add like idk 0.003 to the value now and see if it drives straight enough)
  
  //wait(t, msec);// parameter t is the completion time for the driving in msec.
  //flywheel.stop();
}//



