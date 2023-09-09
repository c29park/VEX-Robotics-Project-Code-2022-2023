#include "vex.h"
double facing;//facing of the robot

void spinRoller(int b, double a){//spins roller based on the parameters: b is whether the roller has to spin reverse and a is the time taken to spin
  if(b==-1)
    intake.spinFor(reverse, a, sec);
  if(b==0)
    intake.spinFor(fwd, a, sec);
}

void strafeR(double rot){//strafes to the right for the "rot" amount of rotations in degrees
  bl.spinFor(reverse, rot, deg, false);
  br.spinFor(fwd, rot, deg, false);
  fr.spinFor(reverse, rot, deg, false);
  fl.spinFor(fwd, rot, deg, true);
}
void strafeL(double rot){//strafes to the left for the "rot" amount of rotations in degrees
  bl.spinFor(fwd, rot, deg, false);
  br.spinFor(reverse, rot, deg, false);
  fr.spinFor(fwd, rot, deg, false);
  fl.spinFor(reverse, rot, deg, true);
}

//these are for skills auto

void turnRight(double a, double b){//turn right function that turns to the heading based on the parameters: a is the turning speed and b is the degrees that the robot is trying to turn 
  facing=DrivetrainInertial.heading();
  Drivetrain.setTurnVelocity(a, pct);
  if((DrivetrainInertial.heading()+b) >= 360){
    Drivetrain.turnToHeading(facing+b-360, degrees, true);
  }else if((DrivetrainInertial.heading()+b) < 360){
    Drivetrain.turnToHeading(facing+b, degrees, true);
  }
}
void turnLeft(double a, double b){//turn left function that turns to the heading based on the parameters: a is the turning speed and b is the degrees that the robot is trying to turn
  facing=DrivetrainInertial.heading();
Drivetrain.setTurnVelocity(a, pct);
if((DrivetrainInertial.heading()-b) <= 0){
    Drivetrain.turnToHeading(facing-b+360, degrees, true);
  }else if((DrivetrainInertial.heading()+b) > 0){
    Drivetrain.turnToHeading(facing+b, degrees, true);
  }
}
void faceHeading(double a, double b){//turning to specific heading to b degrees and turns in a pct velocity
  Drivetrain.setTurnVelocity(a, pct);
  Drivetrain.turnToHeading(b, degrees, true);
}
void eat(double a){//collects disks
  intake.setVelocity(100,pct);
  intake.spinFor(a, degrees, false);
}

void Skills(){//Skills auto
  vex::task PP(PDdrive);
  PP.suspend();  
  DrivetrainInertial.setHeading(0, deg);

  //First Roller
  roller.spinFor(140, deg, true);
  faceHeading(10, 0);
  //Turn towards rings
  turnRight(50, 138.0);
  wait(100,msec);
  eat(7750);
  //Second roller
  faceHeading(30, 91);
  PP.resume();
  PD(350,0);
  roller.spinFor(380, deg, true);
  PD(-185, 800);
  PP.suspend();
  //Aim for Shooting
  flywheel.spin(fwd, 10.8, volt);
  faceHeading(50, 354);
  wait(100, msec);
  PP.resume();
  wait(200, msec);
  PD(-1250, 2200);
  PP.suspend();
  wait(500,msec);
  //Shoot
  push.spinFor(reverse, 50, deg, true);
  wait(500,msec);
  flywheel.stop();
  push.spinFor(forward, 50, deg, false);
  wait(200, msec);
  //Align then intake next 3 rings
  faceHeading(30, 324);
  eat(12000);
  PP.resume();
  PD(900, 1450);
  PP.suspend();
  //Aim for Shooting
  faceHeading(30, 232);
  wait(400, msec);
  flywheel.spin(fwd, 11.0, volt);
  PP.resume();
  PD(850, 1400);
  PP.suspend();
  wait(100,msec);
  faceHeading(30,305);
  wait(500,msec);
  //Shoot
  push.spinFor(reverse, 50, deg, true);
  wait(500,msec);
  flywheel.stop();
  push.spinFor(fwd, 50, deg, false);
  
  wait(100, msec);
  //Head towards opposite corner (All past here is provisional (Not 100% working))
  faceHeading(30, 250);
  PP.resume();
  PD(1100, 1800);
  PP.suspend();
  wait(100, msec);
  faceHeading(30, 225);
  wait(100,msec);
  PP.resume();
  PD(670, 1100);
  PP.suspend();
  //Third roller
  faceHeading(30,180);
  Drivetrain.setTimeout(3, sec);
  roller.spinFor(300, deg, true);
  faceHeading(10, 180);
  turnRight(50, 138.0);
  wait(100,msec);
  eat(7500);
  //Fourth roller
  faceHeading(30, 271);
  PP.resume();
  PD(300, 0);
  roller.spinFor(150, deg, true);
  PD(-180, 800);
  PP.suspend();
  //Prepare for Expansion
  faceHeading(30, 225);
  strafeL(500);
  //Expand
  expa1.set(true);
  expa2.set(true);
  wait(300, msec);
  expa1.set(false);
  expa2.set(false);
  wait(300, msec);
  //Try to expand again
  expa1.set(true);
  expa2.set(true);
}

void r() { //right side auto 
    Drivetrain.setDriveVelocity(35, pct);
    Drivetrain.driveFor(reverse, 6, inches, true);//back up 
    wait(200, msec);
    strafeR(900);//strafe right
    wait(200, msec);
    Drivetrain.setTimeout(2, sec);
    Drivetrain.driveFor(4.5, inches, true);
    wait(1000, msec);//wait and roller 
    Drivetrain.driveFor(1.0, inches, false);
    Drivetrain.setTimeout(2, sec);
    intake.setVelocity(250, pct);
    intake.spinFor(reverse, 500, deg, true);//roller 
    Drivetrain.driveFor(reverse, 5.0, inches, true);
    wait(200, msec);
    Drivetrain.setTimeout(2.0, sec);
    Drivetrain.setTurnVelocity(50.0, pct);
    turnRight(30,88);
    Drivetrain.setTimeout(5.0, sec);
    Drivetrain.driveFor(reverse, 40, inches, true);
    flywheel.spinFor(forward, 2000, deg, false);
    wait(1500, msec);
    push.spinFor(reverse, 50, deg, true);
}

void l(){//left auto 
  Drivetrain.setDriveVelocity(35, pct);
  Drivetrain.driveFor(reverse, 6, inches, true);//back up 
  wait(200, msec);
  strafeR(-900);//strafe right
  wait(200, msec);
  Drivetrain.driveFor(5.5, inches, true);
  wait(1000, msec);//wait and roller 
  Drivetrain.driveFor(1.0, inches, false);
  intake.setVelocity(250, pct);
  intake.spinFor(reverse, 500, deg, true);//roller 
  Drivetrain.driveFor(reverse, 5.0, inches,true);
  wait(200,msec);
  Drivetrain.setTimeout(2.0,sec);
  Drivetrain.setTurnVelocity(50.0,pct);
  turnLeft(30, 88);
  Drivetrain.setTimeout(5.0,sec);
  Drivetrain.driveFor(reverse,40,inches,true);
  flywheel.spinFor(forward, 2000, deg, false);
  wait(1500, msec);
  push.spinFor(reverse, 50, deg, true);
}

//PD settings for drivetrain 
double kp = 0.0093;
double kd = 0.0063;

int desiredValue = 0;

int error;
int prevError = 0;
int derivative;

bool enableDrivePD = true;
bool resetDriveSensors = false;
double MotorPower=0;

//driving straight function
void PD(double a, double t){
  resetDriveSensors = true;//resets the motor positions
  desiredValue = a;// parameter a is the motor spinning value in degrees 
  wait(t, msec);// parameter t is the completion time for the driving in msec.
  Drivetrain.stop(brake); 
}

//overloaded function of PID
void PD(double a, double p, double d, double t){
  resetDriveSensors = true;//resets the motor positions
  kp = p;// p = parameter for kp(directly responsible for speed)
  kd = d;// d = parameter for kd 
  desiredValue = a;
  wait(t, msec);
}

int PDdrive() {
  while(enableDrivePD){
    //reseting the sensor values
    if(resetDriveSensors){
      resetDriveSensors = false;//switch off 
      fl.setPosition(0, deg);
      bl.setPosition(0,deg);
      fr.setPosition(0,deg);
      br.setPosition(0, deg);
      ltrack.setPosition(0, deg);
      rtrack.setPosition(0, deg);
    }
    //getting the position of the motors
    int lPos = ltrack.position(deg);
    int rPos = rtrack.position(deg);
    int flPos = fl.position(deg);
    int blPos = bl.position(deg);
    int frPos = fr.position(deg);
    int brPos = br.position(deg);

    //PD
    //Get the average
    int avgPos = (lPos + rPos + flPos + blPos + frPos + brPos)/6;
    //Proportional 
    error = avgPos - desiredValue; //displacement or pos
    //Derivative
    derivative = error - prevError;//takes the rate of change of the displacement, speed
    //"control of the motors" statement
    MotorPower = error*kp + derivative*kd;
    
    //////////////////////////////////////////////////////////////////////Activate Motors
    fl.spin(reverse, MotorPower, voltageUnits::volt);
    bl.spin(reverse, MotorPower, voltageUnits::volt);
    fr.spin(reverse, MotorPower, voltageUnits::volt);
    br.spin(reverse, MotorPower, voltageUnits::volt);

    prevError = error; //updates the previous Error
    task::sleep(20); 
  }
  return 1;
}
