#include "vex.h"
double facing;
bool odomen = true;
bool PDdriveOn = false;
int sign(float x) {
  if (x > 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}

void spinRoller(int b, double a){
  if(b==-1)
    intake.spinFor(reverse, a, sec);
  if(b==0)
    intake.spinFor(fwd, a, sec);
}
void spinExpa(){
 // expa.setVelocity(90, pct);
 // expa.spinFor(fwd, 10, sec);
}

void strafeR(double rot){
  bl.spinFor(reverse, rot, deg, false);
  br.spinFor(fwd, rot, deg, false);
  fr.spinFor(reverse, rot, deg, false);
  fl.spinFor(fwd, rot, deg, true);
}
void strafeL(double rot){
  bl.spinFor(fwd, rot, deg, false);
  br.spinFor(reverse, rot, deg, false);
  fr.spinFor(fwd, rot, deg, false);
  fl.spinFor(reverse, rot, deg, true);
}

//these are for skills auto

void go(double a, double b, bool c){
  Drivetrain.setDriveVelocity(a, pct);
  Drivetrain.driveFor(forward, b, inches, c);
}
void turnRight(double a, double b){
  facing=DrivetrainInertial.heading();
  Drivetrain.setTurnVelocity(a, pct);
  if((DrivetrainInertial.heading()+b) >= 360){
    Drivetrain.turnToHeading(facing+b-360, degrees, true);
  }else if((DrivetrainInertial.heading()+b) < 360){
    Drivetrain.turnToHeading(facing+b, degrees, true);
  }
}
void turnLeft(double a, double b){
  facing=DrivetrainInertial.heading();
Drivetrain.setTurnVelocity(a, pct);
if((DrivetrainInertial.heading()-b) <= 0){
    Drivetrain.turnToHeading(facing-b+360, degrees, true);
  }else if((DrivetrainInertial.heading()+b) > 0){
    Drivetrain.turnToHeading(facing+b, degrees, true);
  }
}
void faceHeading(double a, double b){
  Drivetrain.setTurnVelocity(a, pct);
  Drivetrain.turnToHeading(b, degrees, true);
}
void eat(double a){
  intake.setVelocity(100,pct);
  intake.spinFor(a, degrees, false);
}
void l(){
  
  enableDrivePD = true;
  vex::task PID(PDdrive);
  Drivetrain.setDriveVelocity(35, pct);
  Drivetrain.driveFor(reverse, 6, inches, true);//back up 
  wait(200, msec);
  strafeR(-900);//strafe right
  wait(200, msec);
  Drivetrain.setTimeout(2.0 , sec);
  Drivetrain.driveFor(5.5, inches, true);
  wait(1000, msec);//wait and roller 
  Drivetrain.driveFor(1.0, inches, false);
  roller.spinFor(50, deg, true);//roller 
  Drivetrain.driveFor(reverse, 5.0, inches,true);
  wait(200,msec);
  Drivetrain.setTimeout(2.0,sec);
  Drivetrain.setTurnVelocity(50.0,pct);
  turnLeft(30, 88);
  Drivetrain.setTimeout(5.0,sec);
  Drivetrain.driveFor(reverse,40,inches,true);
  flywheel.spinFor(forward, 2000, deg, false);
  wait(1500, msec);
  //push.spinFor(reverse, 50, deg, true);
}

const double fieldscale = 1.66548042705, SL = 5.125, SR = 6.25, SB = 5.6875, WheelCircum = 8.63937979737;
//SR = distance to the right from the center of the robot
//SL = distance to the left from the center of the robot
//SB = distance to the back from the center of the robot
double deltaL = 0,deltaR = 0,deltaB = 0,currentL = 0,currentR = 0, currentB = 0, previousL = 0,previousR = 0, previousB = 0, deltaTheta = 0,X = 0,Y = 0,theta = 0,robotHeading = 0;
int count = 0;
 // in inches
// funtion: locationTracking - 2 tracking wheel + inertial
// description: used to track where the robot is relative to the field using
// note: it should be used inside a while loop with threads in autonomous
int positionTracking(){
  while(odomen){
   Brain.Screen.clearScreen();
   if (++count % 50 == 0) {
     Controller1.Screen.print(X);
     Controller1.Screen.print(":");
     Controller1.Screen.print(Y);
     Controller1.Screen.print(":");
     Controller1.Screen.print(robotHeading); Controller1.Screen.newLine();
   }
   currentR = -rtrack.position(degrees);
   currentL = ltrack.position(degrees);
   currentB = btrack.position(degrees);//
   deltaL = (currentL - previousL) * WheelCircum / 360;
   deltaR = (currentR - previousR) * WheelCircum / 360;
   deltaB = (currentB - previousB) * WheelCircum / 360;//
   deltaTheta = DrivetrainInertial.rotation() / 57.295779513 - theta;
   if (fabs(deltaTheta) <= 1.5e-5) {
     X += deltaL * sin(theta);
     Y -= deltaR * cos(theta);
   } else {
     double SideChord = 2*((deltaL / deltaTheta) + SL) * sin(deltaTheta/2);
     double DeltaYSide = SideChord *cos(theta+(deltaTheta/2));
     double DeltaXSide = SideChord *sin(theta+(deltaTheta/2));
     theta += deltaTheta;
     X += DeltaXSide;
     Y -= DeltaYSide;
   }
   robotHeading = theta * 57.295779513;
   previousL = currentL;
   previousR = currentR;
   previousB = currentB;
   deltaTheta = 0;


   int textadjustvalue = 55;
   int rowadjust = 39;
  
  
   //Sets graphical things for our display
   Brain.Screen.setPenWidth( 1 );
   vex::color redtile = vex::color( 210, 31, 60 );
   vex::color bluetile = vex::color( 14, 77, 146 );
   vex::color graytile = vex::color( 49, 51, 53 );
   Brain.Screen.setFillColor(vex::color( 0, 0, 0 ));
   Brain.Screen.setFont(vex::fontType::mono20);
   Brain.Screen.setPenColor( vex::color( 222, 49, 99 ) );


   //Displays all the field tiles, text of odom values, and a dot symbolizing the robot
   Brain.Screen.printAt(40,20 + textadjustvalue, "X-Pos:%f",-X);
   Brain.Screen.setPenColor( vex::color( 191, 10, 48 ) );
   Brain.Screen.printAt(40,50 + textadjustvalue, "Y-Pos:%f",Y);
   Brain.Screen.setPenColor( vex::color( 141, 2, 31 ) );
   Brain.Screen.printAt(40,80 + textadjustvalue, "Theta:%f",theta);
   Brain.Screen.setPenColor( vex::color( 83, 2, 1 ) );
   Brain.Screen.printAt(40,110 + textadjustvalue, "Angle:%f",robotHeading);
   Brain.Screen.setPenColor( vex::color( 255, 255, 255 ) );
   Brain.Screen.setFillColor( graytile );
   Brain.Screen.drawRectangle( 245, 2, 234, 234 );
   Brain.Screen.drawRectangle( 245, 80, 39, 39 );
   Brain.Screen.drawRectangle( 245, 119, 39, 39 );
   Brain.Screen.drawRectangle( 245, 158, 39, 39 );
   Brain.Screen.drawRectangle( 245, 197, 39, 39 );
   Brain.Screen.drawRectangle( 245+rowadjust, 80, 39, 39 );
   Brain.Screen.drawRectangle( 245+rowadjust, 119, 39, 39 );
   Brain.Screen.drawRectangle( 245+rowadjust, 158, 39, 39 );
   Brain.Screen.drawRectangle( 245+rowadjust, 197, 39, 39 );
   Brain.Screen.drawRectangle( 245+(2*rowadjust), 2, 39, 39 );
   Brain.Screen.drawRectangle( 245+(2*rowadjust), 41, 39, 39 );
   Brain.Screen.drawRectangle( 245+(2*rowadjust), 80, 39, 39 );
   Brain.Screen.drawRectangle( 245+(2*rowadjust), 119, 39, 39 );
   Brain.Screen.drawRectangle( 245+(2*rowadjust), 158, 39, 39 );
   Brain.Screen.drawRectangle( 245+(2*rowadjust), 197, 39, 39 );
   Brain.Screen.drawRectangle( 245+(3*rowadjust), 2, 39, 39 );
   Brain.Screen.drawRectangle( 245+(3*rowadjust), 41, 39, 39 );
   Brain.Screen.drawRectangle( 245+(3*rowadjust), 80, 39, 39 );
   Brain.Screen.drawRectangle( 245+(3*rowadjust), 119, 39, 39 );
   Brain.Screen.drawRectangle( 245+(3*rowadjust), 158, 39, 39 );
   Brain.Screen.drawRectangle( 245+(3*rowadjust), 197, 39, 39 );
   Brain.Screen.drawRectangle( 245+(4*rowadjust), 2, 39, 39 );
   Brain.Screen.drawRectangle( 245+(4*rowadjust), 41, 39, 39 );
   Brain.Screen.drawRectangle( 245+(4*rowadjust), 80, 39, 39 );
   Brain.Screen.drawRectangle( 245+(4*rowadjust), 119, 39, 39 );
   Brain.Screen.drawRectangle( 245+(5*rowadjust), 2, 39, 39 );
   Brain.Screen.drawRectangle( 245+(5*rowadjust), 41, 39, 39 );
   Brain.Screen.drawRectangle( 245+(5*rowadjust), 80, 39, 39 );
   Brain.Screen.drawRectangle( 245+(5*rowadjust), 119, 39, 39 );
   Brain.Screen.setFillColor( redtile );
   Brain.Screen.drawRectangle( 245+(4*rowadjust), 158, 39, 39 );
   Brain.Screen.drawRectangle( 245+(4*rowadjust), 197, 39, 39 );
   Brain.Screen.drawRectangle( 245+(5*rowadjust), 158, 39, 39 );
   Brain.Screen.drawRectangle( 245+(5*rowadjust), 197, 39, 39 );
   Brain.Screen.setFillColor( bluetile );
   Brain.Screen.drawRectangle( 245, 2, 39, 39 );
   Brain.Screen.drawRectangle( 245, 41, 39, 39 );
   Brain.Screen.drawRectangle( 245+rowadjust, 2, 39, 39 );
   Brain.Screen.drawRectangle( 245+rowadjust, 41, 39, 39 );


   Brain.Screen.setPenColor( vex::color( 255,255,255));
   Brain.Screen.setFillColor( vex::color(0,0,0) );
  
   //This draws the robot body for position and arm for angle
   double yfieldvalue = ((-Y)*fieldscale)+245-10;
   double xfieldvalue = ((-X)*fieldscale)+245;
   Brain.Screen.drawCircle(xfieldvalue, yfieldvalue, 10 );
   Brain.Screen.setPenWidth( 4 );
   //Line angle calculation:
   //x1 and y1 are the robot's coordinates, which in our case is xfieldvalue and yfieldvalue
   //angle is the angle the robot is facing, which in our case is Theta
   //(x1,y1, x1 + line_length*cos(angle),y1 + line_length*sin(angle)) = (x1,y1,x2,y2)
   Brain.Screen.drawLine(xfieldvalue, yfieldvalue, xfieldvalue+cos(-theta-(M_PI/2))*15, yfieldvalue+ sin(-theta-(M_PI/2)) *15);
   Brain.Screen.render();
   wait(20, msec);
 }
 return 1;
}



   

//PD for drivetrain 
double kp = 0.0093;
double kd = 0.0063;

int desiredValue = 0;

int error;
int prevError = 0;  
int derivative;

bool enableDrivePD = true;
bool resetDriveSensors = false;
double motorPower=0;
void PD(double a, double t){//driving straight function
// don't use this function to drive for like very short distances(e.g. 6 inches or even 11)
  resetDriveSensors = true;//resets the motor positions
  desiredValue = a;// parameter a is the motor spinning value in degrees 

  
  wait(t, msec);// parameter t is the completion time for the driving in msec.
  Drivetrain.stop(brake); 
}//overloaded function of PID
void PD(double a, double p, double d, double t){
  resetDriveSensors = true;//resets the motor positions
  kp = p;// p = parameter for kp(directly responsible for speed)
  kd = d;// d = parameter for kd 
  desiredValue = a;
  wait(t, msec);
  //Drivetrain.stop(brake);
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
 
    int flPos = fl.position(deg);
    int blPos = bl.position(deg);
    int frPos = fr.position(deg);
    int brPos = br.position(deg);
    int ltrackPos = ltrack.position(deg);
    int rtrackPos = rtrack.position(deg);
    //Get the average
    int avgPos = (flPos + blPos + frPos + brPos+ ltrackPos + rtrackPos)/6;
    //Proportional 
    error = avgPos - desiredValue; //displacement or pos
    //Derivative
    derivative = error - prevError;//takes the rate of change of the displacement, speed
   
    
    motorPower = error*kp + derivative *kd;
    
    //////////////////////////////////////////////////////////////////////SPIN THEM ALL
    fl.spin(reverse, motorPower, voltageUnits::volt);
    bl.spin(reverse, motorPower, voltageUnits::volt);
    fr.spin(reverse, motorPower, voltageUnits::volt);
    br.spin(reverse, motorPower, voltageUnits::volt);
    
    prevError = error; //updates the prevError
    
   
    task::sleep(20); 
  }
  return 1;
}






void goTo(double a, double b, double c, bool d){ //(a,b) is desired coord, c is velocity, d is wait for completion
  Controller1.Screen.print(X);
  Controller1.Screen.print(":");
  Controller1.Screen.print(Y);
  Controller1.Screen.newLine();
  Drivetrain.setDriveVelocity(c, pct);
  Drivetrain.setTurnVelocity(c, pct);

  faceHeading(c,(180*atan2((Y-b),(X-a))/M_PI));

  wait(50,msec);

  go(c, (sqrt(((X-a)*(X-a)) + (Y-b)*(Y-b))), d);
  
  Controller1.Screen.print(X);
  Controller1.Screen.print(":");
  Controller1.Screen.print(Y);
  Controller1.Screen.newLine();
}



