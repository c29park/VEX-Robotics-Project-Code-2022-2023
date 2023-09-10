extern bool enableDrivePD;
void spinRoller(int b, double a);
extern double deltaL,deltaR,deltaB,currentL,currentR, currentB, previousL,previousR, previousB, deltaTheta,X,Y,theta,robotHeading;
void Skills();
void spinExpa();
void turnRight(double a, double b);
void turnLeft(double a, double b);
void l();
void r();
void strafeR();
void strafeL();
void PD(double a,double t);
void PD(double a, double p, double d, double t);
int positionTracking();
int PDdrive();
extern double facing;
extern float slewRate;
extern double X;
extern double Y;
extern double theta;
void goTo(double a, double b, double c, bool d);
void backToPoint(float x, float y, float yspeed, float hspeed, float ykp, float hkp, float breakLength);
