//headers for all autonomous methods
extern bool enableDrivePD;
void spinRoller(int b, double a);

void Skills();
void turnRight(double a, double b);
void turnLeft(double a, double b);
void l();
void r();
void strafeR();
void strafeL();
void PD(double a,double t);
void PD(double a, double p, double d, double t);

int PDdrive();
extern double facing;