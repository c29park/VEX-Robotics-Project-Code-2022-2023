void Drive(int left, int right);

extern int yMax;
extern int hMax;
extern int driveCap;

//Driver
void DriverControls();
//methods for all systems 
void Roller(); 
void Flywheel(); 
void Intakes();
void Shoot();
void expansion();
void vib(); //controller rumble function
void PID(double a, double t);
