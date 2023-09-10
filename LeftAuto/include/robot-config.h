using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor fl;
extern motor bl;
extern motor fr;
extern motor br;
extern controller Controller1;
extern controller Controller2;
extern motor flywheel;
extern motor_group leftDrive;
extern motor_group rightDrive;
extern smartdrive Drivetrain;
extern motor intake;
extern motor roller;
extern motor push;
extern inertial DrivetrainInertial;
extern digital_out expa1;
extern digital_out expa2;
extern optical iSeeColour;
extern encoder ltrack;
extern encoder rtrack;
extern encoder btrack;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );