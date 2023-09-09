using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor fl; //front left wheel motor
extern motor bl; //black left wheel motor
extern motor fr; //front right wheel motor
extern motor br; //back right wheel motor
extern controller Controller1; 
extern controller Controller2;
extern motor flywheel; //disk shooter
extern motor_group leftDrive; //motor group of left wheel motors
extern motor_group rightDrive;//motor group of right wheel motors
extern smartdrive Drivetrain; //"smartly" configured group of all wheel motors with the use of inertial sensor 
extern motor intake; //disk collector
extern motor roller; //a motor that activates a wheel to spin to spin the roller
extern motor push;//disk loader that helps shoot the disks
extern inertial DrivetrainInertial;// inertial sensor for the drivetrain
extern digital_out expa1; //left pneumatic system activator for expansion system
extern digital_out expa2; //right pneumatic system activator for expansion system
extern optical iSeeColour; //optical sensor to recognize the color of the roller
extern encoder ltrack; //left wheel encoder 
extern encoder rtrack; //right wheel encoder
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );