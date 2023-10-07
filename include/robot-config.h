using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor leftfront;
extern motor rightfront;
extern motor leftback;
extern motor rightback;
extern rotation right_rot;
extern inertial imu;
extern rotation front_rot;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );