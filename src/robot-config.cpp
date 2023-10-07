#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftfront = motor(PORT1, ratio18_1, true);
motor rightfront = motor(PORT9, ratio18_1, false);
motor leftback = motor(PORT2, ratio18_1, true);
motor rightback = motor(PORT10, ratio18_1, false);
rotation right_rot = rotation(PORT7, false);
inertial imu = inertial(PORT6);
rotation front_rot = rotation(PORT8, true);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}