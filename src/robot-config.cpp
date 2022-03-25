#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor RightDriveMotorA = motor(PORT19, ratio6_1, true);
motor RightDriveMotorB = motor(PORT20, ratio6_1, true);
motor_group RightDrive = motor_group(RightDriveMotorA, RightDriveMotorB);
motor LeftDriveMotorA = motor(PORT11, ratio6_1, false);
motor LeftDriveMotorB = motor(PORT12, ratio6_1, false);
motor_group LeftDrive = motor_group(LeftDriveMotorA, LeftDriveMotorB);
/*vex-vision-config:begin*/
vision Vision = vision (PORT7, 50);
/*vex-vision-config:end*/
sonar Distance = sonar(Brain.ThreeWirePort.C);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}