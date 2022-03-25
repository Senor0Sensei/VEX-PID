using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;
extern motor_group RightDrive;
extern motor_group LeftDrive;
extern signature Vision__REDD;
extern signature Vision__BLU;
extern signature Vision__YEL;
extern signature Vision__SIG_4;
extern signature Vision__SIG_5;
extern signature Vision__SIG_6;
extern signature Vision__SIG_7;
extern vision Vision;
extern sonar Distance;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );