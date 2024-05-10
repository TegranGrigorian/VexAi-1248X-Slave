using namespace vex;

extern brain Brain;


extern gps GPS;
extern smartdrive Drivetrain;
extern controller Controller;
extern motor leftIntake;
extern motor rightIntake;
extern motor leftFront;
extern motor leftMiddle;
extern motor leftBack;
extern motor rightMiddle;
extern motor rightFront;
extern motor rightBack;
void intakePossesion();
void intakeStop();
void intakeOut();
void driveForTime();
void outttake();
void intake();
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
