/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

/*---------------------------------------------------------------------------*/
/*                           PAUSE, READ THIS!                               */
/*THIS ROBOT IS THE SLAVE ROBOT IN THE ROBOT TO ROBOT COMMUNICATION HANDSHAKE*/
/*CODE FROM THIS ROBOT WILL BE DIFFERENT FROM THE MASTER'S CODE              */
/*FUTHERMORE, THIS ROBOT WILL FOLLOW INSTRUCTIONS FROM THE MASTER IF ORDERED */
/*---------------------------------------------------------------------------*/

#include "ai_functions.h"

using namespace vex;

brain Brain;
// Robot configuration code.
motor leftFront = motor(PORT11, ratio6_1, true);
motor leftMiddle = motor(PORT12, ratio6_1, true);
motor leftBack = motor(PORT13, ratio6_1, true);
motor rightFront = motor(PORT17, ratio6_1, false);
motor rightMiddle = motor(PORT18, ratio6_1, false);
motor rightBack = motor(PORT19, ratio6_1, false);
motor leftIntake = motor(PORT1, ratio6_1, false);
motor rightIntake = motor(PORT2, ratio6_1, true);
motor_group leftSide = motor_group(leftBack,leftFront,leftMiddle);
motor_group rightSide = motor_group(rightBack,rightFront,rightMiddle);
distance intakeSensor = distance(PORT4);
// controller Controller1 = controller(primary);
gps GPS = gps(PORT10, 17, 14, distanceUnits::cm, 90.0, turnType::right);
smartdrive Drivetrain = smartdrive(leftSide, rightSide, GPS, 10.205, 14, 11, inches, 1);

// Drive chassis(

// //Specify your drive setup below. There are eight options:
// //ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, and HOLONOMIC_TWO_ROTATION
// //For example, if you are not using odometry, put ZERO_TRACKER_NO_ODOM below:
// ZERO_TRACKER_NO_ODOM,

// //Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
// //You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

// //Left Motors:
// motor_group(leftFront,leftMiddle,leftBack),

// //Right Motors:
// motor_group(rightFront,rightMiddle,rightBack),

// //Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
// PORT3,

// //Input your wheel diameter. (4" omnis are actually closer to 4.125"):
// 3.25,

// //External ratio, must be in decimal, in the format of input teeth/output teeth.
// //If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
// //If the motor drives the wheel directly, this value is 1:
// 0.6,

// //Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
// //For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
// 360,

// /*---------------------------------------------------------------------------*/
// /*                                  PAUSE!                                   */
// /*                                                                           */
// /*  The rest of the drive constructor is for robots using POSITION TRACKING. */
// /*  If you are not using position tracking, leave the rest of the values as  */
// /*  they are.                                                                */
// /*---------------------------------------------------------------------------*/

// //If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

// //FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
// //LF:      //RF:    
// PORT1,     -PORT2,

// //LB:      //RB: 
// PORT3,     -PORT4,

// //If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
// //If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
// //If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
// 3,

// //Input the Forward Tracker diameter (reverse it to make the direction switch):
// 2.75,

// //Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
// //For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
// //This distance is in inches:
// -2,

// //Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
// 1,

// //Sideways tracker diameter (reverse to make the direction switch):
// -2.75,

// //Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
// 5.5

// );

// A global instance of competition
competition Competition;

// create instance of jetson class to receive location and other
// data from the Jetson nano
//
ai::jetson  jetson_comms;


/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot

//since this is the slave robot we do not define it as the manager 
// #define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT11, "robot_1248x", linkType::manager );
#else
#pragma message("building for the worker")
ai::robot_link       link( PORT6, "robot_1248x", linkType::worker );
#endif

/***********************************************************************************************************
 *                                          AUTONOMOUS PROGRAMMIG                                          *
 *  DESCRIPTION: THE CODE IN VEX AI IS VERY UNIQUE, WE CAN INPUT SENSOR DATA INTO NEURAL NETWORKS TO       *
 *   PROVIDE ARTIFICAL INTELLIGENCE TO CERTAIN TASKS. HOWEVER, WE CAN ALSO USE PRE-PROGRAMMED ROUTES AND   *
 *  COMMANDS TO GUIDE THE ROBOT. THEREFORE IT IS IMPORTANT TO DISCLOSE THAT THIS REP USES JAR TEMPLATE'S   *
 * CHASSIS COMMANDS FOR PRE-PROGRAMMED ACTIONS AND VEX AI'S SMARTDRIVE FOR AI BASED ACTIONS. IT IS CRUCIAL *
 *            THE PROGRAMMER KNOWS THE DIFFERENCE BETWEEN PRE-PROGRAMMED AND AI BASED ACTIONS.             *
 ***********************************************************************************************************/
void intake(double s) {
  leftIntake.spin(forward,s,percent);
  rightIntake.spin(forward,s,percent);
}
void outttake(double s) {
  leftIntake.spin(reverse,s,percent);
  rightIntake.spin(reverse,s,percent);
}
void intakeStop(const brakeType mode) {
  leftIntake.stop(mode);
  rightIntake.stop(mode);
}
void driveForTime(directionType dir,double v,double t) {
  Drivetrain.drive(dir,v,velocityUnits::pct);
  wait(t,seconds);
  Drivetrain.stop(brake);
}
// void chassisForward(double s) {
//   chassis.DriveL.spin(forward,s,percent);
//   chassis.DriveR.spin(forward,s,percent);
// }
// void chassisReverse(double s) {
//   chassis.DriveR.spin(reverse,s,percent);
//   chassis.DriveR.spin(reverse,s,percent);
// }
// void chassisStop(const brakeType mode) {
//   chassis.DriveL.stop(mode);
//   chassis.DriveR.stop(mode);
// }
// void chassisReverseFor(double t, double s) {
//   chassisReverse(s);
//   wait(t,seconds);
//   chassisStop(coast);
// }

void intakePossesion(double timeOut) { //run intakes untill ball is detected
  Brain.resetTimer();
  while (intakeSensor.objectDistance(mm) > 100) {
    intake(80);
    if (intakeSensor.objectDistance(mm) < 100 || Brain.timer(seconds) >= timeOut) {
      intakeStop(brake);
      break;
    }
  }
}
void intakeOut(double timeOut) { //outtake untill the ball is gone
  Brain.resetTimer();
  while (intakeSensor.objectDistance(mm) < 150) {
    outttake(100);
    if (intakeSensor.objectDistance(mm) > 200 || Brain.timer(seconds) >= timeOut) {
      wait(.5,seconds);
      intakeStop(hold);
      break;
    }
  }
}

int current_auton_selection = 0;
bool auto_started = false;

void pre_auton(void) {
  #pragma building the pre autonomous
  printf("preautonomous has begun");
  printf("babdf");
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();
  while(auto_started == false){            //Changing the names below will only change their names on the
    Brain.Screen.clearScreen();            //brain screen for auton selection.
    switch(current_auton_selection){       //Tap the brain screen to cycle through autons.
      case 0:
        Brain.Screen.printAt(50, 50, "Drive Test");
        break;
      case 1:
        Brain.Screen.printAt(50, 50, "Drive Test");
        break;
      case 2:
        Brain.Screen.printAt(50, 50, "Turn Test");
        break;
      case 3:
        Brain.Screen.printAt(50, 50, "Swing Test");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 4){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

/**********************************************************************************
 *                             ISOLATION PERIOD NOTES                             *
 * DESCRIPTION: WE WILL IMPLEMENT PRE PROGRAMMED ROUTES FOR THE ISOLATION PERIOD. *
 *         THEREFORE, CHASSIS WILL BE USED A BIT IN THIS SECTION OF CODE.         *
 *      HOWEVER, THE MAIN FOCUS IN THE INTERACTION PERIOD WILL BE AI BASED.       *
 **********************************************************************************/

void auto_Isolation(void) {
  auto_started = true;
  thread t1(dashboardTask); //added so we can use the auton selector when the robot is being held by field controller
  switch(current_auton_selection){  
    case 0:
      //This is the default auton, if you don't select from the brain.
      break;        //Change these to be your own auton functions in order to use the auton selector.
    case 1:         //Tap the screen to cycle through autons.
      
      break;
    case 2:
    
      break;
    case 3:
   
      break;
 }
  driveForTime(forward,100,1);
  intakePossesion(3);
  // Drivetrain.driveFor(forward,10,inches,100,velocityUnits::pct,false);

  // Calibrate GPS Sensor
  // GPS.calibrate();
  // // Optional wait to allow for calibration
  // wait(1,sec);
  // // Finds and moves robot to position of closest green triball
  // getObject();
  // // Intakes the ball
  // double rot = Arm.position(rotationUnits::deg);
  // intake(rot - 100, 1);
  // // Moves to position in front of blue goal
  // wait(1,sec);
  // goToGoal(0);
  // // Scores tri-ball in blue goal
  // dump(rot);
  
}

/*************************************************************************************************
 *                                    INTERACTION  PERIOD NOTES                                  *
 * DESCRIPTION: WE WILL IMPLEMENT AI BASED CODING AND ROUTES IN INTERACTION PERDIOD. THE MAJORITY*
 * OF THE TASKS ARE DONE BY AI AND CONDITIONALS. HOWEVER, A FEW COMMANDS WILL BE PRE-PROGRAMMED. *
 *  THANKFULLY, VEX PROVIDES US THE SMARTDRIVE CLASS SO WE WILL USE IT AND INPUT DATA FROM OUR   *
 *        SENSORS TO GIVE THE ROBOT ITS OWN 'INTELIGENCE' TO DO COMMANDS AND OBJECTIVES.         *
 *************************************************************************************************/

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                        Auto_Interaction Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous interaction*/
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void auto_Interaction(void) {
  // Functions needed: evaluate which ball detected is target, go to target (x,y), intake ball, dump ball, 
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = true;

void autonomousMain(void) {
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period. 
  // ..........................................................................

  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();

  firstAutoFlag = false;
}

int main() {

  // local storage for latest data from the Jetson Nano
  static AI_RECORD       local_map;

  // Run at about 15Hz
  int32_t loop_time = 33;

  // start the status update display
  // thread t1(dashboardTask);
  //commented out since we dont want this to run all the time. we want auton selector

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomousMain);

  // print through the controller to the terminal (vexos 1.0.12 is needed)
  // As USB is tied up with Jetson communications we cannot use
  // printf for debug.  If the controller is connected
  // then this can be used as a direct connection to USB on the controller
  // when using VEXcode.
  //
  // FILE *fp = fopen("/dev/serial2","wb");
  this_thread::sleep_for(loop_time);

  //we aint using an arm :(
  // Arm.setStopping(hold);
  // Arm.setVelocity(60, percent);

  while(1) {

      /*****************************************************************************************************************
       * IF SOMEONE DELETES THIS CODE THAT STARTS AT THE COMMENT, 'GET LAST...' TO 'THIS_THREAD::SLEEP_FOR(LOOP_TIME)',*
       *                   IM GOING TO ATTACK THEM. THAT IS THE JETSON CODE DO NOT TOUCH IT >:(                        *
       ****************************************************************************************************************/

      // get last map data
      jetson_comms.get_data( &local_map );

      // set our location to be sent to partner robot
      link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );

      // fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az);
      
      // request new data    
      // NOTE: This request should only happen in a single task.    
      jetson_comms.request_map();

      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
}