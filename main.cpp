/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
controller Cntrl;

motor LF(PORT6);
motor LM(PORT8, true);
motor LB(PORT10, true);
motor RF(PORT1, true);
motor RM(PORT4);
motor RB(PORT5);
motor intake(PORT20, true);
triport triwire(PORT22);
pneumatics clamp(triwire.A);

motor_group Left(LF, LM, LB);
motor_group Right(RF,RM,RB);

inertial Inertial(PORT12);
// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/



float motorAverage() {
  return (RF.position(deg) + RM.position(deg) + RB.position(deg) +
          LF.position(deg) + LM.position(deg) + LB.position(deg)) /
         6;
}
// relative positioning is faster and easier to code but we lose accuracy
void motorReset() { // we need to reset the motor's rotation after every move
                    // because of relative position
  RF.resetPosition();
  LF.resetPosition();
  RM.resetPosition();
  LM.resetPosition();
  RB.resetPosition();
  LB.resetPosition();
}
void driveStraight(float dist, int maxspeed) {
  motorReset();
  ////////////////////////////////////
  // Only Change These 3 Values Below//
  ///////////////////////////////////
  float kp = .475
  ;   // proportional gain
  float ki = .0003; // integral gain
  float kd = .4;    // derivative gain

  float error = dist - motorAverage(); // error(how big is the error)
  float p = kp * error;                // proportional speed calcuation

  float sumDist = 0;      // sum of the distance traveled (total)
  float i = ki * sumDist; // integral speed calcuation

  float lastError = error; // last error, at the beginning this is = to current
                           // error to not mess up calculations
  float deltaError = lastError - error; // change in error overtime/update
  float d = kd * deltaError;            // derivative speed calcuation

  float speed = p + i + d;

  while (fabs(error) >= 15) {
    error = dist - motorAverage(); // error(how big is the error)
    p = kp * error;                // proportional speed calcuation

    // sumDist = 0;//sum of the distance traveled (total)
    i = ki * sumDist; // integral speed calcuation

    deltaError = error - lastError; // change in error overtime/update
    d = kd * deltaError;
    if (speed > maxspeed) {
      speed = maxspeed;
    } else {
      speed = p + i + d;
    }
    // printf("speed: %f\n",speed);
    printf("p: %f\n", p);
    printf("i: %f\n", i);
    printf("d: %f\n", d);
    printf("speed: %f\n", speed);
    printf("dist: %f\n", motorAverage());
    LF.spin(fwd, speed * .12, volt);
    LM.spin(fwd, speed * .12, volt);
    LB.spin(fwd, speed * .12, volt);
    RF.spin(fwd, speed * .12, volt);
    RM.spin(fwd, speed * .12, volt);
    RB.spin(fwd, speed * .12, volt);
    wait(20, msec);
    lastError = error; // last error, at the beginning this is = to current
                       // error to not mess up calculations
    sumDist += error;
  }
  printf("NICE");
  LF.stop(brake);
  LM.stop(brake);
  LB.stop(brake);
  RF.stop(brake);
  RM.stop(brake);
  RB.stop(brake);
}
void turning(double ang, double accuracy = 0.5) { //-100
motorReset();
  float kp = .39725;                                  // proportional gain
  float ki = 0.0000015;                            // integral gain
  float kd = 0; // derivative gain doesnt do anything in this

  float error = ang - Inertial.rotation(); // error(how big is the error)
  float p = kp * error;                    // proportional speed calcuation

  float sumDist = 0;      // sum of the distance traveled (total)
  float i = ki * sumDist; // integral speed calcuation

  double lastError = error; // last error, at the beginning this is = to current
                            // error to not mess up calculations
  double deltaError = lastError - error; // change in error overtime/update
  double d = kd * deltaError;            // derivative speed calcuation

  float speed = p + i + d;
  while (Inertial.rotation() < (ang - accuracy) ||
         Inertial.rotation() >
             (ang + accuracy)) { // This while loop checks if the current sensor
                                 // value is inbetween the acceptable range,
    error = ang - Inertial.rotation(); // error(how big is the error)
    p = kp * error;                    // proportional speed calcuation

    // sumDist = 0;//sum of the distance traveled (total)
    i = ki * sumDist; // integral speed calcuation

    deltaError = error - lastError; // change in error overtime/update
    d = kd * deltaError;
    speed = p + i + d;
    printf("p: %f\n", p);
    printf("i: %f\n", i);
    printf("d: %f\n", d);
    printf("speed: %f\n", speed);
    printf("inertial: %f\n", Inertial.rotation());

    LF.spin(fwd, speed, pct); // if it is not in the range, the robot will
                              // attempt to correct by moving towards the range.
    LM.spin(fwd, speed, pct);
    LB.spin(fwd, speed, pct);
    RF.spin(fwd, -speed, pct);
    RB.spin(fwd, -speed, pct);
    RM.spin(
        fwd, -speed,
        pct); // speed has to be reversed because thats how you turn the robot
    lastError = error; // last error, at the beginning this is = to current
                       // error to not mess up calculations
    sumDist += error;
  }
  // printf("speed: %f\n",speed);
  LF.stop(brake);
  LM.stop(brake);
  LB.stop(brake);
  RF.stop(brake);
  RM.stop(brake);
  RB.stop(brake);
}
void noPIDDrive(double seconds, float volts, int fowards = 1) {
  LF.spin(fwd, fowards * volts, volt);
  LM.spin(fwd, fowards * volts, volt);
  LB.spin(fwd, fowards * volts, volt);
  RF.spin(fwd, fowards * volts, volt);
  RB.spin(fwd, fowards * volts, volt);
  RM.spin(fwd, fowards * volts, volt);
  wait(seconds,sec);
  LF.stop(coast);
  LM.stop(coast);
  LB.stop(coast);
  RF.stop(coast);
  RB.stop(coast);
  RM.stop(coast);
}

void autonomous(void) {
 driveStraight(-550,80);
 clamp.open();
 wait(0.5,seconds);
 clamp.close();
 turning(75);
 //not tested past this
 driveStraight(400,100);
 intake.spin(fwd,12,volt);
 wait(0.5,seconds);
 intake.stop();
 driveStraight(100,100);
 intake.spin(fwd,12,volt);
 wait(0.5,seconds);
 intake.stop();
 turning(-50);
 driveStraight(300,100);
 turning(30);
 driveStraight(200, 100);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

// void Drive(){
//   LB.spin(fwd,(Cntl.Axis3.position(pct)-Cntl.Axis1.position(pct)),pct);
//   LM.spin(fwd,(Cntl.Axis3.position(pct)-Cntl.Axis1.position(pct)),pct);
//   LF.spin(fwd,(Cntl.Axis3.position(pct)-Cntl.Axis1.position(pct)),pct);


//   RB.spin(fwd,(Cntl.Axis3.position(pct)+Cntl.Axis1.position(pct)),pct);
//   RM.spin(fwd,(Cntl.Axis3.position(pct)+Cntl.Axis1.position(pct)),pct);
//   RF.spin(fwd,(Cntl.Axis3.position(pct)+Cntl.Axis1.position(pct)),pct);
// }
double s_Speed = 0.0;
double s_Turn = 0.0;

void arcadeDrive(double speed, double turn) {
  speed /= 100.0;
  turn /= 100.0;

  speed *= fabs(speed);
  turn *= fabs(turn);

  double turnAdj = fmax(0.6, fabs(speed));

  turn *= turnAdj;

  speed *= 13.0;
  turn *= 13.0;

  s_Speed += (speed - s_Speed) * 0.5;
  s_Turn += (turn - s_Turn) * 0.75;

  const double left = s_Speed + s_Turn;
  const double right = s_Speed - s_Turn;

  RF.spin(fwd, right, volt);
  RM.spin(fwd, right, volt);
  RB.spin(fwd, right, volt);

  LF.spin(fwd, left, volt);
  LM.spin(fwd, left, volt);
  LB.spin(fwd, left, volt);
}

void Drive(){
  double speed = Cntrl.Axis3.position(pct);
  double turn = Cntrl.Axis1.position(pct);

  arcadeDrive(speed, turn);
}

void Intake(){
  if(Cntrl.ButtonR1.pressing()){
    intake.spin(fwd,12,volt);
  }
  else if (Cntrl.ButtonR2.pressing()){
    intake.spin(fwd,-12,volt);
  }
  else{
    intake.stop(coast);
  }
}

void Clamp(){
  if(Cntrl.ButtonL1.pressing()){
    clamp.open();
  }
  else if(Cntrl.ButtonL2.pressing()){
    clamp.close();
  }
}
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    Drive();
    Intake();
    Clamp();
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
