/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       4492A                                                     */
/*    Created:      Thu Mar 24 2022                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// RightDrive           motor_group   19, 20          
// LeftDrive            motor_group   11, 12          
// Vision               vision        7               
// Distance             sonar         C, D            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

competition Competition;

void pre_auton(void) {
  vexcodeInit();
  Brain.Screen.clearScreen();
  RightDrive.setStopping(brake);
  LeftDrive.setStopping(brake);
}

///////////////////////////////////////////////////////////////////////////////

// READ FIRST BEFORE USING!

//Before use:

//Calibrate the Vision Sensor for RED objects, or any color as long as the color sig is named REDD

//Motors are four 600rpm motors geared 36/60, two on each side.

//Wheels are 4in Omniwheels.

//PID constants have not been tuned that much, so tinker with them a bit.

//Explanation for the code is to the right of it.

//Have fun and good luck in future VRC games!

///////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////

//PID Values & Variables

///////////////////////////////////////////////////////////////////////////////

float Kp = 2;
float Ki = 0.02;                                                                    // PID Constants for Drive
float Kd = 3;

float fKp = 7;
float fKi = 0.1;                                                                    // PID Constants for Facing
float fKd = 3;

float currError = 0;                                                                //Sensor Value - Desired Value = Positional Value
float prevError = 0;                                                                //Postional Value 20ms ago / preverror = error
float derivative = 0;                                                               //currError - prevError = Slope *Roughly*
float integral = 0;                                                                 //Total adds up all currError after each cycle / Integral

///////////////////////////////////////////////////////////////////////////////

//PID Drive Controller

///////////////////////////////////////////////////////////////////////////////

void drivePIDForward(float DriveDistance){

  LeftDrive.setPosition(0, degrees);                                                 //Resets motor values before each drive. 
  RightDrive.setPosition(0, degrees);

  DriveDistance = DriveDistance * 60/36 * 12.57 * 2.1;                               //Rough calculation for motor turns to inches.

  float AvgPos = ((LeftDrive.position(degrees) + RightDrive.position(degrees)) / 2); //Finds average position of Left and Right sides.

  currError = DriveDistance - AvgPos;

  while(currError > .5){                                                             //Loops until the robot is within .5 inches of the target value.

    AvgPos = ((LeftDrive.position(degrees) + RightDrive.position(degrees)) / 2);

    currError = DriveDistance - AvgPos;                                              //Gets new PID values (not the same at the constants).
    derivative = currError - prevError;
    integral = integral + currError;
    prevError = currError;

    double MotorPwr = (currError * Kp + integral * Ki + derivative * Kd) / 100 ;     //Dividing by 100 so constants are above 1 and easier to tune.

    if (MotorPwr > 50){                                                              //Sets maximum motor value (can be changed).

      MotorPwr = 50;

    }

    RightDrive.setVelocity(MotorPwr, percent);
    LeftDrive.setVelocity(MotorPwr, percent);

    RightDrive.spin(forward);
    LeftDrive.spin(forward);
      
    wait(20, msec);                                                                  //Sleeps for a short amount of time to not overlod the system.

  }

  RightDrive.stop();
  LeftDrive.stop();

  currError = 0;                                                                     //Resets PID values for next drive / facing.
  derivative = 0;
  integral = 0;
  prevError = 0;

}

void drivePIDReverse(float DriveDistance){

  currError = 0;
  derivative = 0;
  integral = 0;
  prevError = 0;
  LeftDrive.setPosition(0, degrees);
  RightDrive.setPosition(0, degrees);

  DriveDistance = DriveDistance * 60/36 * 12.57 * 2.1;

  float AvgPos = ((LeftDrive.position(degrees) + RightDrive.position(degrees))/-2);  //Has to be multipled by -1 or average positon and error will be negative and motors will not spin.

  currError = DriveDistance - AvgPos;

  while(currError > .5){

    AvgPos = ((LeftDrive.position(degrees) + RightDrive.position(degrees))/-2);

    currError = DriveDistance - AvgPos;
    derivative = currError - prevError;
    integral = integral + currError;
    prevError = currError;

    double MotorPwr = (currError * Kp + integral * Ki + derivative * Kd) /100 ;

    if (MotorPwr > 50){

      MotorPwr = 50;

    }

    RightDrive.setVelocity(MotorPwr, percent);
    LeftDrive.setVelocity(MotorPwr, percent);

    RightDrive.spin(reverse);
    LeftDrive.spin(reverse);
      
    wait(20, msec);

  }

  RightDrive.stop();
  LeftDrive.stop();

  currError = 0;
  derivative = 0;
  integral = 0;
  prevError = 0;

  wait(20, msec);

}

///////////////////////////////////////////////////////////////////////////////

// Face Object w/ PID

///////////////////////////////////////////////////////////////////////////////

void FaceRed(){

  Vision.takeSnapshot(Vision__REDD);                                                 //Takes picture of only REDD signature (you will have to tune your vision sensor).           

  while (Vision.largestObject.centerX < 157 or Vision.largestObject.centerX > 159){  //Loop runs while the object is outside the center of the vision sensor.

    Vision.takeSnapshot(Vision__REDD);

    if (Vision.largestObject.exists){                                                //Will run command if the Vision Sensor sees the object.

      Vision.takeSnapshot(Vision__REDD);

      currError = 0;
      derivative = 0;
      integral = 0;
      prevError = 0;
    
      if (Vision.largestObject.centerX > 158){                                       //If object in to the right of the robot.

        currError = Vision.largestObject.centerX - 158;

        while(currError > 1){                                                        //PID as before except error calulated from Vision Sensor inputs.

          Vision.takeSnapshot(Vision__REDD);

          currError = Vision.largestObject.centerX - 158;
          derivative = currError - prevError;
          integral = integral + currError;
          prevError = currError;

          double MotorPwr = (currError * fKp + integral * fKi + derivative * fKd) /100 ;

          if (MotorPwr > 10){

            MotorPwr = 10;

          }

          RightDrive.setVelocity(MotorPwr, percent);                               
          LeftDrive.setVelocity(MotorPwr, percent);

          RightDrive.spin(reverse);
          LeftDrive.spin(forward);

          wait(20, msec);

        }

      }

      if (Vision.largestObject.centerX < 158){                                        //If object is to the left of the robot.

        currError = 158 - Vision.largestObject.centerX;

        while(currError > 1){

          Vision.takeSnapshot(Vision__REDD);

          currError = 158 - Vision.largestObject.centerX;
          derivative = currError - prevError;
          integral = integral + currError;
          prevError = currError;

          double MotorPwr = (currError * fKp + integral * fKi + derivative * fKd) /100 ;

          if (MotorPwr > 10){
            MotorPwr = 10;
          }
          
          RightDrive.setVelocity(MotorPwr, percent);
          LeftDrive.setVelocity(MotorPwr, percent);

          RightDrive.spin(forward);
          LeftDrive.spin(reverse);

          wait(20, msec);

        }

      }

    }

    else {

      RightDrive.setVelocity(5, percent);                                             //Turns to the left slowly to allow Vision pickup of the object (can be changed).
      LeftDrive.setVelocity(5, percent);                                              //Idle animation while Vision does not see any object.

      RightDrive.spin(forward);
      LeftDrive.spin(reverse);

    }

    wait(20, msec);

  }

  RightDrive.stop();
  LeftDrive.stop();

  currError = 0;
  derivative = 0;
  integral = 0;
  prevError = 0;

  wait(20, msec);
  
}

///////////////////////////////////////////////////////////////////////////////

// Drive to Object w/ PID

///////////////////////////////////////////////////////////////////////////////

void DriveToObject(float dist) {                                                         //Dist is used to ask how close you want to robot to get to the object.

  while(Distance.distance(inches) < dist - .5 or Distance.distance(inches) > dist + .5){ //PID as before but with input from Rangefinder to determine Drive distance. 

    if (Distance.foundObject()){

      if (Distance.distance(inches) > dist) {                                            //If the object is far away, drive forward.

        drivePIDForward(Distance.distance(inches) - dist);                               //Uses same Drive PID controller from before.
      }

      if (Distance.distance(inches) < dist) {                                            //If object is too close, reverse.

        drivePIDReverse(dist - Distance.distance(inches));
      }

    }

    else{                                                                                //If the object is too far away for the Distance Sensor to pick up, it will drive forward until it does.

      RightDrive.setVelocity(30, percent);
      LeftDrive.setVelocity(30, percent);

      RightDrive.spin(forward);
      LeftDrive.spin(forward);
      
    }

  }

  wait(20, msec);

}

///////////////////////////////////////////////////////////////////////////////

// Autonomous And Drive Functions

///////////////////////////////////////////////////////////////////////////////

//Autonomous

void autonomous(void) {
  FaceRed();
  DriveToObject(3);                                                                   //Input is for distance to object in inches.
  FaceRed();
}

//Driver Control

void usercontrol(void) {

  while (1) {

  }  

}

int main() {

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }

}
