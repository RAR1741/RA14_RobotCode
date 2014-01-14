#include "WPILib.h"
#include "DriveTrain.h"
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <fstream>
#include "Gamepad.h"


class RA14Robot : public IterativeRobot
{
	
	
private:
	DriveTrain * myDrive;
	Gamepad * DriverGamepad;
	Gamepad * OperatorGamepad;
	float DriverLeftY;
	float DriverLeftX;
	float DriverRightY;
	float DriverRightX;
	
	
	
	
public:
  RA14Robot()
  {
	myDrive = NULL;  
	DriverGamepad = NULL;
	OperatorGamepad = NULL;	
	  
	  
	  
	  
	  
  }
  
/**
 * Robot-wide initialization code should go here.
 * 
 * Use this method for default Robot-wide initialization which will
 * be called when the robot is first powered on.  It will be called exactly 1 time.
 */
void RA14Robot::RobotInit() {
	
	SmartDashboard::init();
	cout << "2014 Red Alert Robot" << endl;
	cout << "Compiled on: ";
	cout << __DATE__ << " at " << __TIME__ << endl;
	myDrive = new DriveTrain(1,2,3,4,1,2,3,4);
	DriverGamepad = new Gamepad(1);
	OperatorGamepad = new Gamepad(2);
	
	
	
	
	
	cout << "Robot Init Complete..." << endl;
}

/**
 * Initialization code for disabled mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters disabled mode. 
 */
void RA14Robot::DisabledInit() {
}

/**
 * Periodic code for disabled mode should go here.
 * 
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in disabled mode.
 */
void RA14Robot::DisabledPeriodic() {
}

/**
 * Initialization code for autonomous mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters autonomous mode.
 */
void RA14Robot::AutonomousInit() {
}

/**
 * Periodic code for autonomous mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in autonomous mode.
 */
void RA14Robot::AutonomousPeriodic() {
}

/**
 * Initialization code for teleop mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters teleop mode.
 */
void RA14Robot::TeleopInit() {
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 */
void RA14Robot::TeleopPeriodic() {
	
	DriverLeftY = DriverGamepad->GetLeftY();
	DriverRightY = DriverGamepad->GetRightY();
	
	
	
	
	
	
	myDrive->Drive(DriverLeftY, DriverRightY);
	
	
	
	
	
	
}

/**
 * Initialization code for test mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters test mode.
 */
void RA14Robot::TestInit() {
}

/**
 * Periodic code for test mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in test mode.
 */
void RA14Robot::TestPeriodic() {
}

};

START_ROBOT_CLASS(RA14Robot);

