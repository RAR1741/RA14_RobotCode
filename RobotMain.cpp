#include "WPILib.h"
#include "DriveTrain.h"
#include "CamShooter.h"
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <fstream>
#include "Gamepad.h"

using namespace std;

class RA14Robot : public IterativeRobot
{
private:
	CamShooter * myCam;
	DriveTrain * myDrive;
	Gamepad * DriverGamepad;
	Gamepad * OperatorGamepad;
	Compressor * myCompressor;
	float DriverLeftY;
	float DriverRightY;
	bool DriverLeftBumper;
	bool DriverRightBumper;
	
public:
  RA14Robot()
  {
	myCam = NULL;
	myDrive = NULL;  
	DriverGamepad = NULL;
	OperatorGamepad = NULL;	
	myCompressor = NULL;
	DriverLeftY = 0.0;
	DriverRightY = 0.0;  
	DriverLeftBumper = true;
	DriverRightBumper = true;
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
	
	myCompressor = new Compressor(11,1);

	myCam = new CamShooter(9);
	
	myDrive = new DriveTrain(1,2,3,4,1,2,3,4,1,2,3,4);
	//myDrive = new DriveTrain(6,2,7,4,1,2,3,4 );
	
	cout << "Initializing gamepads..." << endl;
	DriverGamepad = new Gamepad(1);
	OperatorGamepad = new Gamepad(2);
	cout << "Gamepads initialized." << endl;
	
	cout << "Robot Init Complete..." << endl;
	
	
}

/**
 * Initialization code for disabled mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters disabled mode. 
 */
void RA14Robot::DisabledInit() {
	myCompressor->Stop();
	
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
	myCompressor->Start();
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 */
void RA14Robot::TeleopPeriodic() 
{
	
	DriverLeftY = DriverGamepad->GetLeftY();
	DriverRightY = DriverGamepad->GetRightY();
	DriverLeftBumper = DriverGamepad->GetLeftBumper(); // Reads state of left bumper 
	DriverRightBumper = DriverGamepad->GetRightBumper(); // Gets state of right bumper
	
	
	if(DriverGamepad->GetA())
	{
		myCam->SetPosition(true);
		cout<<"Forward"<<endl;
	}
	if(DriverGamepad->GetB())
	{
		myCam->SetPosition(false);
		cout<<"Backwards"<<endl;
	}

	//cout<<"The position is "<<myCam->GetPosition()<<endl;
	
	if(DriverLeftBumper)
	{
		myDrive->ShiftUp();
	}
	else if(DriverRightBumper)
	{
		myDrive->ShiftDown();
	}
	
	myDrive->Drive(DriverLeftY, DriverRightY);
	
	myDrive->Debug(cout);
	
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
