#include "WPILib.h"
#include "DriveTrain.h"
#include "CamShooter.h"
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <fstream>
#include "Gamepad.h"
#include "Config.h"

using namespace std;

class RA14Robot : public IterativeRobot
{
private:
	CamShooter * myCam;
	CANJaguar * myJag;
	DriveTrain * myDrive;
	Relay * myCamera;
	Gamepad * DriverGamepad;
	Gamepad * OperatorGamepad;
	Compressor * myCompressor;
	float DriverLeftY;
	float DriverRightY;
	bool DriverLeftBumper;
	bool DriverRightBumper;
	
	DigitalOutput * CurrentSensorReset;
	
	bool ResetSetting;
	
	bool recentlyPressed;
	bool alreadyInitialized;
	
public:
  RA14Robot()
  {
	myCam = NULL;
	myDrive = NULL;  
	DriverGamepad = NULL;
	OperatorGamepad = NULL;	
	myCompressor = NULL;
	myCamera = NULL;
	DriverLeftY = 0.0;
	DriverRightY = 0.0;  
	DriverLeftBumper = true;
	DriverRightBumper = true;
	CurrentSensorReset = NULL;
	ResetSetting = false;
	recentlyPressed = false;
	alreadyInitialized = false;
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
	
	cout << "Initializing configuration..." << endl;
	Config::LoadFromFile("config.txt");
	cout << "Configuration read." << endl;
	cout << "Initializing compressor..." << endl;
	myCompressor = new Compressor(11,1);
	cout << "Compressor initialized." << endl;
	
	cout << "Initializing cam shooter..." << endl;
	myCam = new CamShooter(1, 1, 2, 3);
	//myJag = new CANJaguar(14,CANJaguar::kPercentVbus);
	cout << "Cam shooter initialized." << endl;
	
	cout << "Initializing drivetrain..." << endl;
	//myDrive = new DriveTrain(1,2,3,4,1,2,3,4,1,2,3,4);
	//myDrive = new DriveTrain(6,2,7,4,1,2,3,4 );
	cout << "Drivetrain initialized." << endl;
	
	cout << "Initializing gamepads..." << endl;
	DriverGamepad = new Gamepad(1);
	OperatorGamepad = new Gamepad(2);
	cout << "Gamepads initialized." << endl;
	
	cout << "Initializing CurrentSensor..." << endl;
	
	CurrentSensorReset = new DigitalOutput(5);
	myCamera = new Relay(2);
	
	cout << "CurrentSensor initialized." << endl;
	
	//cout << "Set period to 10Hz" << endl;
	this->SetPeriod(Config::GetSetting("robot_loop_period", 0.05));
	cout << "Period set to " << this->GetLoopsPerSec() << "Hz" << endl;
	
	cout << "Robot Init Complete..." << endl;
	
	
}

void RA14Robot::StartOfCycleMaintenance()
{
	//CurrentSensorReset->Set(0);
	
}

void RA14Robot::EndOfCycleMaintenance()
{
	//CurrentSensorReset->Set(1);
	ResetSetting = ! ResetSetting;
	CurrentSensorReset->Set(ResetSetting);
}

/**
 * Initialization code for disabled mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters disabled mode. 
 */
void RA14Robot::DisabledInit() {
	myCompressor->Stop();
	
	Config::LoadFromFile("config.txt");
	if (alreadyInitialized) {
		Config::Dump();
	}
	myCam->Reset();
	
	
}

/**
 * Periodic code for disabled mode should go here.
 * 
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in disabled mode.
 */
void RA14Robot::DisabledPeriodic() {
	StartOfCycleMaintenance();
	
	EndOfCycleMaintenance();
}

/**
 * Initialization code for autonomous mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters autonomous mode.
 */
void RA14Robot::AutonomousInit() {
	alreadyInitialized = true;
}

/**
 * Periodic code for autonomous mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in autonomous mode.
 */
void RA14Robot::AutonomousPeriodic() {
	StartOfCycleMaintenance();
	
	EndOfCycleMaintenance();
}

/**
 * Initialization code for teleop mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters teleop mode.
 */
void RA14Robot::TeleopInit() {
	myCompressor->Start();
	alreadyInitialized = true;
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 */
void RA14Robot::TeleopPeriodic() 
{
	
	StartOfCycleMaintenance();
	DriverLeftY = DriverGamepad->GetLeftY();
	DriverRightY = DriverGamepad->GetRightY();
	DriverLeftBumper = DriverGamepad->GetLeftBumper(); // Reads state of left bumper 
	DriverRightBumper = DriverGamepad->GetRightBumper(); // Gets state of right bumper
	
	bool ShouldFire = DriverGamepad->GetRightTrigger();
	
	myCam->Process(ShouldFire);
	
	if(DriverGamepad->GetA())
	{
		//myCam->SetPosition(true);
		//myJag->Set(-0.25);
		myCam->SetPosition(90);
		cout<<"Forward"<<endl;
	}
	if(DriverGamepad->GetB())
	{
		//myCam->SetPosition(false);
		//myJag->Set(0.25);
		myCam->SetPosition(110);
		cout<<"Backwards"<<endl;
	}
	
	if(DriverGamepad->GetY())
	{
		recentlyPressed = true;
	}
	
	if(!DriverGamepad->GetY() && recentlyPressed)
	{
		if(myCamera->Get() == Relay::kOff)
			myCamera->Set(Relay::kForward);
		else
			myCamera->Set(Relay::kOff);
		
		recentlyPressed = false;
	}
	//cout<<"The position is "<<myCam->GetPosition()<<endl;
#if 0
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
#endif
	myCam->Debug(cout);
	
	EndOfCycleMaintenance();
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
void RA14Robot::TestPeriodic()
{
	StartOfCycleMaintenance();
	
	EndOfCycleMaintenance();
}



};

START_ROBOT_CLASS(RA14Robot);
