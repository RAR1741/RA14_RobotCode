#include "WPILib.h"
#include "DriveTrain.h"
#include "CamShooter.h"
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <exception>
#include "Gamepad.h"
#include "Config.h"
#include "CurrentSensor.h"
#include "TargetServer.h"
#include "WPIErrors.h"
#include "SpeedControlTalon.h"
#include "Collection.h"
#include "Target.h"

using namespace std;

class RA14Robot : public IterativeRobot
{
private:
	ofstream fout;
	CamShooter * myCam;
	CANJaguar * myJag;
	DriveTrain * myDrive;
	CurrentSensor * myCurrentSensor;
	Relay * myCamera;
	Gamepad * DriverGamepad;
	Gamepad * OperatorGamepad;
	Compressor * myCompressor;
	Collection * myCollection;
	float DriverLeftY;
	float DriverRightY;
	bool DriverLeftBumper;
	bool DriverRightBumper;
	bool ShouldFireButton;
	bool RingLightButton;
	bool BallCollectPickupButton;
	
	DigitalOutput * CurrentSensorReset;
	
	bool ResetSetting;
	
	bool RingLightButtonRecentlyPressed;
	bool alreadyInitialized;
	
	TargetServer * server;
	Target * target;
	
public:
  RA14Robot()
  {
	myCurrentSensor = NULL;
	myCam = NULL;
	myDrive = NULL;  
	DriverGamepad = NULL;
	OperatorGamepad = NULL;	
	myCompressor = NULL;
	myCamera = NULL;
	myCollection = NULL;
	DriverLeftY = 0.0;
	DriverRightY = 0.0;  
	DriverLeftBumper = true;
	DriverRightBumper = true;
	CurrentSensorReset = NULL;
	ResetSetting = false;
	RingLightButtonRecentlyPressed = false;
	RingLightButton = false;
	alreadyInitialized = false;
	ShouldFireButton = false;
	BallCollectPickupButton = false;
	
	server = NULL;
	target = NULL;
  }
  
/**
 * Robot-wide initialization code should go here.
 * 
 * Use this method for default Robot-wide initialization which will
 * be called when the robot is first powered on.  It will be called exactly 1 time.
 */
void RA14Robot::RobotInit() {

	cout << "Initializing SmartDashboard..." << endl;
	SmartDashboard::init();	
	cout << "SmartDachboard Initialized" << endl;
	
	cout << "Initializing configuration..." << endl;
	Config::LoadFromFile("config.txt");
	cout << "Configuration read." << endl;
	
	cout << "Initializing compressor..." << endl;
	myCompressor = new Compressor(11,1);
	cout << "Compressor initialized." << endl;
	
	cout << "Initializing cam shooter..." << endl;
	myCam = new CamShooter(5, 6, 2, 1, 3);
	cout << "Cam shooter initialized." << endl;
	
	cout << "Initializing drivetrain..." << endl;
	myDrive = new DriveTrain(1,2,3,4, 1,2,3,4, 4,5,6,7);
	cout << "Drivetrain initialized." << endl;
	
	cout<<"Initializing current sensor.."<<endl;
	myCurrentSensor = new CurrentSensor(7);
	cout<<"Current sensor initialized."<<endl;
	
	cout<<"Initializing collection system"<<endl;
	myCollection = new Collection(7,3,4);
	cout<<"Collection system initialized"<<endl;
	
	cout << "Initializing gamepads..." << endl;
	DriverGamepad = new Gamepad(1);
	OperatorGamepad = new Gamepad(2);
	cout << "Gamepads initialized." << endl;
	
	cout << "Initializing CurrentSensor..." << endl;
	CurrentSensorReset = new DigitalOutput(12);
	myCamera = new Relay(2);
	cout << "CurrentSensor initialized." << endl;
	
	cout << "Target Server initializing..." << endl;
	server = new TargetServer();
	target = new Target();
	cout << "Target server initialized." << endl;
	
	this->SetPeriod(Config::GetSetting("robot_loop_period", 0.05));
	cout << "Period set to " << this->GetLoopsPerSec() << "Hz" << endl;
	
	cout << "2014 Red Alert Robot" << endl;
	cout << "Compiled on: ";
	cout << __DATE__ << " at " << __TIME__ << endl;
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
	
	if(fout.is_open()) {
		cout << "Closing logging.csv..." << endl;
		fout.close();
	}
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
	if(!fout.is_open()) {
		cout << "Opening logging.csv..." << endl;
		fout.open("logging.csv");
		logheaders();
	}
	myCam->PIDEnable();
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
	if(!fout.is_open()) {
		cout << "Opening logging.csv..." << endl;
		fout.open("logging.csv");
		logheaders();
	}
	myCam->PIDEnable();
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
	
	//Input Acquisition
	DriverLeftY = DriverGamepad->GetLeftY();
	DriverRightY = DriverGamepad->GetRightY();
	DriverLeftBumper = DriverGamepad->GetLeftBumper(); // Reads state of left bumper 
	DriverRightBumper = DriverGamepad->GetRightBumper(); // Gets state of right bumper
	RingLightButton = DriverGamepad->GetY();
	ShouldFireButton = DriverGamepad->GetRightTrigger();
	BallCollectPickupButton = DriverGamepad->GetBack();
	//End Input Acquisition
	
	
	//Current Sensing
	myCurrentSensor->Toggle(DriverGamepad->GetStart());
	//End Current Sensing
	
	
	//Target Processing
	target->Parse(server->GetLatestPacket());
	
	
	/*
	if (target->IsValid()) {
		cout << "\tx = " << target->GetX() << ", y = " << target->GetY() << ", distance = " << target->GetDistance() << "ft";
		cout << "\tside = " << (target->IsLeft() ? "LEFT" : "RIGHT") << ", hot = " << (target->IsHot() ? "HOT" : "NOT") << endl;
	}
	else {
		cout << "No Target Received..." << endl;
	}
	*/
	
	//End Target Processing
	
	
	//Ball Collection
	if(BallCollectPickupButton)
	{
		myCollection->Collect();
	}
	else
	{
		myCollection->ResetPosition();
	}
	//End Ball Collection
	
	
	//Fire Control
	myCam->Process(ShouldFireButton);
	
	myCam->Debug(cout);
	
	//End Fire Control
	
	
	//Ring Light
	if(RingLightButton)
	{
		RingLightButtonRecentlyPressed = true;
	}
	
	if(!RingLightButton && RingLightButtonRecentlyPressed)
	{
		if(myCamera->Get() == Relay::kOff)
			myCamera->Set(Relay::kForward);
		else
			myCamera->Set(Relay::kOff);
		
		RingLightButtonRecentlyPressed = false;
	}
	//End Ring Light

	
	//Drive Processing
	if(DriverLeftBumper)
	{
		myDrive->ShiftUp();
	}
	else if(DriverRightBumper)
	{
		myDrive->ShiftDown();
	}
	
	myDrive->Drive(DriverLeftY, DriverRightY);
	//End Drive Processing
	
	logging();
	target->Parse("");
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

void RA14Robot::logheaders()
{
	myCam->logHeaders(fout);
	fout << endl;
}

void RA14Robot::logging()
{
	myCam->log(fout);
	fout << endl;
}

};

START_ROBOT_CLASS(RA14Robot);
