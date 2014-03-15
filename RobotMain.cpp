#include "WPILib.h"
#include "DriveTrain.h"
#include "CamShooter.h"
#include <cstdio>
#include <math.h>
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
#include "DropSensor.h"

//#define DISABLE_SHOOTER
//#define DISABLE_AUTONOMOUS

using namespace std;

class RA14Robot: public IterativeRobot {
private:
	ofstream fout;
#ifndef DISABLE_SHOOTER
	CamShooter * myCam;
#endif
	CANJaguar * myJag;
	DriveTrain * myDrive;
	Relay * myCamera;
	Gamepad * DriverGamepad;
	Gamepad * OperatorGamepad;
	Compressor * myCompressor;
	Collection * myCollection;
	//Odometer * myOdometer;
	float DriverLeftY;
	float DriverRightY;
	bool DriverLeftBumper;
	bool DriverRightBumper;
	bool ShouldFireButton;
	bool RingLightButton;
	bool BallCollectPickupButton;
	Gamepad::DPadDirection DriverDPad;

	DigitalOutput * CurrentSensorReset;

	bool ResetSetting;

	bool RingLightButtonRecentlyPressed;
	bool alreadyInitialized;

	int auto_case;
	int auto_timer;

	TargetServer * server;
	Target * target;
	CurrentSensorSlot * camMotor1Slot;
	CurrentSensorSlot * camMotor2Slot;
	CurrentSensorSlot * driveLeftSlot;
	CurrentSensorSlot * driveRightSlot;
	Timer * resetCurrentSensorTimer;

	DigitalInput * currentSensor1Reset;

	DigitalOutput * signalOutCycle;
	DigitalOutput * signalOutToggle;

	Timer * missionTimer;

	Gyro * gyro;
	float targetHeading;
	DropSensor * dropSensor;

public:
	RA14Robot() {
		//myCurrentSensor = NULL;
#ifndef DISABLE_SHOOTER
		myCam = NULL;
#endif
		myDrive = NULL;
		DriverGamepad = NULL;
		OperatorGamepad = NULL;
		myCompressor = NULL;
		myCamera = NULL;
		myCollection = NULL;
		DriverLeftY = 0.0;
		DriverRightY = 0.0;
		auto_case = 0;
		DriverLeftBumper = true;
		DriverRightBumper = true;
		CurrentSensorReset = NULL;
		ResetSetting = false;
		RingLightButtonRecentlyPressed = false;
		RingLightButton = false;
		alreadyInitialized = false;
		ShouldFireButton = false;
		BallCollectPickupButton = false;
		DriverDPad = Gamepad::kCenter;
		//myOdometer = NULL;

		server = NULL;
		target = NULL;

		camMotor1Slot = NULL;
		camMotor2Slot = NULL;
		driveLeftSlot = NULL;
		driveRightSlot = NULL;

		currentSensor1Reset = NULL;
		resetCurrentSensorTimer = NULL;

		signalOutCycle = NULL;
		signalOutToggle = NULL;

		gyro = NULL;
		targetHeading = 0;
		
		dropSensor = NULL;
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
		myCompressor = new Compressor(11, 1);
		cout << "Compressor initialized." << endl;

		cout << "Initializing cam shooter..." << endl;
#ifndef DISABLE_SHOOTER
		myCam = new CamShooter(5, 6, 2, 1, 3, 13, 14, Config::GetSetting("cam_loop_period", .004));
#else
		cout << "Cam shooter DISABLED!" << endl;
#endif
		cout << "Cam shooter initialized." << endl;

		cout << "Initializing drivetrain..." << endl;
		myDrive = new DriveTrain(1, 2, 3, 4, 1, 2, 4, 5, 6, 7);
		cout << "Drivetrain initialized." << endl;

		cout << "Initializing current sensor.." << endl;
		//myCurrentSensor = new CurrentSensor(7);
		resetCurrentSensorTimer = new Timer();
		resetCurrentSensorTimer->Start();

		camMotor1Slot = new CurrentSensorSlot(3);
		camMotor2Slot = new CurrentSensorSlot(4);
		driveLeftSlot = new CurrentSensorSlot(5);
		driveRightSlot = new CurrentSensorSlot(6);

		cout << "Current sensor initialized." << endl;

		cout << "Initializing collection system" << endl;
		myCollection = new Collection(7, 3, 4);
		cout << "Collection system initialized" << endl;

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

		cout << "Setting up current sensor" << endl;
		//camMotor1Slot = new CurrentSensorSlot()
		cout << "Current sensor set up" << endl;

		cout << "Setting up Odometer" << endl;
		//myOdometer = new Odometer(4, 5);
		auto_case = (int) Config::GetSetting("auto_case", 1);
		cout << "Setting up Gyro, please do NOT move the robot..." << endl;
		gyro = new Gyro(2);
		cout << "Gyro initialized." << endl;
		
		cout << "Initializing drop sensor" << endl;
		dropSensor = new DropSensor(7);
		cout << "Drop sensor initialized." << endl;

		this->SetPeriod(Config::GetSetting("robot_loop_period", 0.05));
		cout << "Period set to " << this->GetLoopsPerSec() << "Hz" << endl;

		cout << "2014 Red Alert Robot" << endl;
		cout << "Compiled on: ";
		cout << __DATE__ << " at " << __TIME__ << endl;

		cout << "Mission Timer starting:" << endl;
		missionTimer = new Timer();
		cout << "Mission timer started: " << missionTimer->Get() << "s" << endl;

		cout << "Signaling system starting..." << endl;
		//signalOutToggle = new DigitalOutput(13);
		//signalOutCycle = new DigitalOutput(14);
		cout << "Signal system started." << endl;
		cout << "Robot Init Complete..." << endl;

		Config::Dump();
	}

	void RA14Robot::StartOfCycleMaintenance() {
		static bool toggleOut = false;
		//signalOutCycle->Set(1);
		toggleOut = !toggleOut;
		//signalOutToggle->Set( toggleOut );
		//CurrentSensorSlot * slots[4] = { camMotor1Slot, camMotor2Slot,
		//		driveLeftSlot, driveRightSlot };

		//for (int i = 0; i < 4; ++i) {
		//	slots[i]->Process();
		//}

	}

	void RA14Robot::EndOfCycleMaintenance() {
		//CurrentSensorReset->Set(1);
		//	ResetSetting = ! ResetSetting;
		//	CurrentSensorReset->Set(ResetSetting);
		if (resetCurrentSensorTimer->Get() > Config::GetSetting(
				"curent_sensor_reset_time", .1)) {
			CurrentSensorReset->Set(1);
			resetCurrentSensorTimer->Reset();
			CurrentSensorReset->Set(0);
		}
		
		logging();
		target->Parse("");
		//signalOutCycle->Set(0);
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
			missionTimer->Stop();
			Config::Dump();
		}

#ifndef DISABLE_SHOOTER 
		myCam->Reset();
#endif

		/*
		 if(fout.is_open()) {
		 cout << "Closing logging.csv..." << endl;
		 fout.close();
		 }
		 */
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
		missionTimer->Start();
		myDrive->ResetOdometer();
		myCamera->Set(Relay::kForward);
		myCollection->ExtendArm();
		gyro->Reset();
		//myOdometer->Reset();
		myDrive->ShiftDown();
		if (!fout.is_open()) {
			cout << "Opening logging.csv..." << endl;
			fout.open("logging.csv");
			logheaders();
		}
#ifndef DISABLE_SHOOTER
		myCam->Reset();
		myCam->Enable();
#endif
	}

	/**
	 * Periodic code for autonomous mode should go here.
	 *
	 * Use this method for code which will be called periodically at a regular
	 * rate while the robot is in autonomous mode.
	 */
	void RA14Robot::AutonomousPeriodic() {
		StartOfCycleMaintenance();

		target->Parse(server->GetLatestPacket());
		float speed = Config::GetSetting("auto_speed", .1);
		float angle = gyro->GetAngle();
		
		float error = targetHeading - angle;
		float corrected = error * -1 * Config::GetSetting("auto_heading_p", .01);
		cout<<"Gyro angle: "<<angle<<endl;
		cout <<"Corrected: " << corrected << endl;
		if(myDrive->GetOdometer() <= 216 - Config::GetSetting("auto_firing_distance", 96)) //216 is distance from robot to goal
		{
			cout<<"Distance traveled: "<<myDrive->GetOdometer()<<" inches"<<endl;
			myDrive->Drive(corrected, speed);
			//myDrive->Drive(speed,speed);
		}
		else
		{
			myDrive->Drive(0,0);
			cout << "FIRING" << endl;
			myCam->Process(1,0);
		}
		
#ifndef DISABLE_AUTONOMOUS
		switch(auto_case)
		{
			case 1:
			if( target->IsHot() && target->IsValid() )
			{
				cout << "Target is HOTTT taking the shot" << endl;
				//Drive forward and shoot right away
				//if( target->IsLeft() || target->IsRight() )
				//{
					if(myDrive->GetOdometer() <= 216 - Config::GetSetting("auto_firing_distance", 96)) //216 is distance from robot to goal
					{
						myDrive->Drive(speed,speed);
					}
					else
					{
						myDrive->Drive(0,0);
						cout << "FIRING" << endl;
#ifndef DISABLE_SHOOTER
						myCam->Process(1,0);
#endif
					}
				/*}
				else
				{
					cout<<"Error"<<endl;
				}*/
			}
			else if(target->IsValid())
			{
				cout << "Target is valid, but cold. Driving and waiting" << endl;
				//Drive forward and wait to shoot 
				if(myDrive->GetOdometer() <= 216 - Config::GetSetting("auto_firing_distance", 96)) //216 is distance from robot to goal
				{
					myDrive->Drive(speed, speed);
				}
				else
				{
					// now at the firing spot.
					myDrive->Drive(0,0);
					if( target->IsHot() )
					{
						cout << "FIRING" << endl;
#ifndef DISABLE_SHOOTER
						myCam->Process(1,0);
#endif
					}
				}
			}
			else
			{
				//Not valid
				cout << "Not valid target." << endl;
			}
			break;
			default:
			cout<<auto_case<<endl;
			cout<<"Error in autonomous"<<endl;
		}
//#else
//		if (myDrive->GetOdometer() <= (4 * acos(-1) ) ) //216 is distance from robot to goal
//		{
//			float speed = Config::GetSetting("auto_speed", .3);
//			cout << myDrive->GetOdometer() << endl;
//			myDrive->Drive(speed, speed);
//		} else {
//			cout << "Finished driving";
//			myDrive->Drive(0, 0);
//		}
#endif

		EndOfCycleMaintenance();
	}

	/**
	 * Initialization code for teleop mode should go here.
	 * 
	 * Use this method for initialization code which will be called each time
	 * the robot enters teleop mode.
	 */
	void RA14Robot::TeleopInit() {
		Config::LoadFromFile("config.txt");
		myCompressor->Start();
		alreadyInitialized = true;
		missionTimer->Start();
		myDrive->ResetOdometer();
		//myOdometer->Reset();
		if (!fout.is_open()) {
			cout << "Opening logging.csv..." << endl;
			fout.open("logging.csv");
			logheaders();
		}

#ifndef DISABLE_SHOOTER
		myCam->Reset();
		myCam->Enable();
#endif
	}

	/**
	 * Periodic code for teleop mode should go here.
	 *
	 * Use this method for code which will be called periodically at a regular
	 * rate while the robot is in teleop mode.
	 */
	void RA14Robot::TeleopPeriodic() {
		StartOfCycleMaintenance();

		//Input Acquisition
		DriverLeftY = DriverGamepad->GetLeftY();
		DriverRightY = DriverGamepad->GetRightY();
		DriverLeftBumper = DriverGamepad->GetLeftBumper(); // Reads state of left bumper 
		DriverRightBumper = DriverGamepad->GetRightBumper(); // Gets state of right bumper
		RingLightButton = DriverGamepad->GetY();
		ShouldFireButton = DriverGamepad->GetRightTrigger();
		BallCollectPickupButton = DriverGamepad->GetBack();
		DriverDPad = DriverGamepad->GetDPad();
		//End Input Acquisition


		//Current Sensing
		//myCurrentSensor->Toggle(DriverGamepad->GetStart());
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

		/*
		 if(BallCollectPickupButton)
		 {
		 myCollection->Collect();
		 }
		 else
		 {
		 myCollection->ResetPosition();
		 }
		 */

		float spinSpeed = Config::GetSetting("intake_roller_speed", 1);
		int armPosition = 0;
		int rollerPosition = 0;

		switch (DriverDPad) {
		case Gamepad::kCenter:
			armPosition = 0;
			rollerPosition = 0;
			break;
		case Gamepad::kUp:
			armPosition = -1;
			break;
		case Gamepad::kUpLeft:
			armPosition = -1;
			rollerPosition = -1;
			break;
		case Gamepad::kUpRight:
			armPosition = -1;
			rollerPosition = 1;
			break;
		case Gamepad::kDown:
			armPosition = 1;
			break;
		case Gamepad::kDownLeft:
			armPosition = 1;
			rollerPosition = -1;
			break;
		case Gamepad::kDownRight:
			armPosition = 1;
			rollerPosition = 1;
			break;
		case Gamepad::kLeft:
			rollerPosition = -1;
			break;
		case Gamepad::kRight:
			rollerPosition = 1;
			break;
		}

		if (DriverGamepad->GetLeftTrigger()) {
			rollerPosition = -1;
		}

		spinSpeed *= rollerPosition;
		myCollection->SpinMotor(spinSpeed);

		if (armPosition > 0)
			myCollection->ExtendArm();
		else if (armPosition < 0)
			myCollection->RetractArm();
		//End Ball Collection

		//Fire Control

#ifndef DISABLE_SHOOTER
		myCam->Process(ShouldFireButton, DriverGamepad->GetX(), DriverGamepad->GetB());
		myCam->Debug(cout);
#endif

		//End Fire Control


		//Ring Light
		if (RingLightButton) {
			RingLightButtonRecentlyPressed = true;
		}

		if (!RingLightButton && RingLightButtonRecentlyPressed) {
			if (myCamera->Get() == Relay::kOff)
				myCamera->Set(Relay::kForward);
			else
				myCamera->Set(Relay::kOff);

			RingLightButtonRecentlyPressed = false;
		}
		//End Ring Light


		//Drive Processing
		if (DriverLeftBumper) {
			myDrive->ShiftUp();
		} else if (DriverRightBumper) {
			myDrive->ShiftDown();
		}

		//myDrive->Drive(DriverLeftY, DriverRightY);
		myDrive->DriveArcade(DriverGamepad->GetRightX(), DriverGamepad->GetRightY());
		myDrive->Debug(cout);
		//End Drive Processing


		EndOfCycleMaintenance();
	}

	/**
	 * Initialization code for test mode should go here.
	 * 
	 * Use this method for initialization code which will be called each time
	 * the robot enters test mode.
	 */
	void RA14Robot::TestInit() {
		myCompressor->Start();
		Config::LoadFromFile("config.txt");
		Config::Dump();
	}

	/**
	 * Periodic code for test mode should go here.
	 *
	 * Use this method for code which will be called periodically at a regular
	 * rate while the robot is in test mode.
	 */
	void RA14Robot::TestPeriodic() {
		StartOfCycleMaintenance();

		EndOfCycleMaintenance();
	}

	void RA14Robot::logheaders() {
		fout << "MissionTimer,";
#ifndef DISABLE_SHOOTER
		myCam->logHeaders(fout);
#endif
		myDrive->logHeaders(fout);
		fout
				<< "CAMLeftCurrent,CAMRightCurrent,DriveLeftCurrent,DriveRightCurrent,AutoCase,GyroHeading,DropSensor";
		fout << endl;
	}

	void RA14Robot::logging() {
		if (fout.is_open()) {
		fout << missionTimer->Get() << ",";
#ifndef DISABLE_SHOOTER
		myCam->log(fout);
#endif
		myDrive->log(fout);
		CurrentSensorSlot * slots[4] = { camMotor1Slot, camMotor2Slot,
				driveLeftSlot, driveRightSlot };

		for (int i = 0; i < 4; ++i) {
			fout << slots[i]->Get() << ",";
		}

		fout << auto_case << "," << gyro->GetAngle() << "," << dropSensor->GetPosition() << ",";
		fout << endl;
		}
	}

};

START_ROBOT_CLASS(RA14Robot)
;
