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
#include "RAUtils.h"

//#define DISABLE_SHOOTER
//#define DISABLE_AUTONOMOUS
//TODO: Implement stub version of shooter class

using namespace std;

class RA14Robot: public IterativeRobot {
private:
	ofstream fout;
#ifndef DISABLE_SHOOTER
	CamShooter * myCam;
#endif //Ends DISABLE_SHOOTER
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
	Gamepad::DPadDirection OperatorDPad;

	DigitalOutput * CurrentSensorReset;

	bool ResetSetting;

	bool RingLightButtonRecentlyPressed;
	bool alreadyInitialized;

	int auto_case;
	int auto_state;

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
	Timer * auto_timer;

public:
	RA14Robot() {
		//myCurrentSensor = NULL;
#ifndef DISABLE_SHOOTER
		myCam = NULL;
#endif //Ends DISABLE_SHOOTER
		myDrive = NULL;
		DriverGamepad = NULL;
		OperatorGamepad = NULL;
		myCompressor = NULL;
		myCamera = NULL;
		myCollection = NULL;
		DriverLeftY = 0.0;
		DriverRightY = 0.0;
		auto_case = 0;
		auto_state = 0;
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
		OperatorDPad = Gamepad::kCenter;
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
		auto_timer = NULL;
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
#endif //Ends DISABLE_SHOOTER
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
		auto_timer = new Timer();
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
#endif //Ends DISABLE_SHOOTER

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
		Config::LoadFromFile("config.txt");
		auto_case = (int) Config::GetSetting("auto_case", 1);
		alreadyInitialized = true;
		auto_timer->Reset();
		auto_timer->Start();
		missionTimer->Start();
		myDrive->ResetOdometer();
		myCamera->Set(Relay::kForward);
		myCollection->ExtendArm();
		cout<<"Reseting Gyro"<<endl;
		gyro->Reset();
		//myOdometer->Reset();
		//myDrive->ShiftUp();
		myDrive->ShiftDown();
		//shift to high gear
		if (!fout.is_open()) {
			cout << "Opening logging.csv..." << endl;
			fout.open("logging.csv");
			logheaders();
		}
		auto_state = 0;
#ifndef DISABLE_SHOOTER
		myCam->Reset();
		myCam->Enable();
#endif //Ends DISABLE_SHOOTER
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
		float speed = Config::GetSetting("auto_speed", .5);
		cout<<"Auto Speed: "<<Config::GetSetting("auto_speed", 0)<<endl; // original .1
		float angle = gyro->GetAngle();
		float error = targetHeading - angle;
		float corrected = error * Config::GetSetting("auto_heading_p", .01);
		//float corrected = error * Config::GetSetting("auto_heading_p", .01);
		cout<<"Gyro angle: "<<angle<<endl;
		cout <<"Error: " << error << endl;
		//float lDrive = Config::GetSetting("auto_speed", -0.3) + (error * Config::GetSetting("auto_heading_p", .01));
		//float rDrive = Config::GetSetting("auto_speed", -0.3) - (error * Config::GetSetting("auto_heading_p", .01));
		// Reading p value from the config file does not appear to be working. When we get p from config, the math is not correct.
		float lDrive = Config::GetSetting("auto_speed", -0.3) + (error*0.01);
		float rDrive = Config::GetSetting("auto_speed", -0.3) - (error*0.01);
		cout << "Left: " << lDrive << endl;
		cout << "Right: " << rDrive << endl;
									
		
#ifndef DISABLE_AUTONOMOUS
		switch(auto_case)
		{
			case 0:
				// start master autonomous mode
				switch (auto_state) {
				case 0: // start
					auto_timer->Reset();
					auto_timer->Start();
					myCam->Process(false, false, false);
					break;
				case 1:
					myCam->Process(false, false, false);
					if (target->IsValid()) {
						auto_state = 2;
					} else if (auto_timer->Get() >= Config::GetSetting("auto_target_timeout", 1)) {
						auto_state = 10;
					}
					break;
				case 2:
					myCam->Process(false, false, false);
					if (target->IsHot()) {
						auto_state = 10;
					} else {
						if (auto_timer->Get() >= Config::GetSetting("auto_target_hot_timeout", 5)) {
							auto_state = 10;
						}
					}
					break;
				case 10:
					myCam->Process(true, false, false);
					if (myCam->IsReadyToRearm()) {
						auto_state = 11;
					}
					break;
				case 11:
					myCam->Process(false, false, false);
					myDrive->DriveArcade(corrected, speed);
					if (myDrive->GetOdometer() >= Config::GetSetting("auto_drive_distance", 100))
					{
						myDrive->Drive(0,0);
					}
					break;
				case 12:
					myDrive->Drive(0,0);
					break;
				default:
					cout << "Unknown state #" << auto_state << endl; 
					break;
				}
				// end master autonomous mode
				break;
			case 1:
				if( target->IsHot() && target->IsValid() )
				{
					cout << "Target is HOTTT taking the shot" << endl;
					//Drive forward and shoot right away
					//if( target->IsLeft() || target->IsRight() )
					//{
					if(myDrive->GetOdometer() <= 216 - Config::GetSetting("auto_firing_distance", 96)) //216 is distance from robot to goal
					{
						myDrive->Drive(corrected,speed);
					}
					else
					{
						myDrive->Drive(0,0);
						cout << "FIRING" << endl;
#ifndef DISABLE_SHOOTER
						myCam->Process(1,0,0);
#endif //Ends DISABLE_SHOOTER
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
						myDrive->Drive(corrected, speed);
					}
					else
					{
						// now at the firing spot.
						myDrive->Drive(0,0);
						if( target->IsHot() )
						{
							cout << "FIRING" << endl;
#ifndef DISABLE_SHOOTER
							myCam->Process(1,0,0);
#endif //Ends DISABLE_SHOOTER
						}
					}
				}
				else
				{
					//Not valid
					cout << "Not valid target." << endl;
				}
				break;
			
			case 2:
#ifndef DISABLE_SHOOTER
			myCam->Process(false,false,false);
			if(auto_timer->Get() >= 4.0)
			{
				myCam->Process(true,false,false);
			}
#endif //Ends DISABLE_SHOOTER
			
			/*if(myDrive->GetOdometer() <= 216 - Config::GetSetting("auto_firing_distance", 96)) //216 is distance from robot to goal
			{
				cout<<"Distance traveled: "<<myDrive->GetOdometer()<<" inches"<<endl;
				myDrive->Drive(corrected, speed);
			}
			else
			{
				myDrive->Drive(0,0);
				cout << "FIRING" << endl;
			}*/
			if(auto_timer->Get() < 5.0)
			{
				cout<<"Waiting....."<<endl;
			}
			else
			{
				cout<<"Distance traveled: "<<myDrive->GetOdometer()<<" inches"<<endl;
				myDrive->Drive(-.5, -.5);
			}
				
			break;
			
			case 3:
#ifndef DISABLE_SHOOTER
			switch(auto_state) {
				case 0:
					// Home/rearm
					// fire, rearm, eject
					myCam->Process(false, false, false);
					if (myCam->IsReadyToFire()) {
						auto_state = 1;
					}
					break;
				case 1:
					// fire
					myCam->Process(true, false, false);
					if (myCam->IsReadyToRearm()) {
						auto_state = 2;
					}
					break;
				case 2:
					// rearm
					myCam->Process(false, true, false);
					if (myCam->IsReadyToFire()) {
						auto_state = 3;
						auto_timer->Reset();
						auto_timer->Start();
					}
					break;
				case 3:
					myCam->Process(false, false, false);
					myCollection->SpinMotor(Config::GetSetting("intake_roller_speed", .7));
					if (auto_timer->HasPeriodPassed( Config::GetSetting("auto_collection_delay", 1.0) )) {
						auto_state = 4;
					}
					break;
				case 4:
					// fire again!
					myCam->Process(true, false, false);
					auto_state = 5;
					break;
				case 5:
					myCam->Process(false, false, false);
					
					if(myDrive->GetOdometer() <= 216 - Config::GetSetting("auto_firing_distance", 96)) //216 is distance from robot to goal
					{
						myDrive->Drive(corrected, speed);
					} else {
						// now at the firing spot.
						auto_state = 6;
						myDrive->Drive(0,0);
					}
					break;
				case 6:
					myCollection->RetractArm();
					break;
				default:
					cout << "Error, unrecognized state " << auto_state << endl;
			}
#endif //Ends DISABLE_SHOOTER
			
			break;
			
			case 4:	/* One ball Autonomous - Drive forward specific distance, stop then shoot.*/
#ifndef DISABLE_SHOOTER
		
					switch(auto_state) {
		
						case 0:		//Reset odometer, lower the arm and set launcher to ready to fire
							myDrive->ShiftUp();
							myDrive->ResetOdometer();
							myCollection->ExtendArm();
							myCam->Process(false,true,false);
							auto_state = 1;
							break;
						case 1:		// Start driving forward
							// myDrive->Drive(-.5, -.5);
							//myDrive->Drive(lDrive, rDrive);
							myDrive->DriveArcade(corrected, speed);
							auto_state = 2;
							break;
						case 2:		// Continue driving until required distance
							if(myDrive->GetOdometer() >= Config::GetSetting("auto_drive_distance", 96)) {
								myDrive->Drive(0, 0);
								auto_state = 3;
							}
							break;
						case 3:		// Fire launcher
							myCam->Process(true,false,false);
							auto_state = 4;
							break;
						case 4:		// Idle state
							break;
					}
			
#endif //Ends DISABLE_SHOOTER
			
			break;
			
			case 5:	// Two Ball Autonomous - Drag ball 2 while driving forward specific distance, stop then shoot, load shoot next ball
#ifndef DISABLE_SHOOTER
				// By Hugh Meyer - April 1, 2014
				
					switch(auto_state) {
					cout << "Executing mr m's auton" << endl;
				
						case 0:		// Set low gear, reset odometer, extend pickup arm, set launcher ready to fire, set wait timer
							//myDrive->ShiftUp();				// Shift to low gear
							myDrive->ShiftDown();
							myDrive->ResetOdometer();			// Reset odometer to zero
							myCollection->ExtendArm();			// Extend arm to pickup position
							myCam->Process(false,true,false);	// Set launcher to ready to fire position
							auto_timer->Reset();				// Set timer to zero
							auto_timer->Start();				// Start the timer for a short delay while pickup extends
							auto_state = 1;						// Go on  to next state
							break;
																			
						case 1:		// Wait for timer to expire - Let arm get extended and stabalized
							if (auto_timer->HasPeriodPassed( Config::GetSetting("auton5_extend_delay", 1.0) )) {
								auto_state = 2;
							}
							break;
						case 21: 	//reset gyro and reset timer
							gyro->Reset();
							auto_timer->Reset();
							auto_timer->Start();
							auto_state = 22;
							break;
						case 22:
							if(auto_timer->HasPeriodPassed(Config::GetSetting("auton5_gyro_reset_delay", 2) )){
								auto_state = 2;
							}	
							break;								
						case 2:		// Activate pickup roller motor to drag speed, start driving forward
							myCollection->SpinMotor(Config::GetSetting("auton5_drag_speed", 0.3));	// Start motor to drag ball 2
							//myDrive->DriveArcade(corrected, speed);		// Drive straight
							myDrive->DriveArcade(0.0, speed);
							auto_state = 3;
							break;
						case 3:		// Continue driving until required distance, stop driving, stop pickup roller motor
							if(myDrive->GetOdometer() >= Config::GetSetting("auton5_drive_distance", 96.0))
							{
								myCollection->SpinMotor(0);		// Stop collector pickup motor
								myDrive->Drive(0, 0);			// Stop driving
								auto_state = 31;				// On to next state
							}
							break;
						case 31: // slightly un-eject herded ball to avoid contact with launch ball
							myCollection->SpinMotor(Config::GetSetting("auton5_eject_speed", 0.3));
							auto_timer->Reset();
							auto_timer->Start(); // setup timer to time un-eject
							auto_state = 32;
							break;
						case 32: // once timer has run out, stop ejection and fire
							if (auto_timer->HasPeriodPassed(Config::GetSetting("auton5_uneject_time", 0.25))) {
								myCollection->SpinMotor(0); // stop spinner
								auto_state = 4;
							}
							break;
						case 4:		// Fire launcher to shoot first ball, set wait timer
							myCam->Process(true,false,false);	// Fire ball # 1
							auto_timer->Reset();				// Set timer to zero
							auto_timer->Start();				// Start the timer for a short delay while ball launches
							auto_state = 5;						// On to next state
							break;
						case 5:		// Wait for timer to expire after ball one launches
							if (auto_timer->HasPeriodPassed( Config::GetSetting("auton5_ball_1_launch_delay", 1.0) )) {
								auto_state = 6;					// On to next state
							}
							break;
						case 6:		// set launcher ready to fire for ball 2, set wait timer
							myCam->Process(false,true,false);	// Set launcher to ready to fire position
							auto_timer->Reset();				// Set timer to zero
							auto_timer->Start();				// Start the timer for a short delay while ball launches
							auto_state = 7;						// On to next state
							break;
						case 7:		// Wait for timer to expire
							if (auto_timer->HasPeriodPassed( Config::GetSetting("auton5_ball_2_ready2fire_delay", 1.0) )) {
								auto_state = 8;					// Wait for launcher to get ready to accept ball 2
							}
							break;
						case 8:		// Activate pickup roller motor to load ball 2, set wait timer
							myCollection->SpinMotor(Config::GetSetting("auton5_intake_roller_speed", 0.7));
							auto_timer->Reset();				// Set timer to zero
							auto_timer->Start();				// Start the timer for a short delay while ball 2 loads
							auto_state = 9;						// On to next state
							break;
						case 9:		// Wait for timer to expire while ball 2 gets collected into launcher
							if (auto_timer->HasPeriodPassed( Config::GetSetting("auton5_ball_2_settle_delay", 1.0) )) {
								auto_state = 10;				// Wait for ball 2 to be collected and settle
							}
							break;

						case 19:	// Drive backwards a little bit, set wait timer
							myDrive->DriveArcade(-1*corrected, -1*speed);		// Drive straight
							auto_timer->Reset();				// Set timer to zero
							auto_timer->Start();				// Start the timer for a short delay while ball launches
							auto_state = 20;				// Wait for ball 2 to be collected and settle
							break;
		
							
						case 20:		// Wait for timer to expire while ball 2 gets collected into launcher
							if (myDrive->GetOdometer() <= 
									  Config::GetSetting("auton5_drive_distance", 96) 
									- Config::GetSetting("auton5_backup_distance", 6))	{
								myDrive->Drive(0,0);
								auto_state = 10;
							}
							break;
						case 10:	// Fire launcher to shoot second ball and stop roller
							myCam->Process(true,false,false);	// Fire ball # 2
							myCollection->SpinMotor(0);			// Stop spinning the roller
							auto_state = 11;					// All done so go to idle state
							break;
						case 11:	// Idle state
							auto_state = 11;
							break;
/*						case 12:	// More states if we need them for changes.
							auto_state = 13;
							break;
						case 13:	// 
							auto_state = 14;
							break;
						case 14:	// 
							auto_state = 15;
							break;
						case 15:	// 
							auto_state = 15;
							break;
*/
					}
					
#endif //Ends DISABLE_SHOOTER
					
			break;
				
			
			case 6:	// Two Ball Autonomous - Drive forward specific distance, stop then shoot, backup get second ball, drive, shoot
#ifndef DISABLE_SHOOTER
				// By Hugh Meyer - April 1, 2014
					
					switch(auto_state) {
						
						case 0:		// Set low gear, reset odometer, extend pickup arm, set launcher ready to fire
							myDrive->ShiftDown();				// Shift to low gear
							myDrive->ResetOdometer();			// Reset odometer to zero
							myCollection->ExtendArm();			// Extend arm to pickup position
							myCam->Process(false,true,false);	// Set launcher to ready to fire position
							auto_state = 1;						// Go on to next state
							break;
						case 1:		// Drive forward
							myDrive->DriveArcade(corrected, speed);
							auto_state = 2;						// On to next state
							break;
						case 2:		// Continue driving forward until the specific distance is traveled
							if(myDrive->GetOdometer() >= Config::GetSetting("auton6_drive_forward_distance", 96.0)) {
								myDrive->Drive(0, 0);
								auto_state = 3;
							}
							break;
						case 3:		// Fire launcher to launch first ball, set timer
							myCam->Process(true,false,false);	// Launch ball # 1
							auto_timer->Reset();				// Set timer to zero
							auto_timer->Start();				// Start the timer for a short delay while ball launches
							auto_state = 4;						// On to next state
							break;	
						case 4:		// Wait for timer while ball launcher fires
							if (auto_timer->HasPeriodPassed( Config::GetSetting("auton6_ball_1_fire_delay", 1.0) )) {
								auto_state = 5;					// On to next state
							}
							break;
						case 5:		// Reset Odometer, Drive backwards, set launcher to ready to fire position, turn on pickup
							cout<<"Corrected Values: "<<corrected<<endl; 
							cout<<"Speed: "<< -1 * speed<<endl;
							myDrive->ResetOdometer();				// Reset odometer to zero
							myDrive->DriveArcade(-1* corrected, -1 * speed);	// Drive backwards
							myCam->Process(false,true,false);		// Set launcher to ready to fire position
							myCollection->SpinMotor(-1 * Config::GetSetting("auton6_intake_roller_speed", 0.7));	// Turn on pickup
							auto_state = 6;						// On to next state
							break;
						case 6:		// Continue driving backwards a specific distance
							if(myDrive->GetOdometer() <= -1 * Config::GetSetting("auton6_drive_forward_distance", -96.0)) {
								auto_state = 7;					// On to next state
							}
							break;
						case 7:		// Set timer
							auto_timer->Reset();				// Set timer to zero
							auto_timer->Start();				// Start the timer for a short delay while ball launches
							auto_state = 8;						// On to next state
							break;
						case 8:		// Wait for timer while ball loads and settles
							if (auto_timer->HasPeriodPassed( Config::GetSetting("auton6_ball_2_load_delay", 1.0) )) {
								auto_state = 9;					// On to next state
								myDrive->ResetOdometer();
							}
							break;
						case 9:		// Drive forwards
							myDrive->DriveArcade(/*corrected*/ 0.01, speed);
							auto_state = 10;					// On to next state
							break;
						case 10:	// Continue driving forward until the specific distance is traveled
							cout << "I am driving forward: " << myDrive->GetOdometer() << endl;
							myDrive->DriveArcade(0.01, speed);
							if(myDrive->GetOdometer() >= Config::GetSetting("auton6_drive_forward_distance", 96.0)) {
								myDrive->Drive(0, 0);
								auto_state = 11;				// On to next state
							}
							break;
						case 11:	// Fire launcher to launch second ball
							myCam->Process(true,false,false);	// Launch ball # 2
							auto_state = 15;					// On to next state
							break;
/*						case 12:	// 
							auto_state = 13;					// On to next state
							break;
						case 13:	// 
							auto_state = 14;					// On to next state
							break;
						case 14:	// 
							auto_state = 15;					// All done so go to idle state
							break;
*/
						case 15:	// Idle state
							auto_state = 15;
							break;

					}
					cout << myDrive->GetOdometer() << endl;
							
#endif //Ends DISABLE_SHOOTER
							
			break;
			case 7:	// Two Ball Autonomous - Drag ball 2 while driving forward specific distance, stop then shoot, load shoot next ball
			#ifndef DISABLE_SHOOTER
							// By Hugh Meyer - April 1, 2014
							
								switch(auto_state) {
								cout << "Executing mr m's auton" << endl;
							
									case 0:		// Set low gear, reset odometer, extend pickup arm, set launcher ready to fire, set wait timer
										myDrive->ShiftUp();				// Shift to high gear
										//myDrive->ShiftDown();
										myDrive->ResetOdometer();			// Reset odometer to zero
										myCollection->ExtendArm();			// Extend arm to pickup position
										myCam->Process(false,true,false);	// Set launcher to ready to fire position
										auto_timer->Reset();				// Set timer to zero
										auto_timer->Start();				// Start the timer for a short delay while pickup extends
										auto_state = 1;						// Go on  to next state
										break;
																						
									case 1:		// Wait for timer to expire - Let arm get extended and stabalized
										if (auto_timer->HasPeriodPassed( Config::GetSetting("auton7_extend_delay", 1.0) )) {
											auto_state = 2;
										}
										break;
									case 21: 	//reset gyro and reset timer
										gyro->Reset();
										auto_timer->Reset();
										auto_timer->Start();
										auto_state = 22;
										break;
									case 22:
										if(auto_timer->HasPeriodPassed(Config::GetSetting("auton7_gyro_reset_delay", 2) )){
											auto_state = 2;
										}	
										break;								
									case 2:		// Activate pickup roller motor to drag speed, start driving forward
										myCollection->SpinMotor(Config::GetSetting("auton7_drag_speed", 0.3));	// Start motor to drag ball 2
										//myDrive->DriveArcade(corrected, speed);		// Drive straight
										myDrive->DriveArcade(0.05, speed);
										auto_state = 3;
										break;
									case 3:		// Continue driving until required distance, stop driving, stop pickup roller motor
										if(myDrive->GetOdometer() >= Config::GetSetting("auton7_drive_distance", 96.0))
										{
											myCollection->SpinMotor(0);		// Stop collector pickup motor
											myDrive->Drive(0, 0);			// Stop driving
											auto_state = 31;				// On to next state
										}
										break;
									case 31: // slightly un-eject herded ball to avoid contact with launch ball
										myCollection->SpinMotor(Config::GetSetting("auton7_eject_speed", 0.3));
										auto_timer->Reset();
										auto_timer->Start(); // setup timer to time un-eject
										auto_state = 32;
										break;
									case 32: // once timer has run out, stop ejection and fire
										if (auto_timer->HasPeriodPassed(Config::GetSetting("auton7_uneject_time", 0.25))) {
											myCollection->SpinMotor(0); // stop spinner
											auto_state = 4;
										}
										break;
									case 4:		// Fire launcher to shoot first ball, set wait timer
										myCam->Process(true,false,false);	// Fire ball # 1
										auto_timer->Reset();				// Set timer to zero
										auto_timer->Start();				// Start the timer for a short delay while ball launches
										auto_state = 5;						// On to next state
										break;
									case 5:		// Wait for timer to expire after ball one launches
										if (auto_timer->HasPeriodPassed( Config::GetSetting("auton7_ball_1_launch_delay", 1.0) )) {
											auto_state = 6;					// On to next state
										}
										break;
									case 6:		// set launcher ready to fire for ball 2, set wait timer
										myCam->Process(false,true,false);	// Set launcher to ready to fire position
										auto_timer->Reset();				// Set timer to zero
										auto_timer->Start();				// Start the timer for a short delay while ball launches
										auto_state = 7;						// On to next state
										break;
									case 7:		// Wait for timer to expire
										if (auto_timer->HasPeriodPassed( Config::GetSetting("auton7_ball_2_ready2fire_delay", 1.0) )) {
											auto_state = 8;					// Wait for launcher to get ready to accept ball 2
										}
										break;
									case 8:		// Activate pickup roller motor to load ball 2, set wait timer
										myCollection->SpinMotor(Config::GetSetting("auton7_intake_roller_speed", 0.7));
										auto_timer->Reset();				// Set timer to zero
										auto_timer->Start();				// Start the timer for a short delay while ball 2 loads
										auto_state = 9;						// On to next state
										break;
									case 9:		// Wait for timer to expire while ball 2 gets collected into launcher
										if (auto_timer->HasPeriodPassed( Config::GetSetting("auton7_ball_2_settle_delay", 1.0) )) {
											auto_state = 10;				// Wait for ball 2 to be collected and settle
										}
										break;

									case 19:	// Drive backwards a little bit, set wait timer
										myDrive->DriveArcade(-1*corrected, -1*speed);		// Drive straight
										auto_timer->Reset();				// Set timer to zero
										auto_timer->Start();				// Start the timer for a short delay while ball launches
										auto_state = 20;				// Wait for ball 2 to be collected and settle
										break;
					
										
									case 20:		// Wait for timer to expire while ball 2 gets collected into launcher
										if (myDrive->GetOdometer() <= 
												  Config::GetSetting("auton7_drive_distance", 96) 
												- Config::GetSetting("auton7_backup_distance", 6))	{
											myDrive->Drive(0,0);
											auto_state = 10;
										}
										break;
									case 10:	// Fire launcher to shoot second ball and stop roller
										myCam->Process(true,false,false);	// Fire ball # 2
										myCollection->SpinMotor(0);			// Stop spinning the roller
										auto_state = 11;					// All done so go to idle state
										break;
									case 11:	// Idle state
										auto_state = 11;
										break;
			/*						case 12:	// More states if we need them for changes.
										auto_state = 13;
										break;
									case 13:	// 
										auto_state = 14;
										break;
									case 14:	// 
										auto_state = 15;
										break;
									case 15:	// 
										auto_state = 15;
										break;
			*/
								}
								
			#endif //Ends DISABLE_SHOOTER
								
						break;
/*			
			
			case 7:	// Extra structure for more changes
#ifndef DISABLE_SHOOTER
				// By Hugh Meyer - April 1, 2014
					
					switch(auto_state) {
						
						case 0:		// Set low gear, reset odometer, extend pickup arm, set launcher ready to fire, drive foward
							myDrive->ShiftDown();				// Shift to low gear
							myDrive->ResetOdometer();			// Reset odometer to zero
							myCollection->ExtendArm();			// Extend arm to pickup position
							myCam->Process(false,true,false);	// Set launcher to ready to fire position
							myDrive->Drive(lDrive, rDrive);		// Drive straight
							auto_state = 1;						// Go on  to next state
							break;
						case 1:		// 
							auto_state = 2;						// On to next state
							break;
						case 2:		// 
							auto_state = 3;						// On to next state
							break;
						case 3:		// 
							auto_state = 4;						// On to next state
							break;	
						case 4:		// 
							auto_state = 5;						// On to next state
							break;
						case 5:		// 
							auto_state = 6;						// On to next state
							break;
						case 6:		// 
							auto_state = 7;						// On to next state
							break;
						case 7:		// 
							auto_state = 8;						// On to next state
							break;
						case 8:		// 
							auto_state = 9;						// On to next state
							break;
						case 9:		// 
							auto_state = 10;					// On to next state
							break;
						case 10:	// 
							auto_state = 11;					// On to next state
							break;
						case 11:	// 
							auto_state = 12;					// On to next state
							break;
						case 12:	// 
							auto_state = 13;					// On to next state
							break;
						case 13:	// 
							auto_state = 14;					// On to next state
							break;
						case 14:	// 
							auto_state = 15;					// All done so go to idle state
							break;
						case 15:	// Idle state
							auto_state = 15;
							break;
					}
								
#endif //Ends DISABLE_SHOOTER
									
			break;
*/								
			
			default:
			cout<<"Error in autonomous, unrecognized case: "<<auto_case<<endl;
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
#endif //Ends DISABLE_AUTONOMOUS

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
		myDrive->ShiftUp();
		//myOdometer->Reset();
		if (!fout.is_open()) {
			cout << "Opening logging.csv..." << endl;
			fout.open("logging.csv");
			logheaders();
		}

#ifndef DISABLE_SHOOTER
		myCam->Reset();
		myCam->Enable();
#endif //Ends DISABLE_SHOOTER
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
		RingLightButton = OperatorGamepad->GetY();
		ShouldFireButton = DriverGamepad->GetRightTrigger();
		BallCollectPickupButton = DriverGamepad->GetBack();
		DriverDPad = DriverGamepad->GetDPad();
		OperatorDPad = OperatorGamepad->GetDPad();
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

		switch (OperatorDPad) {
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
			armPosition = 0;
			rollerPosition = -1;
		}
		
		if(DriverGamepad->GetB())
		{
			armPosition = -1;
			rollerPosition = 1;
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
		myCam->Process(ShouldFireButton, (OperatorGamepad->GetX() || DriverGamepad->GetX() ), DriverGamepad->GetB());
		//myCam->Debug(cout);
#endif //Ends DISABLE_SHOOTER
		
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
		if(DriverDPad == Gamepad::kUp)
		{
			cout << "Forward" << endl;
			myDrive->reverseDirectionForward();
		}
		else if(DriverDPad == Gamepad::kDown)
		{
			cout << "Backward"<<endl;
			myDrive->reverseDirectionReverse();
		}
		
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
		myCam->Disable();
		myCompressor->Start();
		Config::LoadFromFile("config.txt");
		Config::Dump();
		
		myCamera->Set(Relay::kForward); // turn on light
	}

	/**
	 * Periodic code for test mode should go here.
	 *
	 * Use this method for code which will be called periodically at a regular
	 * rate while the robot is in test mode.
	 */
	void RA14Robot::TestPeriodic() {
		StartOfCycleMaintenance();
		
		std::string packet = server->GetLatestPacket();
		
		target->Parse( packet );
		
		if (target->IsValid()) {
			cout << "target: " << round_to(target->GetX(),2) << "," << round_to(target->GetY(), 2);
			cout << " distance " << round_to(target->GetDistance(), 2) << " ft";
			
			if (target->IsHot()) {
				cout << " HOT ";
			}
			if (target->IsLeft()) {
				cout << " LEFT ";
			}
			if (target->IsRight()) {
				cout << " RIGHT ";
			}
		} else {
			cout << "No target, latest is " << packet;
		}
		
		cout << endl;
		EndOfCycleMaintenance();
	}

	void RA14Robot::logheaders() {
		fout << "MissionTimer,";
#ifndef DISABLE_SHOOTER
		myCam->logHeaders(fout);
#endif //Ends DISABLE_SHOOTER
		myDrive->logHeaders(fout);
		fout << "CAMLeftCurrent,CAMRightCurrent,DriveLeftCurrent,DriveRightCurrent,AutoCase,GyroHeading,DropSensor,BatteryVoltage,";
		fout << "TargetValid,TargetHot,TargetDistance,TargetX,TargetY,TargetIsLeft,TargetIsRight,MatchTime,AutoInternalState,";
		fout << endl;
	}

	void RA14Robot::logging() {
		if (fout.is_open()) {
		fout << missionTimer->Get() << ",";
#ifndef DISABLE_SHOOTER
		myCam->log(fout);
#endif //Ends DISABLE_SHOOTER
		myDrive->log(fout);
		CurrentSensorSlot * slots[4] = { camMotor1Slot, camMotor2Slot,
				driveLeftSlot, driveRightSlot };
		
		DriverStation * ds = DriverStation::GetInstance();

		for (int i = 0; i < 4; ++i) {
			fout << slots[i]->Get() << ",";
		}

		fout << auto_case << "," << gyro->GetAngle() << "," << dropSensor->GetPosition() << "," << ds->GetBatteryVoltage() << ",";
		fout << target->IsValid() << "," << target->IsHot() << "," <<  target->GetDistance() << "," << target->GetX() << ",";
		fout << target->GetY() << "," << target->IsLeft() << "," << target->IsRight() << ",";
		fout << ds->GetMatchTime() << "," << auto_state << ",";
		fout << endl;
		}
	}

};

START_ROBOT_CLASS(RA14Robot)
;
