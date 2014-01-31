#include "WPIlib.h"
//#include "Config.h"
#include "MotionProfile.h"
#include <fstream>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <iomanip>

class CamShooter
{
public:
	CamShooter(int motor, int encoderA, int encoderB, int indexInput);
	~CamShooter();
	
	//Functions
	void Process(bool fire);
	double GetPosition();
	void SetPosition(float pos);
	void logHeaders(ostream &f);
	void log(std::ostream &f);
	void Debug(std::ostream &f);
private:
	bool IndexTripped() { return !!!IndexSensor->Get(); } 
	void InitializeProfile();
	Talon * ShooterMotor;
	Encoder * ShooterEncoder;
	DigitalInput * IndexSensor;
	PIDController * PID;
	bool IndexSeenLastSample;  // Was the index pulse seen during the last sample
	MotionProfile * CamProfile;
	
	typedef enum {
		ReadyToFire = 1,		// Low-energy state where we can fire quickly
		Firing      = 2,        // Releasing cam
		Rearming    = 3,        // Rearming
		ExitFiring  = 4,
	} CamShooterState;
	int m_state;
	
	bool FireButtonLast;
	static const char * StateNumberToString(int state);
							
};
