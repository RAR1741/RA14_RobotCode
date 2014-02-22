#include "WPIlib.h"
//#include "Config.h"
//#include "MotionProfile.h"
#include <fstream>
//#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <iomanip>
#include "RAUtils.h"


class CamMotors : public PIDOutput {
public:
	CamMotors(Talon * left, Talon * right) {
		l = left;
		r = right;
	}
	
	virtual ~CamMotors() {
		/*delete l;
		delete r; */
	}
	void PIDWrite(float out)
	{
		l->Set(out);
		r->Set(-out);
	}
private:
	Talon * l, *r;
	
};

class CamShooter
{
public:
	CamShooter(int motorLeft, int motorRight, int encoderA, int encoderB, int indexInput);
	~CamShooter();
	
	//Functions
	void PIDEnable(void);
	void Process(bool fire, bool rearm);
	double GetPosition();
	void SetPosition(float pos);
	void logHeaders(ostream &f);
	void log(std::ostream &f);
	void Debug(std::ostream &f);
	
	void Reset();
private:
	CamMotors * cam_outputter;
	bool IndexTripped() { return !!!IndexSensor->Get(); } 
	Talon * ShooterMotorLeft;
	Talon * ShooterMotorRight;
	Encoder * ShooterEncoder;
	DigitalInput * IndexSensor;
	PIDController * PID;
	PIDController * PIDRight;
	bool IndexSeenLastSample;  // Was the index pulse seen during the last sample
	//MotionProfile * CamProfile;
	
	typedef enum {
		ReadyToFire = 1,		// Low-energy state where we can fire quickly
		Firing      = 2,        // Releasing cam
		Rearming    = 3,        // Rearming
		Calibration  = 4,
		Testing     = 5,        // test mode. Doesn't try and do anything else
		Homing      = 6,        // Either just fired, or on startup. Wait for index pulse.
	} CamShooterState;
	int m_state;
	
	bool IndexHasBeenReset;
	bool FireButtonLast;
	static const char * StateNumberToString(int state);
							
};
