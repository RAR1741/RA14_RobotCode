#include "WPIlib.h"
#include "semLib.h"
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

class CamShooter : Controller
{
public:
	CamShooter(int motorLeft, int motorRight, int encoderA, int encoderB, int indexInput, 
			int scopeTogglePort,
			int scopeCyclePort, float period);
	~CamShooter();
	
	//Functions
	void Enable();
	void Disable();
	bool IsEnabled();
	
	void PIDEnable(void);
	void Process(bool fire, bool rearm, bool eject = false);
	void InnerProcess();
	double GetPosition();
	void SetPosition(float pos);
	void logHeaders(ostream &f);
	void log(std::ostream &f);
	void Debug(std::ostream &f);
	
	bool IndexTripped();
	void Reset();
private:
	
	static void CallInnerProcess(void * camshooter);
	
	void SetUpPID(float p, float i, float d);
	void ResetPID();
	void DisablePID();
	void EnablePID();
	void SetPIDSetpoint(float setpoint);
	
	float PIDCompute(float input);
	
	SEM_ID AccessSemaphore;
	Notifier * ControlLoop;
	CamMotors * cam_outputter;
	
	bool m_enabled;
	Talon * ShooterMotorLeft;
	Talon * ShooterMotorRight;
	Encoder * ShooterEncoder;
	DigitalInput * IndexSensor;
	DigitalOutput * ScopeToggle;
	DigitalOutput * ScopeCycle;
	bool ScopeToggleState;
	bool IndexSeenLastSample;  // Was the index pulse seen during the last sample
	
	// Local copies user inputs to the tight inner loop, to avoid
	// being tied to DS packets
	bool ShouldFire;
	bool ShouldRearm;
	bool ShouldEject;
	
	// yeah, we're doing our own PID
	float m_P, m_I, m_D;
	float m_minimumInput, m_minimumOutput;
	float m_maximumInput, m_maximumOutput;
	float m_prevError, m_totalError;
	float m_setpoint;
	bool m_pidEnabled;
	
	//MotionProfile * CamProfile;
	
	typedef enum {
		ReadyToFire = 1,		// Low-energy state where we can fire quickly
		Firing      = 2,        // Releasing cam
		Rearming    = 3,        // Rearming
		Calibration  = 4,
		Testing     = 5,        // test mode. Doesn't try and do anything else
		Homing      = 6,        // Either just fired, or on startup. Wait for index pulse.
		Ejecting    = 7,        // Ejecting the ball
	} CamShooterState;
	int m_state;
	
	bool IndexHasBeenSeen;
	bool FireButtonLast;
	static const char * StateNumberToString(int state);
							
};
