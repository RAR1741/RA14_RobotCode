#include "WPIlib.h"
//#include "Config.h"
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
	void Process(); // Basic housekeeping tasks outside of operator control
	double GetPosition();
	void SetPosition(float pos);
	void logHeaders(ostream &f);
	void log(std::ostream &f);
	void Debug(std::ostream &f);
private:
	Talon * ShooterMotor;
	Encoder * ShooterEncoder;
	DigitalInput * IndexSensor;
	PIDController * PID;
	bool IndexSeenLastSample;  // Was the index pulse seen during the last sample
							
};
