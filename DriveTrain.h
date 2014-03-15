#include "WPIlib.h"
#include "SpeedControlTalon.h"
#include "Odometer.h"
//#include "Config.h"
#include <fstream>
#ifndef DRIVE_H__
#define DRIVE_H__

class DriveTrain
{
public:
	//Constructors and deconstructors
	DriveTrain(int fl, int rl, int fr, int rr,
			int leftsolforward, int leftsolreverse,
			int leftencoder_a, int leftencoder_b,
			int rightencoder_a, int rightencoder_b);
	DriveTrain(int fl, int rl, int fr, int rr,
			int leftsolforward, int leftsolreverse,
			int leftencoder_a, int leftencoder_b,
			int rightencoder_a, int rightencoder_b,
			int sensor1, int sensor2, int sensor3, int sensor4);
	~DriveTrain();
	
	//Drive functions
	float GetOdometer() { return REncoder->GetDistance(); }
	void ResetOdometer() { LEncoder->Reset(); REncoder->Reset(); }
	void Drive(double LeftStickY, double RightStickY);
	void DriveArcade(float x, float y);
	void ShiftUp();
	void ShiftDown();
	void logHeaders(ostream &f);
	void log(std::ostream &f);
	
	void reverseDirectionForward();
	void reverseDirectionReverse();
	
	//return odometer object
	//Odometer* getOdometer();
	
	void Debug(std::ostream &f);
private:

	DoubleSolenoid * LeftSol;
	DoubleSolenoid * RightSol;
	Talon * FLMotor;
	Talon * FRMotor;
	Talon * RLMotor;
	Talon * RRMotor;
	
	SpeedControlTalon * SpeedControlFLMotor;
	SpeedControlTalon * SpeedControlFRMotor;
	SpeedControlTalon * SpeedControlRLMotor;
	SpeedControlTalon * SpeedControlRRMotor;
	
	Encoder * LEncoder;
	Encoder * REncoder;
	
	//Odometer
	Odometer* odometer;
	
	//Reverse
	bool reverseDriving;

	float DeadZone(float input);
	
	void ConfigureEncoder(Encoder *e);
	
};
#endif
