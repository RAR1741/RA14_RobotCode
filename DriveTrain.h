#include "WPIlib.h"
//#include "Config.h"
#include <fstream>
#ifndef DRIVE_H__
#define DRIVE_H__

class DriveTrain
{
public:
	//Constructors and deconstructors
	DriveTrain(int fl, int fr, int rl, int rr,int leftsolforward,int leftsolreverse,int rightsolforward,int rightsolreverse);
	~DriveTrain();
	
	//Drive functions
	void Drive(double LeftStickY, double RightStickY);
	void ShiftUp (bool GetRightBumper);		
	void ShiftDown (bool GetLeftBumper);
	void logHeaders(ostream &f);
	void log(std::ostream &f);
private:

	DoubleSolenoid * LeftSol;
	DoubleSolenoid * RightSol;
	Talon * FLMotor;
	Talon * FRMotor;
	Talon * RLMotor;
	Talon * RRMotor;

	float DeadZone(float input);
	
};
#endif
