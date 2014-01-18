#include "WPIlib.h"
//#include "Config.h"
#include <fstream>
#ifndef DRIVE_H__
#define DRIVE_H__

class DriveTrain
{
public:
	//Constructors and deconstructors
	DriveTrain(int fl, int rl, int fr, int rr, int leftsolforward,
			int leftsolreverse,int rightsolforward,int rightsolreverse,
			int leftencoder_a, int leftencoder_b,
			int rightencoder_a, int rightencoder_b);
	~DriveTrain();
	
	//Drive functions
	void Drive(double LeftStickY, double RightStickY);
	void ShiftUp();		
	void ShiftDown();
	void logHeaders(ostream &f);
	void log(std::ostream &f);
	
	void Debug(std::ostream &f);
private:

	DoubleSolenoid * LeftSol;
	DoubleSolenoid * RightSol;
	Talon * FLMotor;
	Talon * FRMotor;
	Talon * RLMotor;
	Talon * RRMotor;
	
	Encoder * LEncoder;
	Encoder * REncoder;

	float DeadZone(float input);
	
	void ConfigureEncoder(Encoder *e);
	
};
#endif
