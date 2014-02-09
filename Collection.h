#include "WPILib.h"


class Collection
{
public:
	Collection(int spinnerWheel, int solForward, int solReverse);
	~Collection();
	void Collect();
	void ResetPosition();
	void SpinMotor();
	void ActuatePneumatics();
	
private:
	DoubleSolenoid * collectSol;
	Talon * spinWheel;
};
