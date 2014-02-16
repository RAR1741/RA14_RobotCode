#include "WPILib.h"


class Collection
{
public:
	Collection(int spinnerWheel, int solForward, int solReverse);
	~Collection();
	void Collect();
	void ResetPosition();
	void SpinMotor(float percent);
	void ExtendArm();
	void RetractArm();
	
	bool IsExtended() { return collectSol->Get() == DoubleSolenoid::kForward; }
private:
	DoubleSolenoid * collectSol;
	Talon * spinWheel;
};
