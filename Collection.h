#include "WPILib.h"


class Collection
{
public:
	Collection(int spinnerWheel, int solForward, int solReverse);
	~Collection();
	void Collect();
	void ResetPosition();
	
private:
	DoubleSolenoid * collectSol;
	Talon * spinWheel;
};
