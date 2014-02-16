#include "Collection.h"

using namespace std;

Collection::Collection(int spinnerWheel, int solForward, int solReverse)
{
	collectSol = new DoubleSolenoid(solForward, solReverse);
	spinWheel = new Talon(spinnerWheel);
}

Collection::~Collection()
{
	
}

void Collection::Collect()
{
	collectSol->Set(DoubleSolenoid::kForward);
	spinWheel->Set(1);
}

void Collection::ResetPosition()
{
	collectSol->Set(DoubleSolenoid::kReverse);
	spinWheel->Set(0);
}

void Collection::SpinMotor(float percent)
{
	spinWheel->Set(percent);
}


void Collection::ExtendArm()
{
	collectSol->Set(DoubleSolenoid::kForward);
}
void Collection::RetractArm()
{
	collectSol->Set(DoubleSolenoid::kReverse);
}

