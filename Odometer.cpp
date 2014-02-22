#include "Odometer.h"
#include <iostream>

using namespace std;

Odometer::Odometer(int channelA, int channelB)
{
	encoder = new Encoder(channelA,channelB,false,Encoder::k4X);
	encoder->Start();
}
void Odometer::Reset(GearRatio ratio)
{
	encoder->Stop();
	if(ratio == Odometer::highGear)
	{
		encoder->SetDistancePerPulse(DRIVE_TRAIN_GEAR_RATIO_HIGH);
	}
	else if(ratio == Odometer::lowGear)
	{
		encoder->SetDistancePerPulse(DRIVE_TRAIN_GEAR_RATIO_LOW);
	}
	encoder->Reset();
	encoder->Start();
}
void Odometer::ChangeRatio(GearRatio ratio)
{
	if(ratio == Odometer::highGear)
	{
		encoder->SetDistancePerPulse(DRIVE_TRAIN_GEAR_RATIO_HIGH);
	}
	else if(ratio == Odometer::lowGear)
	{
		encoder->SetDistancePerPulse(DRIVE_TRAIN_GEAR_RATIO_LOW);
	}
}
double Odometer::getDistance()
{
	return encoder->GetDistance();
}
