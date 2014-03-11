#include "Odometer.h"
#include <iostream>
#include <cmath>
#include <math.h>

using namespace std;

Odometer::Odometer(int channelA, int channelB)
{
	encoder = new Encoder(channelA,channelB,false,Encoder::k4X);
	encoder->Start();
}
void Odometer::Reset()
{
	encoder->Stop();
	encoder->SetDistancePerPulse( ( acos(-1) * 4) / 250 );
	encoder->Reset();
	encoder->Start();
}
double Odometer::getDistance()
{
	return fabs( encoder->GetDistance() );
}
