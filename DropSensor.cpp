#include "DropSensor.h"

static const float kVoltsToPosition = 1;

DropSensor::DropSensor(int channel)
	: AnalogChannel(channel)
{
	
}

DropSensor::DropSensor(int module_number, int channel)
	: AnalogChannel(module_number)
{
	
}

float DropSensor::GetPosition()
{
	return GetVoltage() * kVoltsToPosition;
}
