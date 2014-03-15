#ifndef DROP_SENSOR_H__
#define DROP_SENSOR_H__

#include "wpilib.h"

class DropSensor : AnalogChannel
{
public:
	DropSensor(int channel);
	DropSensor(int module, int channel);
	
	float GetPosition();
};

#endif
