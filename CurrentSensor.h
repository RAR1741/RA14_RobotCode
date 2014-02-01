#include "WPILib.h"
#include <iostream>

class CurrentSensor : public AnalogChannel
{
public:
	CurrentSensor(int channel);
	CurrentSensor(int channel, int module);
	void Calibrate();
	~CurrentSensor();
	void Toggle(int toggleInput);
private:
	double total;
	int count;
	int toggle;
	float kAmpsPerVolt;
	float DCOffset;
	DigitalOutput * dig;
};
