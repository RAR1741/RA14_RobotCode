#include "WPILib.h"
#include <iostream>
#include <queue>

class CurrentSensorSlot : public AnalogChannel
{
public:
	CurrentSensorSlot(int channel);
	CurrentSensorSlot(int channel, int module);
	~CurrentSensorSlot();
	void Calibrate();
	double Get();
	void Process();
private:
	std::queue<double> *sample_queue;
	double total;
	int count;
	int toggle;
	float kAmpsPerVolt;
	float DCOffset;
	DigitalOutput * dig;
};
