#include "CurrentSensor.h"

using namespace std;

CurrentSensor::CurrentSensor(int channel)
	: AnalogChannel(channel){
	DCOffset = 2.385;
	toggle = 0;
	count = 0;
	total = 0.0;
	kAmpsPerVolt = 50.0;
	dig = new DigitalOutput(10);
}
CurrentSensor::CurrentSensor(int module, int channel)
	: AnalogChannel(module, channel)
{
	toggle = 0;
	dig = new DigitalOutput(10);
}
void CurrentSensor::Calibrate()
{
	DCOffset = 0.99*DCOffset + 0.01*(this->GetVoltage());
}
CurrentSensor::~CurrentSensor()
{
	
}
void CurrentSensor::Toggle(int toggleInput)
{
	if (count++ < 10)
	{
		total += (this->GetVoltage() - DCOffset) * kAmpsPerVolt;
	}
	else
	{
		dig->Set(1); //Reseting the peak detector
		cout<<"Average: "<<total*.1<<" ( "<<DCOffset<<" )"<<endl;
		total = 0;
		count = 0;
		dig->Set(0); //Release peak detector from reset
	}
}
