#include "CurrentSensor.h"
using namespace std;

CurrentSensorSlot::CurrentSensorSlot(int channel)
	: AnalogChannel(channel){
	DCOffset = 2.385;
	toggle = 0;
	count = 0;
	total = 0.0;
//	kAmpsPerVolt = 50.0;
	kAmpsPerVolt = 25.0;
	dig = new DigitalOutput(channel);
	sample_queue = new std::queue<double>();
}
CurrentSensorSlot::CurrentSensorSlot(int module, int channel)
	: AnalogChannel(module, channel)
{
	count = 0;
	total = 0.0;
	kAmpsPerVolt = 50.0;
	dig = new DigitalOutput(channel);
	toggle = 0;
	//dig = new DigitalOutput(10);
	
	sample_queue = new std::queue<double>();
}
void CurrentSensorSlot::Calibrate()
{
	DCOffset = 0.99*DCOffset + 0.01*(this->GetVoltage());
}
CurrentSensorSlot::~CurrentSensorSlot()
{
	
}

double CurrentSensorSlot::Get()
{
	return (this->GetVoltage() - DCOffset) * kAmpsPerVolt;
	
//	if (sample_queue->size() == 0) return 0;
//	return total / sample_queue->size();
}

void CurrentSensorSlot::Process()
{
	/*
	double sample = (this->GetVoltage() - DCOffset) * kAmpsPerVolt;
	
	double old_value = sample_queue->front();
	sample_queue->push(sample);
	total += sample;
	
	if (sample_queue->size() > 10) {
		sample_queue->pop();
		total -= old_value;
		
	}
	*/
}

