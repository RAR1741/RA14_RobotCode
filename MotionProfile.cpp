#include "MotionProfile.h"

MotionProfile::MotionProfile(double start_position, double end_position)
{
	table = new LookupTable(start_position, end_position);
	timer = new Timer();
}

MotionProfile::~MotionProfile()
{
	
}

void MotionProfile::LoadFromFile(const char *filename)
{
	table->LoadFromFile(filename);
}

void MotionProfile::Start()
{
	timer->Start();
}

void MotionProfile::Stop()
{
	timer->Stop();
}

void MotionProfile::Reset()
{
	timer->Reset();
}

double MotionProfile::GetValue()
{
	return table->Lookup( timer->Get() );
}


