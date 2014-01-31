#ifndef MOTION_PROFILE_H__
#define MOTION_PROFILE_H__

#include "LookupTable.h"
#include "wpilib.h"

class MotionProfile {
public:
	MotionProfile(double start_position, double final_position);
	~MotionProfile();
	
	void LoadFromFile(const char * filename);
	
	void Start();
	void Stop();
	void Reset();
	
	double GetValue();
private:
	LookupTable * table;
	Timer * timer;
};

#endif
