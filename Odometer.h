#include "WPIlib.h"
#include "Config.h"

class Odometer
{
public:
	Odometer(int channelA, int channelB);
	void Reset();
	double getDistance();
private:
	Encoder* encoder;
};
