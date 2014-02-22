#include "WPIlib.h"
#include "Config.h"

#define DRIVE_TRAIN_GEAR_RATIO_LOW ( Config::GetSetting("drive_train_gear_ratio_low", 0) )
#define DRIVE_TRAIN_GEAR_RATIO_HIGH ( Config::GetSetting("drive_train_gear_ratio_high", 0) )

class Odometer
{
public:
	typedef enum{
			highGear,lowGear
		} GearRatio;
	Odometer(int channelA, int channelB);
	void Reset(GearRatio ratio);
	void ChangeRatio(GearRatio ratio);
	double getDistance();
private:
	Encoder* encoder;
};
