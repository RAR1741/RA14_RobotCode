#include "RAUtils.h"
#include <cmath>

// round @x to the nearest @positions place.
float round_to(float x, int positions)
{
	// might be slight loss of precision here
	// don't care for most applications.
	//todo: figure out the Knuthy way to do this :)
	
	int pow10 = 1;
	for (int i = 0; i < positions; ++i) {
		pow10 *= 10;
	}
	
	float temp = x * pow10;
	
	// this may be vxworks-specific
	return ::roundf(temp) / pow10;
}
