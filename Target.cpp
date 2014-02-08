#include "Target.h"
#include <sstream>

Target::Target()
{
	distance = x = y = left = hot = 0;
}

bool Target::Parse(std::string str)
{
	if (str.length() > 0) {
		std::istringstream in(str);
		
		in >> distance >> x >> y >> left >> hot;
		
		return true;
	} else {
		distance = x = y = left = hot = 0;
		return false;
	}
}

float Target::GetDistance() {
	return distance;
}

float Target::GetX() {
	return x;
}

float Target::GetY() {
	return y;
}

bool Target::IsHot() {
	return hot >= 1;
}

bool Target::IsLeft() {
	return left >= 1;
}

bool Target::IsRight()
{
	return !IsLeft();
}


