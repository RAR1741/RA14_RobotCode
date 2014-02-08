#ifndef TARGET_H__
#define TARGET_H__

#include <string>

class Target {
public:
	Target();
	
	bool Parse(std::string str);
	
	float GetDistance();
	float GetX(); // normalized, [-1,1] 
	float GetY(); // normalized, [-1,1]
	
	bool IsHot();
	bool IsLeft();
	bool IsRight();
	
	bool IsValid() { return (distance > 0); }
private:
	
	float distance;
	float x;
	float y;
	float hot;
	float left;

};

#endif
