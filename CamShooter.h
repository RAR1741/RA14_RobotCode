#include "WPIlib.h"
//#include "Config.h"
#include <fstream>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <iomanip>

class CamShooter
{
public:
	CamShooter(int jag1);
	~CamShooter();
	
	//Functions
	void DisplayPosition();
	void Shoot();
	void logHeaders(ostream &f);
	void log(std::ostream &f);
	void Debug(std::ostream &f);
		
private:
	CANJaguar * ShooterJag;
	
};
