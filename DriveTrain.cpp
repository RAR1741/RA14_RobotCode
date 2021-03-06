#include "DriveTrain.h"
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <iomanip>
#include <algorithm>

using namespace std;

const double kPi = 3.14159265358979;

DriveTrain::DriveTrain(int fl, int rl, int fr, int rr,int leftsolforward, int leftsolreverse,
		int leftencoder_a, int leftencoder_b, int rightencoder_a, int rightencoder_b)
{
	
	FLMotor = new Talon(fl);
	FRMotor = new Talon(fr);
	RLMotor = new Talon(rl);
	RRMotor = new Talon(rr);
	LeftSol = new DoubleSolenoid(leftsolforward,leftsolreverse);
	
	LEncoder = new Encoder(leftencoder_a, leftencoder_b );
	REncoder = new Encoder(rightencoder_a, rightencoder_b);
	
	ConfigureEncoder(LEncoder);
	ConfigureEncoder(REncoder);
	
	reverseDriving = false;
	
	//Odometer
	/*
	odometer = new Odometer(leftencoder_a,leftencoder_b);
	odometer->Reset(); */
	
}

DriveTrain::~DriveTrain()
{
}

void DriveTrain::Debug(ostream & out) 
{
	out << "Left: " << setw(10) << LEncoder->GetRate();
	out << "Right: " << setw(10) << REncoder->GetRate();
	
	out << endl;
}

void DriveTrain::Drive(double LeftStickY, double RightStickY)
{
	float left = DeadZone(float(LeftStickY));
	float right = DeadZone(float(RightStickY * -1));
	
	cout << "Left drive: " << left << "\nRight drive: " << right << endl;
	
	//if(!reverseDriving)
	//{
		FLMotor->Set(left);
		RLMotor->Set(left);
		FRMotor->Set(right);
		RRMotor->Set(right);
	//}
	//else
	//{
		//FLMotor->Set(left * -1);
		//RLMotor->Set(left * -1);
		//FRMotor->Set(right * -1);
		//RRMotor->Set(right * -1);
	//}
	
}

void DriveTrain::DriveArcade(float x, float y)
{
	float leftOut = 0;
	float rightOut = 0;
	
	x = x * xaxis_percent;
	
	if(reverseDriving)
	{
		//x = x*-1;
		y = y * -1;
	}
	
	if (y > 0) {
		if (x > 0) {
			leftOut = y - x;
			rightOut = max(y, x);
		} else {
			leftOut = max(y, -x);
			rightOut = y + x;
		}
	} else {
		if (x > 0) {
			leftOut = - max(-y, x);
			rightOut = y + x;
		} else {
			leftOut = y - x;
			rightOut = - max(-y, -x);
		}
	}
	
	Drive(leftOut, rightOut);
}

void DriveTrain::ShiftUp() //Shifts to the higher gear
{
	LeftSol->Set(DoubleSolenoid::kReverse);
	//RightSol->Set(DoubleSolenoid::kReverse);
	//cout<<"ShiftUp"<<endl;
}
void DriveTrain::ShiftDown() //Shifts to the lower gear
{
	LeftSol->Set(DoubleSolenoid::kForward);
	//RightSol->Set(DoubleSolenoid::kForward);
	//cout<<"ShiftDown"<<endl;
}

float DriveTrain::DeadZone(float input) { //Returns 0 if joystick inputs are within certain range
	if (::fabs(input) <= 0.1)return 0; //Range is set to +- 10% of the center
	return input;
}

void DriveTrain::ConfigureEncoder(Encoder * e)
{
	float feet_per_pulse = 4 * kPi / 250.0;		
	e->Reset();
	e->SetDistancePerPulse(feet_per_pulse);
	e->Start();
}

void DriveTrain::logHeaders(ostream &f)
{
	f << "FrontLeftMotorOutput,FrontRightMotorOutput,RearLeftMotorOutput,RearRightMotorOutput,LeftEncoder,RightEncoder,";
}
void DriveTrain::log(ostream &f)
{
	Talon * motors[4] = { FLMotor, FRMotor, RLMotor, RRMotor };
	for (int i = 0; i < 4; i++) {
		f << motors[i]->Get() << ",";
	}
	f << LEncoder->GetRate() << "," << REncoder->GetRate() << ",";	
}
void DriveTrain::reverseDirectionForward()
{
	reverseDriving = false;
}
void DriveTrain::reverseDirectionReverse()
{
	reverseDriving = true;
}
/*
Odometer* DriveTrain::getOdometer()
{
	return odometer;
}
*/
