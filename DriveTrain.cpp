#include "DriveTrain.h"
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>

using namespace std;


DriveTrain::DriveTrain(int fl, int fr, int rl, int rr,int leftsolforward,int leftsolreverse,int rightsolforward,int rightsolreverse)
{
	FLMotor = new Talon(fl);
	FRMotor = new Talon(fr);
	//RLMotor = new Talon(rl);
	//RRMotor = new Talon(rr);
	LeftSol = new DoubleSolenoid(leftsolforward,leftsolreverse);
	RightSol = new DoubleSolenoid(rightsolforward,rightsolreverse);
	

}

DriveTrain::~DriveTrain()
{
}


void DriveTrain::Drive(double LeftStickY, double RightStickY)
{
	
	FLMotor->Set(DeadZone(LeftStickY));
	RLMotor->Set(DeadZone(LeftStickY));
	FRMotor->Set(DeadZone(RightStickY));
	RRMotor->Set(DeadZone(RightStickY));
	
}
void DriveTrain::ShiftUp() //Shifts to the higher gear
{
	LeftSol->Set(DoubleSolenoid::kReverse);
	RightSol->Set(DoubleSolenoid::kReverse);
	cout<<"ShiftUp"<<endl;
}
void DriveTrain::ShiftDown() //Shifts to the lower gear
{
	LeftSol->Set(DoubleSolenoid::kForward);
	RightSol->Set(DoubleSolenoid::kForward);
	cout<<"ShiftDown"<<endl;
}
void DriveTrain::logHeaders(ostream &f)
{
	/*
	char * motor_names[4] = { "FrontLeft", "FrontRight", "RearLeft", "RearRight" };
	char * header_names[3] = { "Setpoint","Output","Current"};
	
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			f << motor_names[i] << " " << header_names[j] << ","; 
		}
	}
	*/
}
void DriveTrain::log(ostream &f)
{
	/*
	//log for each module
	
	SpeedControlTalon * motors[4] = { FLMotor, FRMotor, RLMotor, RRMotor };
	CurrentSensor * sensors[4] = { FLSensor, FRSensor, RLSensor, RRSensor };  
	for (int i = 0; i < 4; i++) {
		f << motors[i]->GetSetpoint() << "," <<  motors[i]->GetOutput() << ",";
		f << sensors[i]->GetCurrent() << ",";
	}
	*/
}


float DriveTrain::DeadZone(float input) { //Returns 0 if joystick inputs are within certain range
	if (::fabs(input) <= 0.05)return 0; //Range is set to +- 5% of the center
	return input;
}

