#include "DriveTrain.h"
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>

using namespace std;


DriveTrain::DriveTrain(int fl, int rl, int fr, int rr,int leftsolforward,int leftsolreverse,int rightsolforward,int rightsolreverse,
		int leftencoder_a, int leftencoder_b, int rightencoder_a, int rightencoder_b)
{
	FLMotor = new Talon(fl);
	FRMotor = new Talon(fr);
	RLMotor = new Talon(rl);
	RRMotor = new Talon(rr);
	LeftSol = new DoubleSolenoid(leftsolforward,leftsolreverse);
	RightSol = new DoubleSolenoid(rightsolforward,rightsolreverse);
	
	LEncoder = new Encoder(leftencoder_a, leftencoder_b );
	REncoder = new Encoder(rightencoder_a, rightencoder_b);
	
	ConfigureEncoder(LEncoder);
	ConfigureEncoder(REncoder);
}

DriveTrain::~DriveTrain()
{
}

void DriveTrain::Debug(ostream & out) 
{
	out << "Left: " << LEncoder->GetRate();
	out << "\tRight: " << REncoder->GetRate();
	out << endl;
}

void DriveTrain::Drive(double LeftStickY, double RightStickY)
{
	
	FLMotor->Set(DeadZone(float(LeftStickY)));
	RLMotor->Set(DeadZone(float(LeftStickY)));
	FRMotor->Set(DeadZone(float(RightStickY * -1)));
	RRMotor->Set(DeadZone(float(RightStickY * -1)));
	
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
	if (::fabs(input) <= 0.1)return 0; //Range is set to +- 5% of the center
	return input;
}

void DriveTrain::ConfigureEncoder(Encoder * e)
{
	float feet_per_pulse = 12.5 / 12.0 / 250.0;		
	e->Reset();
	e->SetDistancePerPulse(feet_per_pulse);
	e->Start();
}
