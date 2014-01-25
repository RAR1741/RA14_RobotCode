#include "CamShooter.h"


using namespace std;

CamShooter::CamShooter(int jag1)
{
	ShooterJag = new CANJaguar(jag1);
	
	//ShooterJag->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
	//ShooterJag->SetPID(1,0,0);
	//ShooterJag->ConfigEncoderCodesPerRev(100);
	//ShooterJag->EnableControl(0);
}
CamShooter::~CamShooter()
{
}
double CamShooter::GetPosition()
{
	return ShooterJag->GetPosition();
}
void CamShooter::SetPosition(bool pos)
{
	if(pos == false)
		ShooterJag->Set(.25);
	else if(pos == true)
		ShooterJag->Set(-.25);
}
void CamShooter::logHeaders(ostream &f)
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
void CamShooter::log(ostream &f)
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

