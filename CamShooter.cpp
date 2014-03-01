#include "CamShooter.h"
#include "Config.h"
#include "WPILib.h"
#include <iomanip>
#include <cmath>

using namespace std;

#define CAM_READY_TO_FIRE_POSITION ( Config::GetSetting("cam_ready_to_fire_position", 35) )
#define CAM_FIRE_TO_POSITION ( Config::GetSetting("cam_fire_to_position", 45) )
#define CAM_FIRE_POSITION_TOLERANCE ( Config::GetSetting("cam_fire_position_tolerance",3) )
#define CAM_POINT_OF_NO_RETURN ( Config::GetSetting("cam_point_of_no_return", 58) )
#define CAM_SETPOINT_ERROR_LIMIT ( Config::GetSetting("cam_setpoint_error_limit", 25))

CamShooter::CamShooter(int motorLeft,int motorRight, int encoderA, int encoderB, int indexInput)
{
	const float lines_per_rev = 360;
	const float gear_reduction_ratio = (2250.0/49.0);
	ShooterMotorLeft = new Talon(motorLeft);
	ShooterMotorRight = new Talon(motorRight);
	cam_outputter = new CamMotors(ShooterMotorLeft, ShooterMotorRight);
	ShooterEncoder = new Encoder(encoderA, encoderB, false, Encoder::k4X);
	IndexSensor = new DigitalInput(indexInput);
	
	ShooterEncoder->SetDistancePerPulse(100.0 / (gear_reduction_ratio * lines_per_rev) );
	//ShooterEncoder->SetDistancePerPulse(1);
	ShooterEncoder->SetPIDSourceParameter(Encoder::kDistance);
	ShooterEncoder->Reset();
	ShooterEncoder->Start();
	
	PID = new PIDController(Config::GetSetting("cam_p", 0.04),
								Config::GetSetting("cam_i", 0.005), 
								Config::GetSetting("cam_d", 0.03), 
								ShooterEncoder, 
								cam_outputter);
	PID->SetInputRange(0, 110);
	PID->SetOutputRange(-1,1);
	//PID->SetContinuous(true);
	PID->SetTolerance(CAM_FIRE_POSITION_TOLERANCE);

	//PID->Enable();
	m_state = CamShooter::Homing;
	
	IndexHasBeenReset = false;
	FireButtonLast = false;
}
CamShooter::~CamShooter()
{
}

void CamShooter::Reset()  {
	PID->Reset();
	PID->Disable();
	PID->SetPID(
			Config::GetSetting("cam_p", 0.04),
			Config::GetSetting("cam_i", 0.005),
			Config::GetSetting("cam_d", 0.03)
			);
	m_state = CamShooter::Homing;
}
void CamShooter::PIDEnable() {
	PID->Enable();
}

double CamShooter::GetPosition()
{
	return ShooterEncoder->Get();
}

const char * CamShooter::StateNumberToString(int state)
{
	switch (state) {
	case CamShooter::Testing:
		return "Testing";
		break;
	case CamShooter::Rearming:
		return "Rearming";
		break;
	case CamShooter::Firing:
		return "Firing";
		break;
	case CamShooter::ReadyToFire:
		return "ReadyToFire";
		break;
	case CamShooter::Calibration:
		return "Calibration";
		break;
	case CamShooter::Homing:
		return "Homing";
		break;
	default:
		// ERROR
		return "Unknown";
		break;
	}
}

/**
 * @brief Peforms basic housekeeping logic outside of operator control.
 */
void CamShooter::Process(bool fire, bool rearm)
{
	switch (m_state) {
	case CamShooter::Rearming:
		float speed =  Config::GetSetting("cam_home_speed", .5);
		cam_outputter->PIDWrite(speed);
		//PID->SetSetpoint(CAM_READY_TO_FIRE_POSITION);
		//if (PID->OnTarget()) {
		if (CAM_READY_TO_FIRE_POSITION - ShooterEncoder->GetDistance() <= CAM_FIRE_POSITION_TOLERANCE) {
			cam_outputter->PIDWrite(0);
			PID->SetSetpoint(CAM_READY_TO_FIRE_POSITION);
			PID->Reset();
			PID->Enable();
			m_state = CamShooter::ReadyToFire;
		}
		if (ShooterEncoder->GetDistance() >= CAM_POINT_OF_NO_RETURN) {
					m_state = CamShooter::Firing;
				}
		break;
	case CamShooter::ReadyToFire:
		if (fire) {
			m_state = CamShooter::Firing;
			PID->Reset();
			PID->Enable();
		}
		
		if (ShooterEncoder->GetDistance() >= CAM_POINT_OF_NO_RETURN) {
			m_state = CamShooter::Firing;
			cout << "OK, I guess we're firing now. Fine. Whatever." << endl;
		}
		break;
	case CamShooter::Firing:
		PID->SetSetpoint(CAM_FIRE_TO_POSITION);
		
		if (PID->OnTarget() && rearm) {
			m_state = CamShooter::Homing;
		}
		break;
	case CamShooter::Homing:
		// Switch to voltage mode, drive to the index.
		PID->Disable();
		cam_outputter->PIDWrite(Config::GetSetting("CAM_HOME_SPEED", .5));
		
		if (IndexTripped()) {
			ShooterEncoder->Stop();
			ShooterEncoder->Reset();
			ShooterEncoder->Start();
			
			//PID->Enable();
			
			m_state = CamShooter::Rearming;
		}
		break;
	case CamShooter::Calibration:
		// this isn't used anymore
		break;
	case CamShooter::Testing:
		PID->Disable();
		PID->Reset();
		break;
	default:
		// ERROR
		break;
	}
}
void CamShooter::SetPosition(float pos)
{
	
}
void CamShooter::logHeaders(ostream &f)
{
	f << "ShooterMotorLeftPositionSetpoint,ShooterMotorPositionActual,ShooterMotorLeftDemand,CamIndex,ShooterState,";
}
void CamShooter::log(ostream &f)
{
	f << PID->GetSetpoint() << "," << ShooterEncoder->GetDistance() << "," << ShooterMotorLeft->Get() << "," << IndexTripped() << ",";
	f << m_state << ",";
}

void CamShooter::Debug(ostream &out) 
{
	out << "Encoder: "   <<  ShooterEncoder->GetDistance()
		<< " Setpoint: " << PID->GetSetpoint() 
		<< " Motor: "    << ShooterMotorLeft->Get() 
		<< " Sensor: "   << (IndexTripped() ? "SEEN" : "")	
		<< " State: " << CamShooter::StateNumberToString(m_state) << endl;
}



