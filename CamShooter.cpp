#include "CamShooter.h"
#include "Config.h"
#include "WPILib.h"
#include <iomanip>
#include <cmath>

using namespace std;

#define CAM_READY_TO_FIRE_POSITION ( Config::GetSetting("cam_ready_to_fire_position", 35) )
#define CAM_FIRE_TO_POSITION ( Config::GetSetting("cam_fire_to_position", 45) )
#define CAM_FIRE_POSITION_TOLERANCE ( Config::GetSetting("cam_fire_position_tolerance",3) )

CamShooter::CamShooter(int motorLeft,int motorRight, int encoderA, int encoderB, int indexInput)
{
	const float lines_per_rev = 2;
	ShooterMotorLeft = new Talon(motorLeft);
	ShooterMotorRight = new Talon(motorRight);
	ShooterEncoder = new Encoder(encoderA, encoderB, false, Encoder::k4X);
	IndexSensor = new DigitalInput(indexInput);
	
	ShooterEncoder->SetDistancePerPulse(50 / 8.64);
	ShooterEncoder->SetPIDSourceParameter(Encoder::kDistance);
	ShooterEncoder->Reset();
	ShooterEncoder->Start();
	
	IndexSeenLastSample = false;
	
	PID = new PIDController(Config::GetSetting("cam_p", 0.04),
								Config::GetSetting("cam_i", 0.005), 
								Config::GetSetting("cam_d", 0.03), 
								ShooterEncoder, 
								ShooterMotorLeft);
	PID->SetInputRange(0, lines_per_rev);
	PID->SetOutputRange(-1, 1);
	PID->SetContinuous(true);
	
	/*PIDRight = new PIDController(Config::GetSetting("cam_p", 0.04),
								Config::GetSetting("cam_i", 0.005), 
								Config::GetSetting("cam_d", 0.03), 
								ShooterEncoder, 
								ShooterMotorRight);
	PIDRight->SetInputRange(0, lines_per_rev);
	PIDRight->SetOutputRange(-1, 1);
	PIDRight->SetContinuous(true);
	*/
	PID->Enable();
	//PIDRight->Enable();
	
	m_state = CamShooter::Calibration;
	
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
	//PIDRight->Reset();
	//PIDRight->Disable();
	/*PIDRight->SetPID(
			Config::GetSetting("cam_p", 0.04),
			Config::GetSetting("cam_i", 0.005),
			Config::GetSetting("cam_d", 0.03)
			);
	*/
}
void CamShooter::PIDEnable() {
	PID->Enable();
	//PIDRight->Enable();
}

double CamShooter::GetPosition()
{
	return ShooterEncoder->Get();
}

const char * CamShooter::StateNumberToString(int state)
{
	switch (state) {
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
	default:
		// ERROR
		return "Unknown";
		break;
	}
}

/**
 * @brief Peforms basic housekeeping logic outside of operator control.
 */
void CamShooter::Process(bool fire)
{
	bool IndexSeen = IndexTripped();
	bool FireButton = fire;
	float setpoint = PID->GetSetpoint();
	
	float cycle_period = Config::GetSetting("robot_loop_period", 0.05);
	float rate = Config::GetSetting("robot_cam_rearm_rate", 30);
	
	float lines_forward = cycle_period * rate;
	std::cout << "Lines Forward =" << lines_forward << std::endl;

	if (IndexSeen && !IndexSeenLastSample) {
		ShooterEncoder->Reset();
		setpoint = 0 + lines_forward;
		IndexHasBeenReset = true;
		std::cout << "Reset encoders because of index" << std::endl;
	}
	else {
		IndexHasBeenReset = false;
	}
	
	cout << "setpoint before select" << setpoint << endl;
	switch (m_state) {
	case CamShooter::Rearming:
		setpoint += lines_forward;

		if (::fabs(CAM_READY_TO_FIRE_POSITION - setpoint) < CAM_FIRE_POSITION_TOLERANCE) {
			m_state = CamShooter::ReadyToFire;
		}
		
		break;
	case CamShooter::ReadyToFire:
		setpoint = CAM_READY_TO_FIRE_POSITION;
		
		if (!FireButtonLast && FireButton) {
			m_state = CamShooter::Firing;
		}
		break;
	case CamShooter::Firing:
		setpoint = CAM_FIRE_TO_POSITION;
		
		if (ShooterEncoder->GetDistance() >= (CAM_FIRE_TO_POSITION - 1)) {
			m_state = CamShooter::Rearming;
		}
		break;
	case CamShooter::Calibration:
		cout << "setpoint inside calibration " << setpoint << endl;
		setpoint += lines_forward;
		if(IndexHasBeenReset){
			m_state = CamShooter::Rearming;
		}

		break;
		
	default:
		// ERROR
		break;
	}
	cout << "setpoint post-select " << setpoint << endl;
	PID->SetSetpoint(setpoint);
	////PIDRight->SetSetpoint(setpoint);
	cout << "PID setpoint" << PID->GetSetpoint() << endl;
	IndexSeenLastSample = IndexSeen;
	FireButtonLast = FireButton;
}
void CamShooter::SetPosition(float pos)
{

	PID->SetSetpoint(pos);
	////PIDRight->SetSetpoint(pos);
}
void CamShooter::logHeaders(ostream &f)
{
	f << "ShooterMotorLeftPositionSetpoint,ShooterMotorPositionActual,ShooterMotorLeftDemand,CamIndex,ShooterMotorRightPositionSetpoint,ShooterMotorRightDemand";
}
void CamShooter::log(ostream &f)
{
	f << PID->GetSetpoint() << "," << ShooterEncoder->GetDistance() << "," << ShooterMotorLeft->Get() << "," << IndexTripped() << ",";
	//f << PIDRight->GetSetpoint() << "," << ShooterMotorRight->Get() << ",";

}

void CamShooter::Debug(ostream &out) 
{
	out << "Encoder: " << fixed << setprecision(2) << ShooterEncoder->GetDistance()
		<< " Setpoint: " << fixed << setprecision(2) << PID->GetSetpoint() 
		<< " Motor: " << fixed << setprecision(2) << ShooterMotorLeft->Get() 
		<< " Sensor: " << (IndexTripped() ? "SEEN" : "")
		
		<< " State: " << CamShooter::StateNumberToString(m_state) << endl;
	out.unsetf ( std::ios::fixed ); 
}



