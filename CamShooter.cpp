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
	ShooterEncoder->SetPIDSourceParameter(Encoder::kDistance);
	ShooterEncoder->Reset();
	ShooterEncoder->Start();
	
	IndexSeenLastSample = false;
	
	PID = new PIDController(Config::GetSetting("cam_p", 0.04),
								Config::GetSetting("cam_i", 0.005), 
								Config::GetSetting("cam_d", 0.03), 
								ShooterEncoder, 
								cam_outputter);
	PID->SetInputRange(0, 110);
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
	//PIDRight->Enable();
#if 1
	PID->Enable();
	m_state = CamShooter::Calibration;
#else
	m_state = CamShooter::Testing;
#endif
	
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
	RA_DEBUG(lines_forward);

	if (IndexSeen && !IndexSeenLastSample) {
		ShooterEncoder->Reset();
		setpoint = 0 + lines_forward;
		IndexHasBeenReset = true;
		std::cout << "Reset encoders because of index" << std::endl;
	}
	else {
		IndexHasBeenReset = false;
	}
	switch (m_state) {
	case CamShooter::Rearming:
		if (::fabs(ShooterEncoder->GetDistance() - setpoint) <= CAM_SETPOINT_ERROR_LIMIT) {
			setpoint += lines_forward;
		}

		if (::fabs(CAM_READY_TO_FIRE_POSITION - setpoint) < CAM_FIRE_POSITION_TOLERANCE) {
			m_state = CamShooter::ReadyToFire;
		}
		
		break;
	case CamShooter::ReadyToFire:
		setpoint = CAM_READY_TO_FIRE_POSITION;
		
		if (!FireButtonLast && FireButton) {
			m_state = CamShooter::Firing;
		}
		
		if (ShooterEncoder->GetDistance() >= CAM_POINT_OF_NO_RETURN) {
			m_state = CamShooter::Firing;
			cout << "OK, I guess we're firing now. Fine. Whatever." << endl;
		}
		break;
	case CamShooter::Firing:
		setpoint = CAM_FIRE_TO_POSITION;
		
		if (ShooterEncoder->GetDistance() >= (CAM_FIRE_TO_POSITION - 1)) {
			setpoint = ShooterEncoder->GetDistance(); // make sure we don't back-drive
			m_state = CamShooter::Rearming;
		}
		break;
	case CamShooter::Calibration:
		if (::fabs(ShooterEncoder->GetDistance() - setpoint) <= CAM_SETPOINT_ERROR_LIMIT) {
			setpoint += lines_forward;
		}
		if(IndexHasBeenReset){
			m_state = CamShooter::Rearming;
		}

		break;
	case CamShooter::Testing:
		PID->Disable();
		PID->Reset();
		break;
	default:
		// ERROR
		break;
	}
	RA_DEBUG(setpoint);
	PID->SetSetpoint(setpoint);
	////PIDRight->SetSetpoint(setpoint);
	IndexSeenLastSample = IndexSeen;
	FireButtonLast = FireButton;
}
void CamShooter::SetPosition(float pos)
{
	//PID->SetSetpoint(pos);
	////PIDRight->SetSetpoint(pos);
}
void CamShooter::logHeaders(ostream &f)
{
	f << "ShooterMotorLeftPositionSetpoint,ShooterMotorPositionActual,ShooterMotorLeftDemand,CamIndex,ShooterState";
}
void CamShooter::log(ostream &f)
{
	f << PID->GetSetpoint() << "," << ShooterEncoder->GetDistance() << "," << ShooterMotorLeft->Get() << "," << IndexTripped() << ",";
	f << m_state;
}

void CamShooter::Debug(ostream &out) 
{
	out << "Encoder: "   << fixed << setprecision(2) << ShooterEncoder->GetDistance()
		<< " Setpoint: " << fixed << setprecision(2) << PID->GetSetpoint() 
		<< " Motor: "    << fixed << setprecision(2) << ShooterMotorLeft->Get() 
		<< " Sensor: "   << (IndexTripped() ? "SEEN" : "")
		
		<< " State: " << CamShooter::StateNumberToString(m_state) << endl;
	out.unsetf ( std::ios::fixed ); 
}



