#include "CamShooter.h"
#include "Config.h"
#include "WPILib.h"
#include <iomanip>

using namespace std;


CamShooter::CamShooter(int motor, int encoderA, int encoderB, int indexInput)
{
	const float lines_per_rev = 100;
	//ShooterJag = new CANJaguar(jag1,CANJaguar::kPercentVbus);
	ShooterMotor = new Talon(motor);
	ShooterEncoder = new Encoder(encoderA, encoderB, false, Encoder::k4X);
	IndexSensor = new DigitalInput(indexInput);
	
	ShooterEncoder->SetDistancePerPulse(1);
	ShooterEncoder->SetPIDSourceParameter(Encoder::kDistance);
	ShooterEncoder->Reset();
	ShooterEncoder->Start();
	
	IndexSeenLastSample = false;
	
	PID = new PIDController(Config::GetSetting("cam_p", 0.04),
							Config::GetSetting("cam_i", 0.005), 
							Config::GetSetting("cam_d", 0.03), 
							ShooterEncoder, 
							ShooterMotor);
	PID->SetInputRange(0, lines_per_rev);
	PID->SetOutputRange(-1, 1);
	PID->SetContinuous(true);
	PID->Enable();
	
	m_state = CamShooter::Rearming;
	
	//CamProfile = new MotionProfile(0, 100);
	
	FireButtonLast = false;
	
	InitializeProfile();
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
	PID->Enable();
}

void CamShooter::InitializeProfile()
{
	//CamProfile->LoadFromFile("camprofile.txt");
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
	case CamShooter::ExitFiring:
		return "ExitFiring";
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
	switch (m_state) {
	case CamShooter::Rearming:
		if (IndexSeen && !IndexSeenLastSample) {
			ShooterEncoder->Reset();
			setpoint = 0 + lines_forward;
		}
		
		setpoint += lines_forward;
		
		/*
		if (ShooterEncoder->GetDistance() >= 50.0) {
			m_state = CamShooter::ReadyToFire;
		}
		*/
		if (setpoint >= 50.0) {
			m_state = CamShooter::ReadyToFire;
		}
		
		break;
	case CamShooter::ReadyToFire:
		setpoint = 50;
		
		if (!FireButtonLast && FireButton) {
			m_state = CamShooter::Firing;
		}
		break;
	case CamShooter::Firing:
		setpoint = 60;
		
		if (ShooterEncoder->GetDistance() >= 60) {
			m_state = CamShooter::ExitFiring;
		}
		break;
	case CamShooter::ExitFiring:
		if (IndexSeen && !IndexSeenLastSample) {
			ShooterEncoder->Reset();
			setpoint = 0 + lines_forward;
			m_state = CamShooter::Rearming;
		}
				
		setpoint += lines_forward;
		break;
	default:
		// ERROR
		break;
	}
	
	PID->SetSetpoint(setpoint);
	IndexSeenLastSample = IndexSeen;
	FireButtonLast = FireButton;
}
void CamShooter::SetPosition(float pos)
{
	/*
	if(pos == false)
		ShooterJag->Set(.25);
	else if(pos == true)
		ShooterJag->Set(-.25);
	*/
	PID->SetSetpoint(pos);
}
void CamShooter::logHeaders(ostream &f)
{
	f << "ShooterMotorPositionSetpoint,ShooterMotorPositionActual,ShooterMotorDemand,CamIndex,";
}
void CamShooter::log(ostream &f)
{
	f << PID->GetSetpoint() << "," << ShooterEncoder->GetDistance() << "," << ShooterMotor->Get() << "," << IndexTripped() << ",";
}

void CamShooter::Debug(ostream &out) 
{
	out << "Encoder: " << fixed << setprecision(2) << ShooterEncoder->GetDistance()
		<< " Setpoint: " << fixed << setprecision(2) << PID->GetSetpoint() 
		<< " Motor: " << fixed << setprecision(2) << ShooterMotor->Get() 
		<< " Sensor: " << (IndexTripped() ? "SEEN" : "")
		
		<< " State: " << CamShooter::StateNumberToString(m_state) << endl;
	out.unsetf ( std::ios::fixed ); 
}



