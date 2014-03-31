#include "CamShooter.h"
#include "Config.h"
#include "WPILib.h"
#include <iomanip>
#include <cmath>
#include <iostream>

using namespace std;

#define CAM_READY_TO_FIRE_POSITION ( Config::GetSetting("cam_ready_to_fire_position", 35) )
#define CAM_FIRE_TO_POSITION ( Config::GetSetting("cam_fire_to_position", 45) )
#define CAM_FIRE_POSITION_TOLERANCE ( Config::GetSetting("cam_fire_position_tolerance",3) )
#define CAM_POINT_OF_NO_RETURN ( Config::GetSetting("cam_point_of_no_return", 58) )
#define CAM_SETPOINT_ERROR_LIMIT ( Config::GetSetting("cam_setpoint_error_limit", 25))

CamShooter::CamShooter(int motorLeft,int motorRight, int encoderA, int encoderB, int indexInput, 
		int scopeTogglePort, int scopeCyclePort, float period)
{
	ScopeToggleState = false;
	AccessSemaphore = semMCreate(SEM_Q_PRIORITY);
	ControlLoop = new Notifier(CamShooter::CallInnerProcess, this);
	const float lines_per_rev = 360;
	const float gear_reduction_ratio = (2250.0/49.0);
	ShooterMotorLeft = new Talon(motorLeft);
	ShooterMotorRight = new Talon(motorRight);
	cam_outputter = new CamMotors(ShooterMotorLeft, ShooterMotorRight);
	ShooterEncoder = new Encoder(encoderA, encoderB, false, Encoder::k4X);
	IndexSensor = new DigitalInput(indexInput);
	ScopeToggle = new DigitalOutput(scopeTogglePort);
	ScopeCycle = new DigitalOutput(scopeCyclePort);
	
	ShooterEncoder->SetDistancePerPulse(100.0 / (gear_reduction_ratio * lines_per_rev) );
	//ShooterEncoder->SetDistancePerPulse(1);
	ShooterEncoder->SetPIDSourceParameter(Encoder::kDistance);
	ShooterEncoder->Reset();
	ShooterEncoder->Start();
	
	/*
	PID = new PIDController(Config::GetSetting("cam_p", 0.04),
								Config::GetSetting("cam_i", 0.005), 
								Config::GetSetting("cam_d", 0.03), 
								ShooterEncoder, 
								cam_outputter);
	*/
	SetUpPID( Config::GetSetting("cam_p", 0.04),
			  Config::GetSetting("cam_i", 0.005),
			  Config::GetSetting("cam_d", 0.03)
			);
	m_setpoint = 0;
	DisablePID();
	m_minimumInput = 0;
	m_maximumInput = 100 + 10; // 10 is the headroom if the homing is WAY off
	m_minimumOutput = -1;
	m_maximumOutput = 1;
	
	/*
	PID->SetInputRange(0, 110);
	PID->SetOutputRange(-1,1);
	//PID->SetContinuous(true);
	PID->SetTolerance(CAM_FIRE_POSITION_TOLERANCE);
	*/

	//PID->Enable();
	m_state = CamShooter::Homing;
	
	IndexHasBeenSeen = false;
	m_enabled = false;
	ControlLoop->StartPeriodic(period);
}

void CamShooter::SetUpPID(float p, float i, float d)
{
	m_P = p;
	m_I = i;
	m_D = d;
	ResetPID();
}

void CamShooter::ResetPID()
{
	m_totalError = m_prevError = 0;
}

void CamShooter::SetPIDSetpoint(float setpoint)
{
	m_setpoint = setpoint;
}

float CamShooter::PIDCompute(float input)
{
	float m_error = m_setpoint - input;
	float m_result = 0;
	if (::fabs(m_error) > (m_maximumInput - m_minimumInput) / 2)
	{
		if (m_error > 0)
		{
			m_error = m_error - m_maximumInput + m_minimumInput;
		} else {
			m_error = m_error + m_maximumInput - m_minimumInput;
		}
	}
	
	if (m_I != 0)
	{
		double potentialIGain = (m_totalError + m_error) * m_I;
		if (potentialIGain < m_maximumOutput)
		{
			if (potentialIGain > m_minimumOutput)
				m_totalError += m_error;
			else
				m_totalError = m_minimumOutput / m_I;
		}
		else
		{
			m_totalError = m_maximumOutput / m_I;
		}
	}
	float m_F = 0; // What is this anyway?
	m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError) + m_setpoint * m_F;
	m_prevError = m_error;

	if (m_result > m_maximumOutput) m_result = m_maximumOutput;
	else if (m_result < m_minimumOutput) m_result = m_minimumOutput;
	
	return m_result;
}

CamShooter::~CamShooter()
{
	semFlush(AccessSemaphore);
	delete ControlLoop;
}

void CamShooter::Enable()
{
	CRITICAL_REGION(AccessSemaphore)
	{
		m_enabled = true;
	}
	END_REGION;
}

bool CamShooter::IsEnabled()
{
	bool enabled = false;
	CRITICAL_REGION(AccessSemaphore)
	{	
		enabled = m_enabled;
	}
	END_REGION;
	return m_enabled;
}

void CamShooter::Disable()
{
	CRITICAL_REGION(AccessSemaphore)
	{
		m_enabled = false;
		DisablePID();
	}
	END_REGION;
}

void CamShooter::EnablePID()
{
	m_pidEnabled = true;
}

void CamShooter::DisablePID()
{
	m_pidEnabled = false;
	ResetPID();
}

bool CamShooter::IndexTripped() 
{
	
	bool index_tripped = false;
	
	CRITICAL_REGION(AccessSemaphore)
	{
		index_tripped = IndexHasBeenSeen;
	}
	END_REGION;
	return index_tripped;
}

void CamShooter::CallInnerProcess(void * controller)
{
	CamShooter * control = (CamShooter*) controller;
	control->InnerProcess();
}

void CamShooter::Reset()  {
	CRITICAL_REGION(AccessSemaphore)
	{
		SetUpPID(
			Config::GetSetting("cam_p", 0.04),
			Config::GetSetting("cam_i", 0.005),
			Config::GetSetting("cam_d", 0.03)
			);
		m_state = CamShooter::Homing;
		ShooterEncoder->Reset();
		ShooterEncoder->Start();
		DisablePID();
	}
	END_REGION;
}
void CamShooter::PIDEnable() {
	CRITICAL_REGION(AccessSemaphore)
	{
		//PID->Enable();
	}
	END_REGION;
}

double CamShooter::GetPosition()
{
	float position = 0;
	
	CRITICAL_REGION(AccessSemaphore)
	{
		position = ShooterEncoder->Get();
	}
	END_REGION;
	return position;
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
	case CamShooter::Ejecting:
		return "Ejecting";
		break;
	default:
		// ERROR
		return "Unknown";
		break;
	}
}

/**
 * @brief Inner loop for CamShooter. Handles most of the work of running the system.
 * 
 * @remarks Make sure to keep code in here nice and succinct. This is one of the few places
 *          where performance is critical. 
 */
void CamShooter::InnerProcess()
{
	ScopeToggleState = !ScopeToggleState;
	ScopeToggle->Set(ScopeToggleState);
	ScopeCycle->Set(1);
	// Input section. Do all necessary input chores here, to minimize
	// the number of semtakes we use.
	bool fire = false;
	bool rearm = false;
	bool eject = false;
	bool enabled = false;
	
	// Attempt to get information like encoder counts ahead of time
	// and concentrate all set calls to the end of the function.
	float ShooterEncoderDistance;
	float PIDSetpoint = 0;
	bool SetSetpoint = false;
	float PWMOutput = 0;
	bool SetPWMOutput = false;
	
	bool ResetEncoder = false;
	
	bool PIDEnable = false;
	bool ControlPID = false;
	
	float HomeSpeed = .1;
	
	bool PIDOnTarget = false;
	
	float EjectPosition = 30;
	float PointOfNoReturn = 0;
	float ReadyToFirePosition = 0;
	float FireToPosition = 0;
	float CamFirePositionTolerance = 0;
	
	int NextState = 0;
	
	CRITICAL_REGION(AccessSemaphore)
	{
		// TODO: Get this code out of the inner loop.
				HomeSpeed = Config::GetSetting("cam_home_speed", .5);
				EjectPosition = Config::GetSetting("CAM_EJECT_POSITION", 30);
				ReadyToFirePosition = CAM_READY_TO_FIRE_POSITION;
				PointOfNoReturn = CAM_POINT_OF_NO_RETURN;
				FireToPosition = CAM_FIRE_TO_POSITION;
				CamFirePositionTolerance = CAM_FIRE_POSITION_TOLERANCE;
		enabled = m_enabled;
		fire = ShouldFire;
		rearm = ShouldRearm;
		eject = ShouldEject;
		IndexHasBeenSeen = !!IndexSensor->Get();
		
		ShooterEncoderDistance = ShooterEncoder->GetDistance();
		PIDSetpoint = m_setpoint;
		
		PIDEnable = m_pidEnabled;
		
		PIDOnTarget = (::fabs(m_setpoint - ShooterEncoderDistance) < CamFirePositionTolerance);
		
		
		NextState = m_state;
	}
	END_REGION;
	
	// If we aren't enabled, just get out of this function.
	if (enabled) {
		CRITICAL_REGION(AccessSemaphore)
		{
			switch (m_state) {
			case CamShooter::Rearming:
				if (PIDOnTarget) {
					NextState = CamShooter::ReadyToFire;
				}
				if (ShooterEncoderDistance >= PointOfNoReturn) {
					NextState = CamShooter::Firing;
				}
				break;
			case CamShooter::ReadyToFire:
				if (fire) {
					NextState = CamShooter::Firing;
					PIDEnable = true;
					ControlPID = true;
				}
				
				if (ShooterEncoderDistance >= PointOfNoReturn) {
					NextState = CamShooter::Firing;
					//cout << "OK, I guess we're firing now. Fine. Whatever." << endl;
				} else if (eject) {
					SetSetpoint = true;
					PIDSetpoint = EjectPosition;
					NextState = CamShooter::Ejecting;
				}
				break;
			case CamShooter::Firing:
				PIDSetpoint = FireToPosition;
				SetSetpoint = true;
				
				if (PIDOnTarget && rearm) {
					PIDEnable = false;
					ControlPID = true;
					NextState = CamShooter::Homing;
				}
				break;
			case CamShooter::Homing:
				// Switch to voltage mode, drive to the index.
				PWMOutput = HomeSpeed;
				SetPWMOutput = true;
				
				if (IndexHasBeenSeen) {
					ResetEncoder = true;
					PIDEnable = true;
					ControlPID = true;
					PIDSetpoint = ReadyToFirePosition;
					SetSetpoint = true;
					NextState = CamShooter::Rearming;
				}
				break;
			case CamShooter::Calibration:
				// this isn't used anymore
				break;
			case CamShooter::Ejecting:
				if (!eject) {
					PIDSetpoint = ReadyToFirePosition;
					SetSetpoint = true;
					NextState = CamShooter::Rearming;
					
				}
				break;
			case CamShooter::Testing:
				PIDEnable = false;
				ControlPID = true;
				break;
			default:
				// ERROR
				break;
			}
			
			m_state = NextState;
				
			if (ResetEncoder) {
				ShooterEncoder->Stop();
				ShooterEncoder->Reset();
				ShooterEncoder->Start();
			}
			
			if (ControlPID) {
				if (PIDEnable) {
					EnablePID();
				} else {
					DisablePID();
					ResetPID();
				}
			}
			if (SetSetpoint) {
				SetPIDSetpoint(PIDSetpoint);
			}
			if (PIDEnable) {
				PWMOutput = PIDCompute(ShooterEncoderDistance);
				SetPWMOutput = true;
			}
			
			if (SetPWMOutput) {
				cam_outputter->PIDWrite(PWMOutput);
			}
		}
		END_REGION;
	}
	
	// Cleanup section. Output all temporary variables so the outside loop can 
	// deal with the rest of the robot.
	
	ScopeCycle->Set(0);
}
/**
 * @brief Peforms basic housekeeping logic outside of operator control.
 */
void CamShooter::Process(bool fire, bool rearm, bool eject)
{
	cout << "Start of Process()" << endl;
	CRITICAL_REGION(AccessSemaphore)
	{
		ShouldFire = fire;
		ShouldRearm = rearm;
		ShouldEject = eject;
	}
	END_REGION;
	cout << "End of Process()" << endl;
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
	double setpoint, distance, left;
	bool index_tripped;
	int state = 0;
	
	CRITICAL_REGION(AccessSemaphore)
	{
		setpoint = m_setpoint;
		distance = ShooterEncoder->GetDistance();
		index_tripped = IndexHasBeenSeen;
		left = ShooterMotorLeft->Get();
		state = m_state;
	}
	END_REGION;
		
	f << setpoint << "," << distance << "," << left << "," << index_tripped << ",";
	f << state << ",";
}

void CamShooter::Debug(ostream &out) 
{
	float distance, setpoint, left;
	bool index_tripped;
	int state;
	CRITICAL_REGION(AccessSemaphore)
	{
		setpoint = m_setpoint;
		distance = ShooterEncoder->GetDistance();
		index_tripped = IndexHasBeenSeen;
		left = ShooterMotorLeft->Get();
		state = m_state;
	}
	END_REGION;
	out << "Encoder: "   <<  distance
		<< " Setpoint: " << setpoint 
		<< " Motor: "    << left 
		<< " Sensor: "   << (index_tripped ? "SEEN" : "")	
		<< " State: " << CamShooter::StateNumberToString(state) << endl;
	
}



