#include "CamShooter.h"
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
	
	PID = new PIDController(0.04, 0.005, 0.03, ShooterEncoder, ShooterMotor);
	PID->SetInputRange(0, lines_per_rev);
	PID->SetOutputRange(-1, 1);
	PID->SetContinuous(true);
	PID->Enable();
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
	return ShooterEncoder->Get();
}

/**
 * @brief Peforms basic housekeeping logic outside of operator control.
 */
void CamShooter::Process()
{
	bool IndexSeen = !!!IndexSensor->Get();
	bool RisingEdge = false;
	bool FallingEdge = false;
	
	if (IndexSeen && !IndexSeenLastSample) {
		// Rising edge. We just saw the index pulse for the first time
		RisingEdge = true;
	} else if (!IndexSeen && IndexSeenLastSample) {
		// Falling edge. We are no longer seeing the index pulse
		FallingEdge = true;
	}
	
	if (RisingEdge) {
		// Reset position count to zero.
		ShooterEncoder->Reset();
		ShooterEncoder->Start();
		PID->SetSetpoint(90);
		PID->Reset();
		PID->Enable();
	}
	
	IndexSeenLastSample = IndexSeen; // Record for next iteration
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

void CamShooter::Debug(ostream &out) 
{
	out << "Encoder: " << fixed << setprecision(6) << ShooterEncoder->GetDistance() << 
	 "  Motor: " << fixed << setprecision(6) << ShooterMotor->Get() << 
	 "Sensor: " << (!!!IndexSensor->Get() ? "SEEN" : "") << endl;
}

