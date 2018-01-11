#include <iostream>
#include <memory>
#include <string>
//#include "AHRS.h"
#include "math.h"

// And So It Begins...
#include "RJ_RobotMap.h"
#include "RJ_Auto.h"

class Robot: public frc::IterativeRobot {

	// Robot Hardware Setup
	RJ_RobotMap IO;

	// Autonomous Programs
	RJ_Auto AutoProgram;

	// Built-In Drive code for teleop
	DifferentialDrive Adrive;

	// Drive Input Filter
	float OutputX = 0.0, OutputY = 0.0;

	// Default junk for testing
	VictorSP Dpad1;
	VictorSP Dpad2;
	VictorSP RightStick1;
	VictorSP RightStick2;

	// Limit Switches
	DigitalInput DiIn8, DiIn9;

	// create pdp variable
	//PowerDistributionPanel *pdp = new PowerDistributionPanel();

	// Solenoids
	Solenoid *XYbutton = new Solenoid(4);
	Solenoid *Bbutton = new Solenoid(3);
	Solenoid *Abutton = new Solenoid(2);
	Solenoid *IntakeButton = new Solenoid(1);

	// State Variables
	bool driveButtonYPrev = false;
	bool intakeDeployed = false;
	bool XYDeployed = false;

public:
	Robot() :
		AutoProgram(&IO),
		Adrive(*IO.DriveBase.MotorLeft[0], *IO.DriveBase.MotorRight[0]), Dpad1(
					8), Dpad2(9), RightStick1(6), RightStick2(7), DiIn8(8), DiIn9(
					9) {
		// NOP
	}

private:
	void RobotInit() {
		//disable drive watchdogs
		Adrive.SetSafetyEnabled(false);

	}

	void TeleopInit() {
		// drive command averaging filter
		OutputX = 0, OutputY = 0;
	}

	void RobotPeriodic() {
		// link multiple motors together
		// TODO: Replace with a SpeedControllerGroup
		IO.DriveBase.MotorLeft[1]->Set(IO.DriveBase.MotorLeft[0]->Get());
		IO.DriveBase.MotorLeft[2]->Set(IO.DriveBase.MotorLeft[0]->Get());
		IO.DriveBase.MotorRight[1]->Set(IO.DriveBase.MotorRight[0]->Get());
		IO.DriveBase.MotorRight[2]->Set(IO.DriveBase.MotorRight[0]->Get());
	}

	void DisabledPeriodic() {
		// Update Smart Dashboard
	}

	void TeleopPeriodic() {
		double Deadband = 0.11;
		double DPadSpeed = 1.0;
		bool RightStickLimit1 = DiIn8.Get();
		bool RightStickLimit2 = DiIn9.Get();

		//high gear & low gear controls
		if (IO.DS.DriveStick->GetRawButton(5))
			IO.DriveBase.SolenoidShifter->Set(true);// High gear press LH bumper

		if (IO.DS.DriveStick->GetRawButton(6))
			IO.DriveBase.SolenoidShifter->Set(false);// Low gear press RH bumper

		//  Rumble code
		//  Read all motor current from PDP and display on drivers station
		//double driveCurrent = pdp->GetTotalCurrent();	// Get total current
		double driveCurrent = 0;

		// rumble if current to high
		double LHThr = 0.0;		// Define value for rumble
		if (driveCurrent > 125.0)// Rumble if greater than 125 amps motor current
			LHThr = 0.5;
		Joystick::RumbleType Vibrate;				// define Vibrate variable
		Vibrate = Joystick::kLeftRumble;		// set Vibrate to Left
		IO.DS.DriveStick->SetRumble(Vibrate, LHThr); // Set Left Rumble to RH Trigger
		Vibrate = Joystick::kRightRumble;		// set vibrate to Right
		IO.DS.DriveStick->SetRumble(Vibrate, LHThr);// Set Right Rumble to RH Trigger

		//drive controls
		double SpeedLinear = IO.DS.DriveStick->GetRawAxis(1) * 1; // get Yaxis value (forward)
		double SpeedRotate = IO.DS.DriveStick->GetRawAxis(4) * -1; // get Xaxis value (turn)

		// Set dead band for X and Y axis
		if (fabs(SpeedLinear) < Deadband)
			SpeedLinear = 0.0;
		if (fabs(SpeedRotate) < Deadband)
			SpeedRotate = 0.0;

		//slow down direction changes from 1 cycle to 5
		OutputY = (0.8 * OutputY) + (0.2 * SpeedLinear);
		OutputX = (0.8 * OutputX) + (0.2 * SpeedRotate);

		//drive
		if (IO.DS.DriveStick->GetRawButton(4)) {
			//boiler auto back up when y button pushed
			if (!driveButtonYPrev) {
				resetEncoder();
				//	ahrs->ZeroYaw();
				driveButtonYPrev = true;
			}
			//forward(autoBackupDistance);
		} else {
			//manual control
			driveButtonYPrev = false;
			Adrive.ArcadeDrive(OutputY, OutputX, true);
		}

		/*
		 * MANIP CODE
		 */

		//A Button to extend (Solenoid On)
		Abutton->Set(IO.DS.OperatorStick->GetRawButton(1));

		//B Button to extend (Solenoid On)
		Bbutton->Set(IO.DS.OperatorStick->GetRawButton(2));

		//if Left Bumper button pressed, extend (Solenoid On)
		if (IO.DS.OperatorStick->GetRawButton(5)) {
			intakeDeployed = true;
			IntakeButton->Set(intakeDeployed);
		}

		//else Right Bumper pressed, retract (Solenoid Off)
		else if (IO.DS.OperatorStick->GetRawButton(6)) {
			intakeDeployed = false;
			IntakeButton->Set(intakeDeployed);
		}
		//if 'X' button pressed, extend (Solenoid On)
		if (IO.DS.OperatorStick->GetRawButton(3)) {
			XYDeployed = true;
			XYbutton->Set(XYDeployed);
		}

		//else 'Y' button pressed, retract (Solenoid Off)
		else if (IO.DS.OperatorStick->GetRawButton(4)) {
			XYDeployed = false;
			XYbutton->Set(XYDeployed);
		}

		//dpad POV stuff
		if (IO.DS.OperatorStick->GetPOV(0) == 0) {
			Dpad1.Set(DPadSpeed);
			Dpad2.Set(DPadSpeed);
		} else if (IO.DS.OperatorStick->GetPOV(0) == 180) {
			Dpad1.Set(-DPadSpeed);
			Dpad2.Set(-DPadSpeed);
		} else {
			Dpad1.Set(0.0);
			Dpad2.Set(0.0);
		}

		double RightSpeed = IO.DS.OperatorStick->GetRawAxis(4) * -1; // get Xaxis value for Right Joystick

		if (fabs(RightSpeed) < Deadband) {
			RightSpeed = 0.0;
		} else if (RightSpeed > Deadband and !RightStickLimit1)
			RightSpeed = 0.0;
		else if (RightSpeed < Deadband and !RightStickLimit2)
			RightSpeed = 0.0;

		//	if (OperatorStick.GetRawAxis(2) > 0.5) {
		//		RightSpeed = 1.0;
		//	} else if (OperatorStick.GetRawAxis(3) > 0.5) {
		//		RightSpeed = -1.0;
		//	}
		//	else if (OperatorStick.GetRawAxis(4) < Deadband)
		//		RightSpeed = 0.0;

		RightStick1.Set(RightSpeed);
		RightStick2.Set(RightSpeed);

	}

	void motorSpeed(double left, double right) {
		IO.DriveBase.MotorLeft[0]->Set(-left);
		IO.DriveBase.MotorRight[0]->Set(right);
	}

	int stopMotors() {
		motorSpeed(0, 0);
		return 1;
	}

	//------------- Start Code for Running Encoders --------------
	double readEncoder() {

		double usableEncoderData;
		double l = IO.DriveBase.EncoderLeft->GetDistance();
		double r = IO.DriveBase.EncoderRight->GetDistance();

		//If a encoder is disabled switch l or r to each other.
		if (l > 0) {
			usableEncoderData = fmax(r, l);
		} else if (l == 0) {
			usableEncoderData = r;
		} else {
			usableEncoderData = fmin(r, l);
		}
		return usableEncoderData;
	}

	void resetEncoder() {
		IO.DriveBase.EncoderLeft->Reset();
		IO.DriveBase.EncoderRight->Reset();
	}
	//------------- End Code for Running Encoders --------------------


	void AutonomousInit() {
		AutoProgram.Initalize();
	}

	void AutonomousPeriodic() {
		AutoProgram.Periodic();
	}



}
;

START_ROBOT_CLASS(Robot)
