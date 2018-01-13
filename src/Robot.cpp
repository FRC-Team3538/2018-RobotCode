//2018-RobotCode
#include <iostream>
#include <memory>
#include <string>
#include "AHRS.h"
#include "WPILib.h"
#include "math.h"
#include "Encoder.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

// Time forward: go forward for 0.75 seconds.
#define RED_2_CASE2_FWD (71.0)
#define RED_2_CASE3_TIME (1.0)
#define RED_2_CASE3_LSPEED (-0.2)
#define RED_2_CASE3_RSPEED (-0.2)
//linear calibrations
// tolerance in inches
#define LINEAR_TOLERANCE (0.2)
// This is the gain for using the encoders to set the distance
//		while going straight.
#define KP_LINEAR (0.27)
// This is the gain for using the gyroscope to go straight
#define KP_ROTATION (0.017)
#define LINEAR_SETTLING_TIME (0.1)
#define LINEAR_MAX_DRIVE_SPEED (0.75)

//turning calibrations
#define ROTATIONAL_TOLERANCE (1.0)
// This is the gain for turning using the gyroscope
#define ERROR_GAIN (-0.05)
#define ROTATIONAL_SETTLING_TIME (0.5)

//encoder max drive time
#define MAX_DRIVE_TIME (3.0)

class Robot: public frc::IterativeRobot {
	RobotDrive Adrive;
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooseAutonSelector, chooseDriveEncoder,
			chooseKicker, chooseShooter;
	const std::string RH_Encoder = "RH_Encoder";
	const std::string LH_Encoder = "LH_Encoder";
	std::string autoSelected, encoderSelected;
	Joystick Drivestick;
	Joystick OperatorStick;
	VictorSP DriveLeft0;
	VictorSP DriveLeft1;
	VictorSP DriveLeft2;
	VictorSP DriveRight0;
	VictorSP DriveRight1;
	VictorSP DriveRight2;
	VictorSP Dpad1;
	VictorSP Dpad2;
	VictorSP RightStick1;
	VictorSP RightStick2;
	Timer AutonTimer;
	Timer EncoderCheckTimer;
	Encoder EncoderLeft;
	Encoder EncoderRight;

	float OutputX, OutputY;
	DigitalInput DiIn8, DiIn9;

	AHRS *ahrs;
//tells us what state we are in in each auto mode
	int modeState;
	bool AutonOverride;
	int AutoVal;
	int isWaiting = 0;			/////***** Divide this into 2 variables.

	// create pdp variable
	PowerDistributionPanel *pdp = new PowerDistributionPanel();

//Solenoid's declared
	Solenoid *driveSolenoid = new Solenoid(0);
	Solenoid *XYbutton = new Solenoid(4);
	Solenoid *Bbutton = new Solenoid(3);
	Solenoid *Abutton = new Solenoid(2);
	Solenoid *IntakeButton = new Solenoid(1);

	bool useRightEncoder = false;
	bool driveRightTriggerPrev = false;
	bool driveButtonYPrev = false;
	bool operatorRightTriggerPrev = false;
	bool intakeDeployed = false;
	bool XYDeployed = false;
	bool shooterOn = false;

public:
	Robot() :
			Adrive(DriveLeft0, DriveRight0), Drivestick(0), OperatorStick(1), DriveLeft0(
					0), DriveLeft1(1), DriveLeft2(2), DriveRight0(3), DriveRight1(
					4), DriveRight2(5), Dpad1(8), Dpad2(9), RightStick1(6), RightStick2(
					7), EncoderLeft(0, 1), EncoderRight(2, 3), OutputX(0), OutputY(
					0), DiIn8(8), DiIn9(9), ahrs(0), modeState(0), AutonOverride(
					0), AutoVal(0) {

	}

private:
	void RobotInit() {
		chooseDriveEncoder.AddDefault(LH_Encoder, LH_Encoder);
		chooseDriveEncoder.AddObject(RH_Encoder, RH_Encoder);
		frc::SmartDashboard::PutData("Encoder", &chooseDriveEncoder);

		//turn off shifter solenoids
		driveSolenoid->Set(false);

		//disable drive watchdogs
		Adrive.SetSafetyEnabled(false);

		//changes these original negative values to positive values
		EncoderLeft.SetReverseDirection(true);
		EncoderRight.SetReverseDirection(false);

		//calibrations for encoders
		EncoderLeft.SetDistancePerPulse(98.0 / 3125.0 * 4.0);
		EncoderRight.SetDistancePerPulse(98.0 / 3125.0 * 4.0);

		//drive command averaging filter
		OutputX = 0, OutputY = 0;

		//variable that chooses which encoder robot is reading for autonomous mode
		useRightEncoder = true;
	}

	void AutonomousInit() override {
		modeState = 1;
		isWaiting = 0;							/////***** Rename this.

		AutonTimer.Reset();
		AutonTimer.Start();
		EncoderCheckTimer.Reset();
		EncoderCheckTimer.Start();

		// Encoder based auton
		resetEncoder();

		// Turn off drive motors
		DriveLeft0.Set(0);
		DriveLeft1.Set(0);
		DriveLeft2.Set(0);
		DriveRight0.Set(0);
		DriveRight1.Set(0);
		DriveRight2.Set(0);

		//zeros the navX
		if (ahrs) {
			ahrs->ZeroYaw();
		}

		//forces robot into low gear
		driveSolenoid->Set(false);

	}

	void TeleopInit() {
		OutputX = 0, OutputY = 0;

	}

	void RobotPeriodic() {
		//links multiple motors together
		DriveLeft1.Set(DriveLeft0.Get());
		DriveLeft2.Set(DriveLeft0.Get());
		DriveRight1.Set(DriveRight0.Get());
		DriveRight2.Set(DriveRight0.Get());

		// Encoder Selection for autotools
		encoderSelected = chooseDriveEncoder.GetSelected();
		useRightEncoder = (encoderSelected == RH_Encoder);

		// Select Auto Program
		autoSelected = chooseAutonSelector.GetSelected();

	}

	void DisabledPeriodic() {

	}

	void AutonomousPeriodic() {
		autoForward();
	}

	void TeleopPeriodic() {
		double Deadband = 0.11;
		double DPadSpeed = 1.0;
		bool RightStickLimit1 = DiIn8.Get();
		bool RightStickLimit2 = DiIn9.Get();

		//high gear & low gear controls
		if (Drivestick.GetRawButton(5))
			driveSolenoid->Set(true);			// High gear press LH bumper
		if (Drivestick.GetRawButton(6))
			driveSolenoid->Set(false);			// Low gear press RH bumper

		//  Rumble code
		//  Read all motor current from PDP and display on drivers station
		double driveCurrent = pdp->GetTotalCurrent();	// Get total current

		// rumble if current to high
		double LHThr = 0.0;		// Define value for rumble
		if (driveCurrent > 125.0)// Rumble if greater than 125 amps motor current
			LHThr = 0.5;
		Joystick::RumbleType Vibrate;				// define Vibrate variable
		Vibrate = Joystick::kLeftRumble;		// set Vibrate to Left
		Drivestick.SetRumble(Vibrate, LHThr);  	// Set Left Rumble to RH Trigger
		Vibrate = Joystick::kRightRumble;		// set vibrate to Right
		Drivestick.SetRumble(Vibrate, LHThr);// Set Right Rumble to RH Trigger

		//drive controls
		double SpeedLinear = Drivestick.GetRawAxis(1) * 1; // get Yaxis value (forward)
		double SpeedRotate = Drivestick.GetRawAxis(4) * -1; // get Xaxis value (turn)

		// Set dead band for X and Y axis
		if (fabs(SpeedLinear) < Deadband)
			SpeedLinear = 0.0;
		if (fabs(SpeedRotate) < Deadband)
			SpeedRotate = 0.0;

		//slow down direction changes from 1 cycle to 5
		OutputY = (0.8 * OutputY) + (0.2 * SpeedLinear);
		OutputX = (0.8 * OutputX) + (0.2 * SpeedRotate);

		//drive
		if (Drivestick.GetRawButton(4)) {
			//boiler auto back up when y button pushed
			if (!driveButtonYPrev) {
				EncoderRight.Reset();
				EncoderLeft.Reset();
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
		Abutton->Set(OperatorStick.GetRawButton(1));

		//B Button to extend (Solenoid On)
		Bbutton->Set(OperatorStick.GetRawButton(2));

		//if Left Bumper button pressed, extend (Solenoid On)
		if (OperatorStick.GetRawButton(5)) {
			intakeDeployed = true;
			IntakeButton->Set(intakeDeployed);
		}

		//else Right Bumper pressed, retract (Solenoid Off)
		else if (OperatorStick.GetRawButton(6)) {
			intakeDeployed = false;
			IntakeButton->Set(intakeDeployed);
		}
		//if 'X' button pressed, extend (Solenoid On)
		if (OperatorStick.GetRawButton(3)) {
			XYDeployed = true;
			XYbutton->Set(XYDeployed);
		}

		//else 'Y' button pressed, retract (Solenoid Off)
		else if (OperatorStick.GetRawButton(4)) {
			XYDeployed = false;
			XYbutton->Set(XYDeployed);
		}

		//dpad POV stuff
		if (OperatorStick.GetPOV(0) == 0) {
			Dpad1.Set(DPadSpeed);
			Dpad2.Set(DPadSpeed);
		} else if (OperatorStick.GetPOV(0) == 180) {
			Dpad1.Set(-DPadSpeed);
			Dpad2.Set(-DPadSpeed);
		} else {
			Dpad1.Set(0.0);
			Dpad2.Set(0.0);
		}

		double RightSpeed = OperatorStick.GetRawAxis(4) * -1; // get Xaxis value for Right Joystick

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

	int forward(double targetDistance) {
		double encoderDistance = readEncoder();
		double encoderError = encoderDistance - targetDistance;
		double driveCommandLinear = encoderError * KP_LINEAR;
		//limits max drive speed
		if (driveCommandLinear > LINEAR_MAX_DRIVE_SPEED) {
			driveCommandLinear = LINEAR_MAX_DRIVE_SPEED;
		} else if (driveCommandLinear < -1 * LINEAR_MAX_DRIVE_SPEED) { /////***** "-1" is a "magic number." At least put a clear comment in here.
			driveCommandLinear = -1 * LINEAR_MAX_DRIVE_SPEED;
		}
		//gyro values that make the robot drive straight
		double gyroAngle = ahrs->GetAngle();
		double driveCommandRotation = gyroAngle * KP_ROTATION;
		//encdoer check
		if (EncoderCheckTimer.Get() > MAX_DRIVE_TIME) {
			motorSpeed(0.0, 0.0);
			DriverStation::ReportError(
					"(forward) Encoder Max Drive Time Exceeded");
		} else {
			//calculates and sets motor speeds
			motorSpeed(driveCommandLinear + driveCommandRotation,
					driveCommandLinear - driveCommandRotation);
		}
		//routine helps prevent the robot from overshooting the distance
		if (isWaiting == 0) {
			if (abs(encoderError) < LINEAR_TOLERANCE) {
				isWaiting = 1;
				AutonTimer.Reset();
			}
		}
		//timed wait
		else {
			float currentTime = AutonTimer.Get();
			if (abs(encoderError) > LINEAR_TOLERANCE) {
				isWaiting = 0;
			} else if (currentTime > LINEAR_SETTLING_TIME) {
				isWaiting = 0;
				return 1;
			}
		}
		return 0;
	}

	//need to change signs!!!
	int timedDrive(double driveTime, double leftMotorSpeed,
			double rightMotorSpeed) {
		float currentTime = AutonTimer.Get();
		if (currentTime < driveTime) {
			motorSpeed(leftMotorSpeed, rightMotorSpeed);
		} else {
			stopMotors();
			return 1;
		}
		return 0;
	}

#define AR2_INIT 1
#define AR2_FWD 2
#define AR2_TIMED 3
#define AR2_END 4
	void autoForward(void) {
		//change to timed drive
		//puts gear on front of airship
		switch (modeState) {
		case AR2_INIT:
			// This uses state 1 for initialization.
			// This keeps the initialization and the code all in one place.
			ahrs->ZeroYaw();
			resetEncoder();
			modeState = AR2_FWD;
			break;
		case AR2_FWD:
			if (forward(RED_2_CASE2_FWD)) {
				AutonTimer.Reset();
				modeState = AR2_TIMED;
			}
			break;
		case AR2_TIMED:
			if (timedDrive(RED_2_CASE2_FWD, RED_2_CASE3_LSPEED,
			RED_2_CASE3_RSPEED)) {
				AutonTimer.Reset();
				modeState = AR2_END;
			}
			break;
		default:
			stopMotors();
		}
		return;
	}

	void motorSpeed(double leftMotor, double rightMotor) {
		DriveLeft0.Set(leftMotor * -1);
		DriveLeft1.Set(leftMotor * -1);
		DriveLeft2.Set(leftMotor * -1);
		DriveRight0.Set(rightMotor);
		DriveRight1.Set(rightMotor);
		DriveRight2.Set(rightMotor);
	}

//need to change signs!!!
	int stopMotors() {
		//sets motor speeds to zero
		motorSpeed(0, 0);
		return 1;
	}

	//------------- Start Code for Running Encoders --------------
	double readEncoder() {
		double usableEncoderData;
		double r = EncoderRight.GetDistance();
		double l = EncoderLeft.GetDistance();
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
		EncoderLeft.Reset();
		EncoderRight.Reset();
		EncoderCheckTimer.Reset();
	}
	//------------- End Code for Running Encoders --------------------


private:

}
;

START_ROBOT_CLASS(Robot)
