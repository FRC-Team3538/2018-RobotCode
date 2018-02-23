#include <iostream>
#include <memory>
#include <string>
#include "math.h"
#include <algorithm>

// And So It Begins...
#include "RJ_RobotMap.h"

#define ElevDeadband (0.125)	// deadband for elevator gears and motors, value to move elevator up

class Robot: public frc::TimedRobot {

	// Robot Hardware Setup
	RJ_RobotMap IO;

	// Built-In Drive code for teleop
	DifferentialDrive Adrive { IO.DriveBase.MotorsLeft, IO.DriveBase.MotorsRight };

	// create pdp variable
	PowerDistributionPanel *pdp = new PowerDistributionPanel();

	// Override Elevator lower limit switch, upper limit switch, and elevator encoder
	bool ElevOverride = false;

	// Drive Input Filter
	float OutputX = 0.0, OutputY = 0.0;

	// Teleop Elevator Position
	double ElevPosTarget = 800;
	bool ElevatorSetFlag = true;

	// State Variables
	double ElevIError = 0;

	//Autonomous Variables
	int autoWaiting = 0;
	Timer AutonTimer, autoSettleTimer;
	std::string autoGameData, autoDelay, autoPosition, autoEncoder;
	int autoModeState;

	void RobotInit() {
		//disable drive watchdogs
		Adrive.SetSafetyEnabled(false);

		// Reset Encoders
		IO.DriveBase.EncoderLeft.Reset();
		IO.DriveBase.EncoderRight.Reset();

		// Zeros the NavX Yaw
		IO.DriveBase.ahrs.ZeroYaw();

		// Start Vision Thread
		std::thread visionThread(VisionThread);
		visionThread.detach();
	}

	void RobotPeriodic() {

		// Update Smart Dash
		SmartDashboardUpdate();
		//IO.NavXDebugger();

		// Get SmartDash Choosers
		autoDelay = IO.DS.chooseAutoDelay.GetSelected();
		autoPosition = IO.DS.chooseAutoProgram.GetSelected();
		autoEncoder = IO.DS.chooseAutoEncoder.GetSelected();

		// Get the game-specific message (ex: RLL)
		autoGameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		//VisionThread();
	}

	void DisabledPeriodic() {
		// NOP
	}

	void TeleopInit() {
		// drive command averaging filter
		OutputX = 0, OutputY = 0;

	}

	void TeleopPeriodic() {
		double Control_Deadband = 0.11; // input where the joystick actually starts to move
		double Drive_Deadband = 0.11; // command at which the motors begin to move

		// Drive Control Inputs
		double SpeedLinear = IO.DS.DriveStick.GetY(GenericHID::kLeftHand) * 1; // get Yaxis value (forward)
		double SpeedRotate = IO.DS.DriveStick.GetX(GenericHID::kRightHand) * -1; // get Xaxis value (turn)

		// Set dead band for control inputs
		SpeedLinear = deadband(SpeedLinear, Control_Deadband);
		SpeedRotate = deadband(SpeedRotate, Control_Deadband);

		// Smoothing algorithm for x^3
		if (SpeedLinear > 0)
			SpeedLinear = (1 - Drive_Deadband) * pow(SpeedLinear, 3) + Drive_Deadband;
		else
			SpeedLinear = (1 - Drive_Deadband) * pow(SpeedLinear, 3) - Drive_Deadband;

		// Limit max rotation speed for increased sensitivity
		SpeedRotate = 0.8 * SpeedRotate;

		// Moving Average Filter (Previous 5 commands are averaged together.)
		OutputY = (0.8 * OutputY) + (0.2 * SpeedLinear);
		OutputX = (0.8 * OutputX) + (0.2 * SpeedRotate);

		// Drive Code (WPI Built-in)
		Adrive.ArcadeDrive(OutputX, OutputY, true);

		// Drive Shifter Controls
		if (IO.DS.DriveStick.GetBumper(frc::GenericHID::kRightHand))
			IO.DriveBase.SolenoidShifter.Set(true); // High gear

		if (IO.DS.DriveStick.GetBumper(frc::GenericHID::kLeftHand))
			IO.DriveBase.SolenoidShifter.Set(false); // Low gear

		//  Rumble code
		//  Read all motor current from PDP and display on drivers station
		//double driveCurrent = pdp->GetTotalCurrent();	// Get total current
		double driveCurrent = pdp->GetTotalCurrent();

		// rumble if current to high
		double RbtThr = 0.0;		// Define value for total rumble current
		if (driveCurrent > 125.0)		// Rumble if greater than 125 amps motor current
			RbtThr = 1.0;

		IO.DS.DriveStick.SetRumble(Joystick::kLeftRumble, RbtThr); // Set Left Rumble to RbtThr
		IO.DS.DriveStick.SetRumble(Joystick::kRightRumble, RbtThr);	// Set Right Rumble to RbtThr

		/*
		 * MANIP CODE
		 */

		// reversing controller input so up gives positive input
		double ElevatorStick = IO.DS.OperatorStick.GetY(frc::XboxController::kLeftHand) * -1;
		ElevatorStick = deadband(ElevatorStick, Control_Deadband);

		// Smoothing algorithm for x^3
		// No reverse bias is needed thanks to gravity
		ElevatorStick = (1 - ElevDeadband) * pow(ElevatorStick, 3) + ElevDeadband;

		// Elevator Preset Positions [DPAD]
		switch (IO.DS.OperatorStick.GetPOV()) {
		case 270:
			// Dpad Left - Portal height
			ElevPosTarget = 10500;
			break;
		case 90:
			// Dpad Right - Switch height
			ElevPosTarget = 2600;
			break;
		case 180:
			// Dpad Down - ground/intake level
			ElevPosTarget = 1342;
			break;
		case 0:
			// Dpad  Up - Scale Position
			ElevPosTarget = 15000;
			break;
		}

		if (fabs(ElevatorStick) > 0.0) {
			// Manual control of Joystick
			elevatorSpeed(ElevatorStick);
			ElevPosTarget = IO.DriveBase.EncoderElevator.Get();
		} else if (!ElevOverride) {
			// Hold Current Position if Elevator Override = false
			elevatorPosition(ElevPosTarget);
		} else {
			// Stop elevator movement when Elevator Override = true;
			elevatorSpeed(0);
		}

		// Controller Rumble if the elevator motor current is high
		double elevCurrent_m1 = pdp->GetCurrent(8);
		double elevCurrent_m2 = pdp->GetCurrent(9);

		double EleThr = 0.0;		// Define value for elevator rumble current
		if (elevCurrent_m1 > 8.0 or elevCurrent_m2 > 8.0)
			EleThr = 1.0;

		IO.DS.OperatorStick.SetRumble(Joystick::kLeftRumble, EleThr); // Set Left Rumble to EleThr
		IO.DS.OperatorStick.SetRumble(Joystick::kRightRumble, EleThr); // Set Right Rumble to EleThr

		//
		// Wrist control
		//
		double OpRightTrigger = IO.DS.OperatorStick.GetTriggerAxis(frc::GenericHID::kRightHand);
		double OpLeftTrigger = IO.DS.OperatorStick.GetTriggerAxis(frc::GenericHID::kLeftHand);

		IO.DriveBase.Wrist1.Set(OpRightTrigger - OpLeftTrigger);

		//
		// Claw control
		//
		if (IO.DS.OperatorStick.GetAButton()) {
			// A Button - Loose Intake
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kOff);
			IO.DriveBase.ClawIntake1.Set(1);

		} else if (IO.DS.OperatorStick.GetBButton()) {
			// B Button - Forceful Eject
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward);
			IO.DriveBase.ClawIntake1.Set(-1);

		} else if (IO.DS.OperatorStick.GetXButton()) {
			// X Button - Tight Intake
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse);
			IO.DriveBase.ClawIntake1.Set(1);

		} else if (IO.DS.OperatorStick.GetYButton()) {
			// Y Button - Drop it like it's hot
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse);
			IO.DriveBase.ClawIntake1.Set(0);

		} else {
			// Default Hold Cube
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward);
			IO.DriveBase.ClawIntake1.Set(0);
		}

	}

	void AutonomousInit() {
		autoModeState = 1;
		autoWaiting = 0;

		AutonTimer.Reset();
		AutonTimer.Start();

		autoSettleTimer.Reset();
		autoSettleTimer.Start();

		// Encoder based auton
		IO.DriveBase.EncoderLeft.Reset();
		IO.DriveBase.EncoderRight.Reset();

		// Turn off drive motors
		IO.DriveBase.MotorsLeft.Set(0);
		IO.DriveBase.MotorsRight.Set(0);

		// Reset the navX heading
		IO.DriveBase.ahrs.ZeroYaw();

		// Force robot into low gear
		IO.DriveBase.SolenoidShifter.Set(false);

		// Shut the claw by default
		IO.DriveBase.ClawClamp.Set(DoubleSolenoid::Value::kForward);
		IO.DriveBase.ClawIntake1.Set(0);

	}

	// Reset all the stuff that needs to be reset at each state
	void autoNextState(int nextState) {
		autoModeState = nextState;
		AutonTimer.Reset();
		autoSettleTimer.Reset();
		IO.DriveBase.ahrs.ZeroYaw(); // TODO: DAW - I don't like this...
	}

	void AutonomousPeriodic() {

		// Delay our auton program if required
		if (autoDelay == IO.DS.sAutoDelay3 and AutonTimer.Get() < 3)
			return;
		if (autoDelay == IO.DS.sAutoDelay5 and AutonTimer.Get() < 5)
			return;

		// Select a Starting Location
		if (autoPosition == IO.DS.AutoCenterSpot) {
			// Start Center, score in switch
			autoCenter();
		}
		if (autoPosition == IO.DS.AutoLeftSpot) {
			// Start Left, score scale

		}

	}

	void autoCenter(void) {

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);

		bool SwitchLeft = (autoGameData[0] == 'L');
		bool SwitchRight = (autoGameData[0] == 'R');

		switch (autoModeState) {
		case 1:
			ElevPosTarget = 6500;
			IO.DriveBase.Wrist1.Set(0.35);

			if (autoForward(24))
				autoNextState(2);
			break;
		case 2:
			// Pick a direction based on FMS switch state
			if (SwitchLeft) {
				if (autoTurn(90))
					autoNextState(3);

			} else if (SwitchRight) {
				if (autoTurn(-90))
					autoNextState(3);
			}

			break;
		case 3:
			if (SwitchLeft) {
				if (autoForward(48))
					autoNextState(4);

			} else if (SwitchRight) {
				if (autoForward(48))
					autoNextState(4);
			}

			break;
		case 4:
			// Pick a direction based on FMS switch state
			if (SwitchLeft) {
				if (autoTurn(-90))
					autoNextState(5);

			} else if (SwitchRight) {
				if (autoTurn(90))
					autoNextState(5);
			}
			break;
		case 5:

			if (autoForward(36))
				autoNextState(6);
			break;
		case 6:
			AutonTimer.Reset();
			AutonTimer.Stop();
			stopMotors();

			IO.DriveBase.Wrist1.Set(0.2);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse);

			autoNextState(7);

			break;
		default:
			stopMotors();

		}

		return;
	}

	void motorSpeed(double leftMotor, double rightMotor) {
		IO.DriveBase.MotorsLeft.Set(leftMotor);
		IO.DriveBase.MotorsRight.Set(rightMotor);
	}

	void elevatorSpeed(double elevMotor) {
		bool ElevatorUpperLimit = IO.DriveBase.SwitchElevatorUpper.Get();
		bool ElevatorLowerLimit = IO.DriveBase.SwitchElevatorLower.Get();

		if (ElevatorLowerLimit == false) {
			IO.DriveBase.EncoderElevator.Reset(); // Reset encoder to 0
		}

		if ((!ElevatorUpperLimit) and (elevMotor > 0) and (!ElevOverride)) {
			IO.DriveBase.Elevator1.Set(0);
			IO.DriveBase.Elevator2.Set(0);
		} else if ((!ElevatorLowerLimit) and (elevMotor < 0) and (!ElevOverride)) {
			IO.DriveBase.Elevator1.Set(0);
			IO.DriveBase.Elevator2.Set(0);
		} else {
			IO.DriveBase.Elevator1.Set(elevMotor);
			IO.DriveBase.Elevator2.Set(elevMotor);
		}
		SmartDashboard::PutNumber("elevMotor", elevMotor);
	}

#define Elevator_MAXSpeed (0.70)
#define Elevator_KP (0.002)
#define ElevatorPositionTol (3)

	bool elevatorPosition(double Elev_position) {

		double ElevEncoderRead = IO.DriveBase.EncoderElevator.Get();
		double ElevError = ElevEncoderRead - Elev_position;
		double ElevCmd = ElevError * -Elevator_KP; // P term

		//Limit Elevator Max Speed
		if (ElevCmd > Elevator_MAXSpeed)
			ElevCmd = Elevator_MAXSpeed;
		if (ElevCmd < -Elevator_MAXSpeed)
			ElevCmd = -Elevator_MAXSpeed;

		// Check if we made it to the target
		if (fabs(ElevError) <= ElevatorPositionTol) {
			return true;
		} else
			elevatorSpeed(ElevCmd);
		return false;
	}

#define Wrist_MaxSpeed (1)
#define Wrist_Idle (.4)

	bool wristPosition(int position) {
// Controls the wrist position.
// for now it will send the wrist to position 1 or position 2 then return true when it is in that position
		int wristOutput;
		bool switchWrist1 = IO.DriveBase.SwitchWrist1.Get();
		bool switchWrist2 = IO.DriveBase.SwitchWrist1.Get();
		bool inCorrectPosition = false;
		switch (position) {
		case 1:
			///if the wrist is in position 1
			if (switchWrist1 == true) {
				//it is in position 1
				wristOutput = Wrist_Idle;
				inCorrectPosition = true;
			} else {
				// it isn't in position 1
				wristOutput = 1;
				inCorrectPosition = false;
			}
			break;
		case 2:
			if (switchWrist2 == true) {
				wristOutput = -Wrist_Idle;
				inCorrectPosition = true;
			} else {
				wristOutput = -1;
				inCorrectPosition = false;
			}
			break;
		case 0:
			inCorrectPosition = false;
			break;
		default:
			inCorrectPosition = false;
			break;
		}

		// Cap the speed to the maximum
		wristOutput = std::max(std::min(wristOutput, Wrist_MaxSpeed), -Wrist_MaxSpeed);

		IO.DriveBase.Wrist1.Set(wristOutput);

		return inCorrectPosition;

	}

// Drivetrain functions

	int stopMotors() {
		//sets motor speeds to zero
		motorSpeed(0, 0);
		return 1;
	}

	// Go AutoForward autonomously...
#define KP_LINEAR (0.27)
#define LINEAR_SETTLING_TIME (0.250)
#define LINEAR_MAX_DRIVE_SPEED (0.35)
#define LINEAR_TOLERANCE (0.125)
#define KP_ROTATION (0.017)
#define ROTATIONAL_SETTLING_TIME (0.5)

	int autoForward(double targetDistance) {

		// Inches per second-ish... (No encoder mode)
		double encoderDistance = AutonTimer.Get() * 50.0;
		if (encoderDistance > targetDistance)
			encoderDistance = targetDistance;

		// If an encoder is available, use it...
		double encoderLeft = IO.DriveBase.EncoderLeft.GetDistance();
		double encoderRight = IO.DriveBase.EncoderRight.GetDistance();

		if (autoEncoder == IO.DS.EncoderAuto) {
			// Automatically select the larger value (assume one was disconnected)
			if (fabs(encoderLeft) > fabs(encoderRight))
				encoderDistance = encoderLeft;
			else
				encoderDistance = encoderRight;

		} else if (autoEncoder == IO.DS.EncoderLeft)
			encoderDistance = encoderLeft;

		else if (autoEncoder == IO.DS.EncoderRight)
			encoderDistance = encoderRight;

		// Calculate motor power
		double encoderError = targetDistance - encoderDistance;
		double driveCommandLinear = encoderError * KP_LINEAR;

		// limit max drive speed to reduce slippage
		if (driveCommandLinear > LINEAR_MAX_DRIVE_SPEED)
			driveCommandLinear = LINEAR_MAX_DRIVE_SPEED;
		if (driveCommandLinear < -LINEAR_MAX_DRIVE_SPEED)
			driveCommandLinear = -LINEAR_MAX_DRIVE_SPEED;

		// Use Gyro to drive straight
		double gyroAngle = IO.DriveBase.ahrs.GetAngle();
		double driveCommandRotation = gyroAngle * KP_ROTATION;
		//calculates and sets motor speeds
		motorSpeed(driveCommandLinear + driveCommandRotation, driveCommandLinear - driveCommandRotation);

		// Allow robot to come to a stop after reaching target
		if (abs(encoderError) > LINEAR_TOLERANCE) {
			autoSettleTimer.Reset();
		} else if (autoSettleTimer.Get() > LINEAR_SETTLING_TIME) {
			return 1;
		}

		return 0;
	}

#define ROTATION_kP (-0.05)
#define ROTATION_TOLERANCE (2.0)

	int autoTurn(float targetYaw) {

		float currentYaw = IO.DriveBase.ahrs.GetAngle();
		float yawError = -targetYaw - currentYaw;
		float yawCommand = yawError * ROTATION_kP;

		// Limit max rotation speed
		if (yawCommand > 0.5)
			yawCommand = 0.5;
		if (yawCommand < -0.5)
			yawCommand = -0.5;

		// dooo it!
		motorSpeed(yawCommand, -yawCommand);

		// Allow for the robot to settle into position
		if (abs(yawError) > ROTATION_TOLERANCE) {
			autoSettleTimer.Reset();
		} else if (autoSettleTimer.Get() > ROTATIONAL_SETTLING_TIME) {
			return 1;
		}

		return 0;
	}

	int timedDrive(double driveTime, double leftMotorSpeed, double rightMotorSpeed) {
		float currentTime = AutonTimer.Get();
		if (currentTime < driveTime) {
			motorSpeed(leftMotorSpeed, rightMotorSpeed);
		} else {
			stopMotors();
			return 1;
		}
		return 0;
	}

	frc::Relay::Value LEDcontrol(int LEDcontrolcode) {
		int relayoutput;
		for (int i = 7; i > -1; i = i - 1) {
			relayoutput = (LEDcontrolcode & 00000011);

			switch (relayoutput) {
			case 0:
				return frc::Relay::Value::kOff;
				break;
			case 1:
				return frc::Relay::Value::kReverse;
				break;
			case 2:
				return frc::Relay::Value::kForward;
				break;
			case 3:
				return frc::Relay::Value::kOn;
				break;
			default:
				return frc::Relay::Value::kOn;

			}

		}

		IO.DriveBase.LED0.Set(Relay::Value::kOn);
	}

	void SmartDashboardUpdate() {

		// Auto State
		SmartDashboard::PutString("Auto Posn", autoPosition);
		SmartDashboard::PutNumber("Auto State (#)", autoModeState);
		SmartDashboard::PutNumber("Auto Timer (s)", AutonTimer.Get());

		// Drive Encoders
		SmartDashboard::PutNumber("Drive Encoder Left (RAW)", IO.DriveBase.EncoderLeft.GetRaw());
		SmartDashboard::PutNumber("Drive Encoder Left (Inches)", IO.DriveBase.EncoderLeft.GetDistance());

		SmartDashboard::PutNumber("Drive Encoder Right (RAW)", IO.DriveBase.EncoderRight.GetRaw());
		SmartDashboard::PutNumber("Drive Encoder Right (Inch)", IO.DriveBase.EncoderRight.GetDistance());

		// Elevator Encoders
		SmartDashboard::PutNumber("Elevator Encoder", IO.DriveBase.EncoderElevator.Get());

		// Gyro
		if (&IO.DriveBase.ahrs) {
			SmartDashboard::PutNumber("Gyro Angle", IO.DriveBase.ahrs.GetAngle());
		} else {
			SmartDashboard::PutNumber("Gyro Angle", 999);
		}

		// Game Specific Message
		SmartDashboard::PutString("autoGameData", autoGameData);

		// State Vars
		SmartDashboard::PutNumber("ElevPosTarget", ElevPosTarget);

		// Elevator Limit Switches
		SmartDashboard::PutBoolean("SwitchElevatorUpper", IO.DriveBase.SwitchElevatorUpper.Get());
		SmartDashboard::PutBoolean("SwitchElevatorLower", IO.DriveBase.SwitchElevatorLower.Get());
	}

	// Dead band function
	// Scales output to accommodate for the loss of the deadband region
	double deadband(double input, double minval) {

		// If less than deadband value, return zero
		if (fabs(input) < minval)
			return 0.0;

		// Transform input so that output has full range [0.0 - 1.0]
		if (input > 0)
			return input * (1 - minval) - minval;
		else
			return input * (1 - minval) + minval;

	}

	// Vision Thread
	// Runs in parallel to the main robot control program
	static void VisionThread() {
		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
		camera.SetVideoMode(cs::VideoMode::kMJPEG, 640, 480, 30);
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("RGB", 160, 120);
		outputStreamStd.SetVideoMode(cs::VideoMode::kGray, 160, 120, 30);
		cv::Mat source;
		cv::Mat output;

		while (true) {
			cvSink.GrabFrame(source);
			cvtColor(source, output, cv::COLOR_BGR2GRAY);
			outputStreamStd.PutFrame(output);

			// Sleep to limit loop rate
			std::chrono::milliseconds timespan(250);
			std::this_thread::sleep_for(timespan);
		}

	}
};

START_ROBOT_CLASS(Robot);

