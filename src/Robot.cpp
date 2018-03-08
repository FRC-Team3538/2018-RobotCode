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
	Timer AutonTimer, autoSettleTimer, autoTotalTime;
	std::string autoGameData, autoDelay, autoTarget, autoEncoder, autoPosition, autoFinisher;
	int autoModeState;  // current step in auto sequence
	double autoHeading; // current gyro heading to maintain

	// For Calculating Acceleration of the drive base
	double DriveEncLeftRatePrev = 0.0;
	double DriveEncRightRatePrev = 0.0;
	double DriveEncLeftAccel = 0.0;
	double DriveEncRightAccel = 0.0;

	void RobotInit() {
		//disable drive watchdogs
		Adrive.SetSafetyEnabled(false);

		// Reset Encoders
		IO.DriveBase.EncoderLeft.Reset();
		IO.DriveBase.EncoderRight.Reset();

		// Zeros the NavX Yaw
		IO.DriveBase.ahrs.ZeroYaw();
	}

	void RobotPeriodic() {

		// Update Smart Dash
		SmartDashboardUpdate();
		//IO.NavXDebugger();

		// Get SmartDash Choosers
		autoDelay = IO.DS.chooseAutoDelay.GetSelected();
		autoTarget = IO.DS.chooseAutoProgram.GetSelected();
		autoEncoder = IO.DS.chooseAutoEncoder.GetSelected();
		autoPosition = IO.DS.chooseAutoPosStart.GetSelected();
		autoFinisher = IO.DS.chooseAutoFinisher.GetSelected();

		// Calculate DriveBase Acceleration
		DriveEncLeftAccel = IO.DriveBase.EncoderLeft.GetRate() - DriveEncLeftRatePrev;
		DriveEncLeftRatePrev = IO.DriveBase.EncoderLeft.GetRate();
		DriveEncRightAccel = IO.DriveBase.EncoderRight.GetRate() - DriveEncRightRatePrev;
		DriveEncRightRatePrev = IO.DriveBase.EncoderRight.GetRate();

		// Get the game-specific message (ex: RLL)
		autoGameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		std::transform(autoGameData.begin(), autoGameData.end(), autoGameData.begin(), ::toupper);

		// Launchpad data
		IO.DS.LaunchPad.SetOutput(3, frc::DriverStation::GetInstance().IsAutonomous());
		IO.DS.LaunchPad.SetOutput(4, frc::DriverStation::GetInstance().IsOperatorControl());

		// Disable closed loop control and limit switches
		// Command Override level 9
		ElevOverride = IO.DS.LaunchPad.GetRawButton(1);
	}

	void DisabledPeriodic() {
		// NOP
	}

	void TeleopInit() {
		// drive command averaging filter
		OutputX = 0, OutputY = 0;
		elevatorSpeed(0);
		ElevPosTarget = 0;

		ElevPosTarget = IO.DriveBase.EncoderElevator.Get();

		IO.DriveBase.SolenoidShifter.Set(true); // Low gear
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
		if (SpeedLinear > 0.0)
			SpeedLinear = (1 - Drive_Deadband) * pow(SpeedLinear, 3) + Drive_Deadband;
		else
			SpeedLinear = (1 - Drive_Deadband) * pow(SpeedLinear, 3) - Drive_Deadband;

		// Smoothing algorithm for x^3
		if (SpeedRotate > 0.0)
			SpeedRotate = (1 - Drive_Deadband) * pow(SpeedRotate, 3) + Drive_Deadband;
		else
			SpeedRotate = (1 - Drive_Deadband) * pow(SpeedRotate, 3) - Drive_Deadband;

		// Moving Average Filter (Previous 5 commands are averaged together.)
		OutputY = (0.8 * OutputY) + (0.2 * SpeedLinear);
		OutputX = (0.8 * OutputX) + (0.2 * SpeedRotate);

		// Drive Code (WPI Built-in)
		Adrive.ArcadeDrive(OutputY, OutputX);

		// Drive Shifter Controls
		if (IO.DS.DriveStick.GetBumper(frc::GenericHID::kRightHand))
			IO.DriveBase.SolenoidShifter.Set(false); // High gear

		if (IO.DS.DriveStick.GetBumper(frc::GenericHID::kLeftHand))
			IO.DriveBase.SolenoidShifter.Set(true); // Low gear

		if (IO.DS.DriveStick.GetAButton()) {
			// A Button - Loose Intake
			IO.DriveBase.Zbar.Set(true);
		} else {
			IO.DriveBase.Zbar.Set(false);
		}

		if (IO.DS.DriveStick.GetBButton()) {
			// A Button - Loose Intake
			IO.DriveBase.Zbar1.Set(true);
		} else {
			IO.DriveBase.Zbar1.Set(false);
		}

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

		if (fabs(ElevatorStick) > Control_Deadband) {
			// Manual control of Joystick
			//ElevatorStick = (1 - ElevDeadband) * pow(ElevatorStick, 3) + ElevDeadband;

			// Smoothing algorithm for x^3
			if (ElevatorStick > 0.0)
				ElevatorStick = (1 - ElevDeadband) * pow(ElevatorStick, 3) + ElevDeadband;
			else
				ElevatorStick = (1 - ElevDeadband) * pow(ElevatorStick, 3) - ElevDeadband;

			elevatorSpeed(ElevatorStick);
			ElevPosTarget = IO.DriveBase.EncoderElevator.Get();

		} else if (!ElevOverride) {
			// Hold Current Position if Elevator Override = false
			elevatorPosition(ElevPosTarget);
		} else {
			// Stop elevator movement when Elevator Override = true;
			elevatorSpeed(0.0);
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
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward);
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

		// Reset Timers
		AutonTimer.Reset();
		AutonTimer.Start();

		autoSettleTimer.Reset();
		autoSettleTimer.Start();

		autoTotalTime.Reset();
		autoTotalTime.Start();

		// Encoder based auton
		IO.DriveBase.EncoderLeft.Reset();
		IO.DriveBase.EncoderRight.Reset();

		// Turn off drive motors
		IO.DriveBase.MotorsLeft.Set(0);
		IO.DriveBase.MotorsRight.Set(0);

		// Reset the navX heading
		IO.DriveBase.ahrs.ZeroYaw();
		autoHeading = 0.0;

		// Force robot into low gear
		IO.DriveBase.SolenoidShifter.Set(true);

		// Shut the claw by default
		IO.DriveBase.ClawClamp.Set(DoubleSolenoid::Value::kForward);
		IO.DriveBase.ClawIntake1.Set(0);

		OutputY = 0;
		OutputX = 0;
	}

	// Reset all the stuff that needs to be reset at each state
	void autoNextState() {
		autoModeState++;
		AutonTimer.Reset();
		autoSettleTimer.Reset();
		IO.DriveBase.EncoderLeft.Reset();
		IO.DriveBase.EncoderRight.Reset();

		stopMotors();

	}

	void AutonomousPeriodic() {

		// Delay our auton program if required
		if (autoDelay == IO.DS.sAutoDelay3 and AutonTimer.Get() < 3)
			return;
		if (autoDelay == IO.DS.sAutoDelay5 and AutonTimer.Get() < 5)
			return;

		// Select a Starting Location
		if (autoTarget == IO.DS.AutoLine) {
			autoLine();
		}
		if (autoTarget == IO.DS.AutoSwitch && autoPosition == IO.DS.sAutoCenter) {
			//autoCenter();

			if (autoGameData[0] == 'L')
				autoCenterFast(1);

			if (autoGameData[0] == 'R')
				autoCenterFast(-1);

		}
		//Left Side
		if (autoTarget == IO.DS.AutoSwitch && autoPosition == IO.DS.sAutoLeft) {
			autoSwitchSide(false);
		}
		//Right Side
		if (autoTarget == IO.DS.AutoSwitch && autoPosition == IO.DS.sAutoRight) {
			autoSwitchSide(true);
		}
		if (autoTarget == IO.DS.AutoScale && autoPosition == IO.DS.sAutoLeft) {
			//autoScale(false);

			if (autoGameData[1] == 'L')
				autoScaleFastNear(1);

			if (autoGameData[1] == 'R')
				autoScaleFastFar(1);
		}

		if (autoTarget == IO.DS.AutoScale && autoPosition == IO.DS.sAutoRight) {
			//autoScale(false);

			if (autoGameData[1] == 'L')
				autoScaleFastFar(-1);

			if (autoGameData[1] == 'R')
				autoScaleFastNear(-1);
		}
		//Start of adam logic for scale autos
		// Left Side
		if (autoTarget == IO.DS.AutoNearest && autoPosition == IO.DS.sAutoRight) {
			if (autoGameData[1] == 'R') {
				autoScaleFastNear(-1);

			} else if (autoGameData[1] == 'L') {

				if (autoGameData[0] == 'R')
					autoSwitchSide(true);

				else if (autoGameData[0] == 'L')
					autoLine();
			}

		}
		//Right Side
		if (autoTarget == IO.DS.AutoNearest && autoPosition == IO.DS.sAutoLeft) {
			if (autoGameData[1] == 'L') {
				autoScaleFastNear(1);

			} else if (autoGameData[1] == 'R') {

				if (autoGameData[0] == 'L')
					autoSwitchSide(false);

				else if (autoGameData[0] == 'R')
					autoLine();
			}
		}


		//end of adam screwing around

	}

	/*
	 * AUTO PROGRAM - CROSS THE LINE
	 *
	 * Start robot anywhere.
	 *
	 * The robot will simply cross the line.
	 * This is really just a test program...
	 */
	void autoLine(void) {

		switch (autoModeState) {
		case 1:
			if (autoForward(12.0 * 10))
				autoNextState();
			break;
			/*
			 case 2:
			 if (autoTurn(-90.0))
			 autoNextState();
			 break;

			 case 3:
			 if (autoForward(48.0))
			 autoNextState();
			 break;
			 */
		default:
			stopMotors();

		}

		return;
	}

	/*
	 * AUTO PROGRAM - CENTER SWITCH FAST
	 *
	 *
	 */
	void autoCenterFast(double direction) {

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);
		ElevPosTarget = 800;

		// High gear
		IO.DriveBase.SolenoidShifter.Set(false);

		switch (autoModeState) {
		case 1:

			if (autoForward(18))
				autoNextState();
			break;

		case 2:
			if (autoTurn(45.0 * direction))
				autoNextState();
			break;

		case 3:

			if (autoForward(60.0))
				autoNextState();
			break;

		case 4:
			if (autoTurn(0.0))
				autoNextState();
			break;

		case 5:
			if (timedDrive(1.5, 0.3, 0.3))
				//if (autoForward(20.0, 0.7))
				autoNextState();
			break;
		case 6:
			// Eject!
			IO.DriveBase.ClawIntake1.Set(-1.0);

			// keep pushing!
			if (timedDrive(1.0, 0.15, 0.15)) {
				IO.DriveBase.ClawIntake1.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());
			}

			break;
		default:
			stopMotors();

		}

		return;
	}

	void autoScaleFastNear(double direction) {

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);

		// High gear
		IO.DriveBase.SolenoidShifter.Set(false);

		switch (autoModeState) {
		case 1:
			// Auto home the elevator
			elevatorSpeed(-0.2);

			if (autoForward(300 - 24, 1.0, 0.1)) {
				autoNextState();
				ElevPosTarget = 16000;
			}
			break;

		case 2:
			if (autoTurn(-45.0 * direction) && elevatorPosition(ElevPosTarget)) {
				autoNextState();
			}
			break;

		case 3:
			// Eject!
			IO.DriveBase.ClawIntake1.Set(-1.0);

			// The Crowd Goes Wild!
			if (AutonTimer.Get() > 1.0) {
				IO.DriveBase.ClawIntake1.Set(0.0);
				autoNextState();
				ElevPosTarget = 800;

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());
			}

			break;

		default:
			stopMotors();

		}

		return;
	}

	void autoScaleFastFar(double direction) {

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);

		// High gear
		IO.DriveBase.SolenoidShifter.Set(false);

		switch (autoModeState) {
		case 1:
			// Auto home the elevator
			elevatorSpeed(-0.2);

			if (autoForward(226, 1.0, 0.1)) {
				autoNextState();
			}
			break;

		case 2:
			if (autoTurn(-90.0 * direction)) {
				autoNextState();
			}
			break;

		case 3:
			if (autoForward(186 + 18, 1.0, 0.1)) {
				autoNextState();
				ElevPosTarget = 16000;
			}
			break;

		case 4:
			if (autoTurn(0.0) && elevatorPosition(ElevPosTarget)) {
				autoNextState();
			}
			break;

		case 5:
			if (autoForward(24, 1.0, 0.1)) {
				autoNextState();
			}
			break;

		case 6:
			if (autoFinisher == IO.DS.sAutoYes)
				autoNextState();
			else
				autoModeState = 8;
			break;

		case 7:
			// Eject!
			IO.DriveBase.ClawIntake1.Set(-1.0);

			// The Crowd Goes Wild!
			if (AutonTimer.Get() > 1.0) {
				IO.DriveBase.ClawIntake1.Set(0.0);
				autoNextState();
				ElevPosTarget = 800;

			}

			break;

		case 8:
			stopMotors();

			// Display auton Time
			SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());

			autoNextState();

			break;

		default:
			stopMotors();

		}

		return;
	}

	/*
	 * AUTO PROGRAM - SIDE SWITCH
	 *
	 * Start robot in side of wall, with the corner of the robot touching the portal
	 *
	 * The robot will go the the proper side of the switch based on FMS data.
	 * But it will score on the side or rear of switch, so that we do not interfere with
	 * our alliance partners doing a center switch program.
	 *
	 * Input parameter is which side of the field the robot is starting on (left | right)
	 */
	void autoSwitchSide(bool isStartRightPos) {

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);

		// Determin which path to take
		bool SwitchNear;
		bool SwitchFar;
		double rotDir;

		if (isStartRightPos) {
			SwitchNear = (autoGameData[0] == 'R');
			SwitchFar = (autoGameData[0] == 'L');
			rotDir = -1.0;
		} else {
			SwitchNear = (autoGameData[0] == 'L');
			SwitchFar = (autoGameData[0] == 'R');
			rotDir = 1.0;
		}

		// Auto Sequence
		switch (autoModeState) {
		case 1:
			ElevPosTarget = 6500;
			IO.DriveBase.Wrist1.Set(-0.35);

			if (SwitchNear)
				if (autoForward(150))
					autoNextState();

			if (SwitchFar)
				if (autoForward(226))
					autoNextState();

			break;

		case 2:
			if (autoTurn(-90 * rotDir))
				autoNextState();
			break;

		case 3:
			if (SwitchNear)
				if (autoForward(18)) {
					autoNextState();
					autoModeState = 7; // Go To End
				}

			if (SwitchFar)
				if (autoForward(145))
					autoNextState();

			break;

		case 4:
			if (autoTurn((-90 - 25) * rotDir))
				autoNextState();
			break;

		case 5:
			if (autoForward(36.0))
				autoNextState();
			break;

		case 6:
			if (autoTurn(-180 * rotDir))
				autoNextState();
			break;

		case 7:  // dont forget to update step 3!!!!!
			//if (timedDrive(0.5, 0.8, 0.8))
			autoNextState();
			break;

		case 8:
			AutonTimer.Reset();
			AutonTimer.Stop();
			stopMotors();

			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse);
			IO.DriveBase.ClawIntake1.Set(-1);

			autoNextState();

			break;
		default:
			stopMotors();

		}

		return;
	}

	/*
	 * AUTO PROGRAM -  SCALE
	 *
	 * Start robot in side of wall, with the corner of the robot touching the portal
	 *
	 * The robot will go the the proper side of the scale based on FMS data.
	 * But it will score on the side or front of switch, so that we do not interfere with
	 * our alliance partners doing the same program
	 *
	 * Input parameter is which side of the field the robot is starting on (left | right)
	 */
	void autoScale(bool isRightSide) {

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);
		double elevatorPreset = 6500;
		//double elevatorPreset = 17500;
		double elevError;

		bool targetNear;
		bool targetFar;
		double rotDir;

		if (isRightSide) {
			targetNear = (autoGameData[1] == 'R');
			targetFar = (autoGameData[1] == 'L');
			rotDir = -1.0;
		} else {
			targetNear = (autoGameData[1] == 'L');
			targetFar = (autoGameData[1] == 'R');
			rotDir = 1.0;
		}

		switch (autoModeState) {
		case 1:

			if (targetNear)
				if (autoForward(304))
					autoNextState();

			if (targetFar)
				if (autoForward(226))
					autoNextState();

			break;

		case 2:

			if (targetNear) {
				IO.DriveBase.Wrist1.Set(-0.35);
				ElevPosTarget = elevatorPreset;
				elevError = fabs(IO.DriveBase.EncoderElevator.Get() - ElevPosTarget);
				if (autoTurn(-90 * rotDir) && elevError < 100)
					autoNextState();
			}

			if (targetFar)
				if (autoTurn(-90 * rotDir))
					autoNextState();
			break;

		case 3:
			if (targetNear)
				if (autoForward(6)) {
					autoNextState();
					autoModeState = 6; // Go To End
				}

			if (targetFar)
				if (autoForward(186))
					autoNextState();

			break;

		case 4:
			IO.DriveBase.Wrist1.Set(-0.35);
			ElevPosTarget = elevatorPreset;
			elevError = fabs(IO.DriveBase.EncoderElevator.Get() - ElevPosTarget);

			if (autoTurn(0) && elevError < 100)
				autoNextState();
			break;

		case 5:
			if (autoForward(24.0))
				autoNextState();
			break;

		case 6: // dont forget to update step 3!!!!!
			AutonTimer.Reset();
			AutonTimer.Stop();
			stopMotors();

			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse);
			IO.DriveBase.ClawIntake1.Set(-1);

			autoNextState();

			break;
		default:
			stopMotors();

		}

		return;

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
	}

#define Elevator_MAXSpeed (1.0)
#define Elevator_KP (0.002)
#define ElevatorPositionTol (60)

	bool elevatorPosition(double Elev_position) {

		// Get Current Encoder Value
		double ElevEncoderRead = IO.DriveBase.EncoderElevator.Get();

		// Anti-bounce
		bool ElevatorUpperLimit = IO.DriveBase.SwitchElevatorUpper.Get();
		if ((!ElevatorUpperLimit) and (!ElevOverride))
			Elev_position = ElevEncoderRead - 100;

		// Motor Command Calculation
		double ElevError = ElevEncoderRead - Elev_position;
		double ElevCmd = ElevError * -Elevator_KP; // P term

		//Limit Elevator Max Speed
		ElevCmd = absMax(ElevCmd, Elevator_MAXSpeed);

		SmartDashboard::PutNumber("Elev error", fabs(ElevError));

		elevatorSpeed(ElevCmd);

		// Check if we made it to the target
		return (fabs(ElevError) <= ElevatorPositionTol);
	}

// Drivetrain functions

	void motorSpeed(double leftMotor, double rightMotor) {

		// Moving Average Filter
		double cycles = 0.2;
		OutputY = (cycles * OutputY) + (1.0 - cycles) * leftMotor;
		OutputX = (cycles * OutputX) + (1.0 - cycles) * rightMotor;

		IO.DriveBase.MotorsLeft.Set(-OutputY);
		IO.DriveBase.MotorsRight.Set(OutputX);
	}

	int stopMotors() {
		//sets motor speeds to zero
		motorSpeed(0, 0);
		return 1;
	}

	// Go AutoForward autonomously...
#define KP_LINEAR (0.056)
#define LINEAR_TOLERANCE (1.0)

#define ROTATION_kP (0.04)
#define ROTATION_TOLERANCE (10.0)
#define ROTATIONAL_SETTLING_TIME (0.0)
#define ROTATIONAL_MAX_SPEED (0.40)

	int autoForward(double targetDistance, double max_speed, double settle_time) {

		double encoderDistance = getEncoderDistance();

		// Calculate motor power
		double encoderError = targetDistance - encoderDistance;
		double driveCommandLinear = encoderError * KP_LINEAR;

		// limit max drive speed
		driveCommandLinear = absMax(driveCommandLinear, max_speed);

		// Min speed
		driveCommandLinear = absMin(driveCommandLinear, 0.11);

		// Use Gyro to drive straight
		double gyroAngle = IO.DriveBase.ahrs.GetAngle();
		double driveCommandRotation = (gyroAngle - autoHeading) * ROTATION_kP;
		driveCommandRotation = absMax(driveCommandRotation, ROTATIONAL_MAX_SPEED);

		// Do iiiiit!
		motorSpeed(driveCommandLinear - driveCommandRotation, driveCommandLinear + driveCommandRotation);

		// Allow robot to come to a stop after reaching target
		if (abs(encoderError) > LINEAR_TOLERANCE) {
			autoSettleTimer.Reset();

		} else if (autoSettleTimer.Get() > settle_time)
			return 1;

		return 0;
	}

	// Overload for backwards compatibility
	int autoForward(double targetDistance) {
		return autoForward(targetDistance, 1.0, 0.0);
	}

	int autoTurn(float targetYaw) {

		// For linear drive function
		autoHeading = -targetYaw;

		// Calculate motor command
		float currentYaw = IO.DriveBase.ahrs.GetAngle();
		float yawError = -targetYaw - currentYaw;
		float yawCommand = yawError * -ROTATION_kP;

		// Limit max rotation speed
		yawCommand = absMax(yawCommand, ROTATIONAL_MAX_SPEED);

		// dooo it!
		motorSpeed(-yawCommand, yawCommand);

		// Allow for the robot to settle into position
		if (abs(yawError) > ROTATION_TOLERANCE)
			autoSettleTimer.Reset();

		else if (autoSettleTimer.Get() > ROTATIONAL_SETTLING_TIME)
			return 1;

		return 0;
	}

	int timedDrive(double driveTime, double leftMotorSpeed, double rightMotorSpeed) {

		if (AutonTimer.Get() < driveTime) {

			// Use Gyro to drive straight
			double gyroAngle = IO.DriveBase.ahrs.GetAngle();
			double driveCommandRotation = (gyroAngle - autoHeading) * ROTATION_kP;
			driveCommandRotation = absMax(driveCommandRotation, ROTATIONAL_MAX_SPEED);

			motorSpeed(leftMotorSpeed - driveCommandRotation, rightMotorSpeed + driveCommandRotation);

		} else {

			stopMotors();
			return 1;
		}
		return 0;
	}

	// Gets the encoder distance since last reset
	// Algorithm selected by the dashboard chooser
	double getEncoderDistance() {

		// Inches per second-ish... (No encoder mode)
		double encoderDistance = 0; // = AutonTimer.Get() * 48.0;

		// If an encoder is available, use it...
		double encoderLeft = IO.DriveBase.EncoderLeft.GetDistance();
		double encoderRight = IO.DriveBase.EncoderRight.GetDistance();

		if (autoEncoder == IO.DS.EncoderAuto) {
			// Automatically select the larger value (assume one was disconnected)
			if (fabs(encoderLeft) > fabs(encoderRight))
				encoderDistance = encoderLeft;
			else
				encoderDistance = encoderRight;
		}

		if (autoEncoder == IO.DS.EncoderBoth)
			encoderDistance = (encoderLeft + encoderRight) / 2.0;

		if (autoEncoder == IO.DS.EncoderLeft)
			encoderDistance = encoderLeft;

		if (autoEncoder == IO.DS.EncoderRight)
			encoderDistance = encoderRight;

		return encoderDistance;

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

	// Dead band function
	// Scales output to accommodate for the loss of the deadband region
	double deadband(double input, double minval) {

		// If less than deadband value, return zero
		if (fabs(input) < minval)
			return 0.0;

		// Transform input so that output has full range [0.0 - 1.0]
		if (input < 0.0)
			return input * (1 - minval) - minval;
		else
			return input * (1 - minval) + minval;

	}

	// Limits absolute value of input
	double absMax(double input, double maxval) {

		// Just in case the max is negative.
		maxval = fabs(maxval);

		// If out of bounds, return max
		if (fabs(input) > maxval) {

			if (input > 0.0)
				return maxval;
			else
				return -maxval;
		}

		// Seems good
		return input;
	}

	// Limits absolute value of input
	double absMin(double input, double minval) {

		// Just in case the max is negative.
		minval = fabs(minval);

		// If out of bounds, return max
		if (fabs(input) < minval) {

			if (input > 0.0)
				return minval;
			else
				return -minval;
		}

		// Seems good
		return input;
	}

	void SmartDashboardUpdate() {

		// Motor Outputs
		SmartDashboard::PutNumber("Drive Left (PWM)", IO.DriveBase.MotorsLeft.Get());
		SmartDashboard::PutNumber("Drive Right (PWM)", IO.DriveBase.MotorsRight.Get());

		SmartDashboard::PutNumber("Elev PWM", IO.DriveBase.Elevator2.Get());

		// Auto State
		SmartDashboard::PutString("Auto Target", autoTarget);
		SmartDashboard::PutString("Auto Position", autoPosition);
		SmartDashboard::PutString("Auto Finisher", autoFinisher);
		SmartDashboard::PutNumber("Auto State (#)", autoModeState);
		SmartDashboard::PutNumber("Auto Timer (s)", AutonTimer.Get());
		SmartDashboard::PutNumber("Auto Heading", autoHeading);

		// Drive Encoders
		SmartDashboard::PutNumber("Drive Encoder Left (RAW)", IO.DriveBase.EncoderLeft.GetRaw());
		SmartDashboard::PutNumber("Drive Encoder Left (Inches)", IO.DriveBase.EncoderLeft.GetDistance());

		SmartDashboard::PutNumber("Drive Encoder Right (RAW)", IO.DriveBase.EncoderRight.GetRaw());
		SmartDashboard::PutNumber("Drive Encoder Right (Inch)", IO.DriveBase.EncoderRight.GetDistance());

		// Elevator Encoders
		SmartDashboard::PutNumber("Elevator Encoder [RAW]", IO.DriveBase.EncoderElevator.Get());
		SmartDashboard::PutNumber("Elevator Encoder [INCH]", IO.DriveBase.EncoderElevator.GetDistance());

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

		// Sensor Override
		SmartDashboard::PutBoolean("Elevator Override", ElevOverride);

	}

};

START_ROBOT_CLASS(Robot);

