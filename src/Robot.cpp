//Windsor Code 2018

#include <iostream>
#include <memory>
#include <string>
#include "math.h"
#include <algorithm>

// And So It Begins...
#include "RJ_RobotMap.h"

#define MAIN_LOOP_PERIOD (0.020)

#define ElevDeadband (0.125)	// deadband for elevator gears and motors, value to move elevator up

class Robot: public frc::TimedRobot {

	// Robot Hardware Setup
	RJ_RobotMap IO;

	// Built-In Drive code for teleop
	DifferentialDrive Adrive { IO.DriveBase.MotorsLeft, IO.DriveBase.MotorsRight };
	bool gearState; // For power-breaking feature

	// create pdp variable
	PowerDistributionPanel *pdp = new PowerDistributionPanel();

	// Override Elevator lower limit switch, upper limit switch, and elevator encoder
	bool SensorOverride = false;

	// Drive Input Filter
	float OutputX = 0.0, OutputY = 0.0;

	// Elevator Position
	double ElevPosTarget = 800;
	bool ElevatorSetFlag = true;

	// Teleop Elevator Position
	double WristTarget = 0.0;

	//Autonomous Variables
	Timer AutonTimer, autoSettleTimer, autoTotalTime;
	std::string autoGameData, autoDelay, autoTarget, autoEncoder, autoPosition, autoFinisher;
	int autoModeState;  // current step in auto sequence
	double autoHeading; // current gyro heading to maintain
	bool auto2CubeStartRight = false; // What side of the field is the 2cube starting

	// Status of Auto and Teleop
	bool AutoStateCheck = false;
	bool TeleopStateCheck = false;

	void RobotInit() {
		//disable drive watchdogs
		Adrive.SetSafetyEnabled(false);

		// Reset Encoders
		IO.DriveBase.EncoderLeft.Reset();
		IO.DriveBase.EncoderRight.Reset();

		// Zeros the NavX Yaw
		IO.DriveBase.ahrs.ZeroYaw();

		// 20ms is the default, but lets enforce it.
		this->SetPeriod(MAIN_LOOP_PERIOD);
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

		// Get the game-specific message (ex: RLL)
		autoGameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		std::transform(autoGameData.begin(), autoGameData.end(), autoGameData.begin(), ::toupper);

		// Launchpad data
		IO.DS.LaunchPad.SetOutput(3, AutoStateCheck);
		IO.DS.LaunchPad.SetOutput(4, TeleopStateCheck);

		// Disable closed loop control and limit switches
		//ElevOverride = IO.DS.LaunchPad.GetRawButton(1);
		if (IO.DS.OperatorStick.GetStartButton())
			SensorOverride = false;
		if (IO.DS.OperatorStick.GetBackButton())
			SensorOverride = true;

		// If the lower limit switch is hit reset the encoder to 0
		if (!IO.DriveBase.SwitchElevatorLower.Get()) {
			IO.DriveBase.EncoderElevator.Reset();
		}

		if (IO.DS.OperatorStick.GetStartButton())
			SensorOverride = false;
		if (IO.DS.OperatorStick.GetBackButton())
			SensorOverride = true;

	}

	void DisabledPeriodic() {
		// NOP
		// Set Auto and Teleop state to false
		AutoStateCheck = false;
		TeleopStateCheck = false;
	}

	void TeleopInit() {
		// drive command averaging filter
		OutputX = 0, OutputY = 0;

		// Hold current elevator position
		ElevPosTarget = IO.DriveBase.EncoderElevator.Get();
		WristTarget = IO.DriveBase.WristPot.Get();

		// Low Gear by default
		IO.DriveBase.SolenoidShifter.Set(true);
		gearState = true;

		// turn on teleOp State to true
		TeleopStateCheck = true;

	}

	void TeleopPeriodic() {
		double Control_Deadband = 0.11; // input where the joystick actually starts to move
		double Drive_Deadband = 0.11; // command at which the motors begin to move

		// Drive Control Inputs
		double SpeedLinear = IO.DS.DriveStick.GetY(GenericHID::kLeftHand) * 1; // get Yaxis value (forward)
		double SpeedRotate = IO.DS.DriveStick.GetX(GenericHID::kRightHand) * -1; // get Xaxis value (turn)

		// Power Brake
		bool bPowerBrake = (fabs(IO.DS.DriveStick.GetTriggerAxis(frc::GenericHID::kRightHand)) > Drive_Deadband);

		//Swap below deadband
		// Bad joystick compensation. :)
		SpeedLinear *= 1.05;
		SpeedRotate *= 1.05;

		//Swap Above ^^^^
		// Set dead band for control inputs
		SpeedLinear = deadband(SpeedLinear, Control_Deadband);
		SpeedRotate = deadband(SpeedRotate, Control_Deadband);

		// Smoothing algorithm for x^3
		if (SpeedLinear > 0.0)
			SpeedLinear = (1 - Drive_Deadband) * pow(SpeedLinear, 3) + Drive_Deadband;
		else if (SpeedLinear < 0.0)
			SpeedLinear = (1 - Drive_Deadband) * pow(SpeedLinear, 3) - Drive_Deadband;
		else
			SpeedLinear = 0.0;  // added for clarity

		// Smoothing algorithm for x^3
		if (SpeedRotate > 0.0)
			SpeedRotate = (1 - Drive_Deadband) * pow(SpeedRotate, 3) + Drive_Deadband;
		else if (SpeedRotate < 0.0)
			SpeedRotate = (1 - Drive_Deadband) * pow(SpeedRotate, 3) - Drive_Deadband;
		else
			SpeedRotate = 0.0; // added for clarity

		// Drive Shifter Controls
		if (IO.DS.DriveStick.GetBumper(frc::GenericHID::kRightHand))
			gearState = false; // High

		if (IO.DS.DriveStick.GetBumper(frc::GenericHID::kLeftHand))
			gearState = true; // Low

		// Power Brake
		if (bPowerBrake) {
			SpeedLinear *= 0.4;
			IO.DriveBase.SolenoidShifter.Set(true);
		} else {
			IO.DriveBase.SolenoidShifter.Set(gearState);
		}

		// Moving Average Filter (Previous 5 commands are averaged together.)

		llvm::StringRef sDF = "DriveFilter";
		double df = frc::SmartDashboard::GetNumber(sDF, 0.2);
		frc::SmartDashboard::PutNumber(sDF, df);
		frc::SmartDashboard::SetPersistent(sDF);

		OutputY = (df * OutputY) + ((1.0 - df) * SpeedLinear);
		OutputX = (df * OutputX) + ((1.0 - df) * SpeedRotate);

		// Drive Code (WPI Built-in)
		Adrive.ArcadeDrive(OutputY, OutputX, false);

		//  Rumble code
		//  Read all motor current from PDP and display on drivers station
		//double driveCurrent = pdp->GetTotalCurrent();	// Get total current
		double driveCurrent = pdp->GetTotalCurrent();

		// rumble if current to high
		double RbtThr = 0.0;		// Define value for total rumble current
		if (driveCurrent > 175.0)		// Rumble if greater than 125 amps motor current
			RbtThr = 0.0;

		IO.DS.DriveStick.SetRumble(Joystick::kLeftRumble, RbtThr); // Set Left Rumble to RbtThr
		IO.DS.DriveStick.SetRumble(Joystick::kRightRumble, RbtThr);	// Set Right Rumble to RbtThr

		//
		// Stuff that Doesn't fit on the op Controller:
		//

		// Z-Bar controls
		IO.DriveBase.Zbar.Set(IO.DS.DriveStick.GetAButton());
		IO.DriveBase.Zbar1.Set(IO.DS.DriveStick.GetBButton());

		// Winch Control [DPAD]
		switch (IO.DS.DriveStick.GetPOV()) {
		case 180:
			// Dpad Down - ground/intake level
			IO.DriveBase.Winches.Set(1.0);
			break;

		case 0:
			// Dpad  Up - Scale Position
			IO.DriveBase.Winches.Set(-1.0);
			break;

		default:
			IO.DriveBase.Winches.Set(0);
			break;
		}

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
			ElevPosTarget = 500;
			break;
		case 0:
			// Dpad  Up - Scale Position
			ElevPosTarget = 17500;
			break;
		}

		if (fabs(ElevatorStick) > Control_Deadband) {

			// Smoothing algorithm for x^3
			if (ElevatorStick > 0.0)
				ElevatorStick = (1 - ElevDeadband) * pow(ElevatorStick, 3) + ElevDeadband;
			else
				ElevatorStick = (1 - ElevDeadband) * pow(ElevatorStick, 3) - ElevDeadband;

			elevatorSpeed(ElevatorStick);
			ElevPosTarget = IO.DriveBase.EncoderElevator.Get();

		} else if (!SensorOverride) {
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
		double wristStick = IO.DS.OperatorStick.GetX(frc::GenericHID::kRightHand);
		wristStick = deadband(wristStick, Control_Deadband);
		wristStick = cubedControl(wristStick, Control_Deadband);

		//IO.DriveBase.Wrist1.Set(wristStick);


		if (fabs(wristStick) > Control_Deadband) {
			/// Manual Control
			wristSpeed(wristStick);
			WristTarget = IO.DriveBase.WristPot.Get();

		} else if (!SensorOverride) {
			// Hold Current Position
			wristPosition(WristTarget);

		} else {
			elevatorSpeed(0.0);
		}


		// Wrist Presets
		if (IO.DS.OperatorStick.GetAButton()) {
			WristTarget = 0;
		}
		if (IO.DS.OperatorStick.GetXButton()) {
			WristTarget = 45;
		}
		if (IO.DS.OperatorStick.GetYButton()) {
			WristTarget = -45;
		}

		//
		// Intake Control
		//
		double OpRightTrigger = IO.DS.OperatorStick.GetTriggerAxis(frc::GenericHID::kRightHand);
		double OpLeftTrigger = IO.DS.OperatorStick.GetTriggerAxis(frc::GenericHID::kLeftHand);
		bool OpRightBumper = IO.DS.OperatorStick.GetBumper(frc::GenericHID::kRightHand);
		bool OpLeftBumper = IO.DS.OperatorStick.GetBumper(frc::GenericHID::kLeftHand);
		bool OpButtonB = IO.DS.OperatorStick.GetBButton();

		double intakeCommand = (OpRightTrigger - OpLeftTrigger);
		intakeCommand = deadband(intakeCommand, Control_Deadband) * 0.65;

		//
		// Claw control
		//

		if (OpRightBumper or (OpRightTrigger > 0.125)) {
			// Loose Intake
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kOff); // Compliant
			IO.DriveBase.ClawIntake.Set(1.0);

		} else if (OpLeftBumper) {
			// Drop it like it's hot
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open
			IO.DriveBase.ClawIntake.Set(0.0);

		} else if (OpButtonB) {
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward); // Closed
			IO.DriveBase.ClawIntake.Set(1.0); // Intake

		} else {
			// Default Hold Cube
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward); // Closed
			IO.DriveBase.ClawIntake.Set(intakeCommand);
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

		// Reset Encoders
		IO.DriveBase.EncoderLeft.Reset();
		IO.DriveBase.EncoderRight.Reset();

		// Turn off drive motors
		IO.DriveBase.MotorsLeft.Set(0);
		IO.DriveBase.MotorsRight.Set(0);

		// Reset the navX heading
		IO.DriveBase.ahrs.ZeroYaw();
		autoHeading = 0.0;

		// Low gear by default
		IO.DriveBase.SolenoidShifter.Set(true);

		// Default Wrist motor to not move
		IO.DriveBase.Wrist1.Set(0.0);

		// Shut the claw by default
		IO.DriveBase.ClawClamp.Set(DoubleSolenoid::Value::kForward);
		IO.DriveBase.ClawIntake.Set(0);

		// Reset the moving average filters for drive base
		OutputY = 0;
		OutputX = 0;

		// Default Elevator default
		ElevPosTarget = 800;
		WristTarget = 25;

		// turn on Auto State to true
		AutoStateCheck = true;

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
		if (autoDelay == IO.DS.sAutoDelay3 and autoTotalTime.Get() < 3)
			return;
		if (autoDelay == IO.DS.sAutoDelay5 and autoTotalTime.Get() < 5)
			return;

		// Closed Loop Position Hold;
		//wristPosition();

		// Cross the Line Auto
		if (autoTarget == IO.DS.AutoLine) {
			autoLine();
		}

		// Center Start
		if (autoPosition == IO.DS.sAutoCenter) {

			// Switch
			if (autoTarget == IO.DS.AutoSwitch) {

				if (autoGameData[0] == 'L')
					autoCenterFast(false);

				if (autoGameData[0] == 'R')
					autoCenterFast(true);
			}

		}

		// Left Start
		if (autoPosition == IO.DS.sAutoLeft) {

			// Switch
			if (autoTarget == IO.DS.AutoSwitch) {

				if (autoGameData[1] == 'L')
					autoSwitchNearSide(false);

				if (autoGameData[1] == 'R')
					autoSwitchBackShoot(false);
			}

			// Scale
			if (autoTarget == IO.DS.AutoScale) {

				if (autoGameData[1] == 'L')
					autoScaleFastNear(false);

				if (autoGameData[1] == 'R')
					autoScaleFastFar(false);
			}

			// Our side
			if (autoTarget == IO.DS.AutoNearSide) {

				if (autoGameData[1] == 'L') {
					autoScaleFastNear(false);

				} else if (autoGameData[0] == 'L') {
					autoSwitchNearSide(false);

				} else if (autoGameData[0] == 'R') {
					autoLine();

				}
			}

			// Near Scale, Near Switch, Far Scale
			if (autoTarget == IO.DS.AutoNearSide) {

				if (autoGameData[1] == 'L') {
					autoScaleFastNear(false);

				} else if (autoGameData[0] == 'L') {
					autoSwitchNearSide(false);

				} else if (autoGameData[0] == 'R') {
					autoScaleFastFar(false);

				}
			}
		}

		// Right Start
		if (autoPosition == IO.DS.sAutoRight) {

			// Switch
			if (autoTarget == IO.DS.AutoSwitch) {

				if (autoGameData[1] == 'R')
					autoSwitchNearSide(true);

				if (autoGameData[1] == 'L')
					autoSwitchBackShoot(true);
			}

			// Scale
			if (autoTarget == IO.DS.AutoScale) {

				if (autoGameData[1] == 'R')
					autoScaleFastNear(true);

				if (autoGameData[1] == 'L')
					autoScaleFastFar(true);
			}

			// Our side
			if (autoTarget == IO.DS.AutoNearSide) {

				if (autoGameData[1] == 'R') {
					autoScaleFastNear(true);

				} else if (autoGameData[0] == 'R') {
					autoSwitchNearSide(true);

				} else if (autoGameData[0] == 'L') {
					autoLine();

				}
			}

			// Near Scale, Near Switch, Far Scale
			if (autoTarget == IO.DS.AutoNearSide) {

				if (autoGameData[1] == 'R') {
					autoScaleFastNear(true);

				} else if (autoGameData[0] == 'R') {
					autoSwitchNearSide(true);

				} else if (autoGameData[0] == 'L') {
					autoScaleFastFar(true);

				}
			}
		}

		// 2Cube Auto
		if (autoFinisher == IO.DS.sAutoCube2Score) {
			if (!auto2CubeStartRight) {
				if (autoGameData[1] == 'L')
					autoCube2ScoreNear(1);
				else
					autoCube2ScoreFar(1);
			} else {
				if (autoGameData[1] == 'L')
					autoCube2ScoreNear(-1);
				else
					autoCube2ScoreFar(-1);
			}
		}
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
	void autoCenterFast_org(bool isGoRight) {

		// Mirror path if starting on right
		double rot = 1;
		double CAoffset = 67;
		if (isGoRight) {
			rot = -1;
			CAoffset = 53;
		}

		SmartDashboard::PutNumber("OffSet Set", CAoffset);

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);

		// High gear
		IO.DriveBase.SolenoidShifter.Set(false);

		//Hold Cube
		//IO.DriveBase.ClawIntake.Set(0.2);

		switch (autoModeState) {
		case 1:
			if (autoForward(18)) {
				ElevPosTarget = 800;
				autoNextState();
			}
			break;

		case 2:
			if (autoTurn(45.0 * rot))
				autoNextState();
			break;

		case 3:
			if (autoForward(CAoffset)) {
				autoNextState();
			}
			break;

		case 4:
			if (autoTurn(0.0, 0.40, 0.15))
				autoNextState();
			break;

		case 5:
			if (timedDrive(1.7, 0.5, 0.5))
				autoNextState();
			break;

		case 6:
			IO.DriveBase.Wrist1.Set(-0.4);

			if (AutonTimer.Get() > 0.5 ) {
			//if (AutonTimer.Get() > 1.5 || wristPosition(45.0)) {
				IO.DriveBase.Wrist1.Set(0.25);
				//wristPosition(0.0);
				autoNextState();
			}
			break;

		case 7:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.65);

			// keep pushing!
			if (timedDrive(1.0, 0.15, 0.15)) {
				IO.DriveBase.ClawIntake.Set(0.0);
				IO.DriveBase.Wrist1.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());
			}

			break;

		case 8:
			if (autoFinisher == IO.DS.sAutoCube2Get) {
				autoNextState();

			} else {
				autoNextState();
				autoModeState = 0;
			}

			break;
		case 9:
			// Start of wall hug path
			if (autoForward(-50))
				autoNextState();
			break;

		case 10:
			if (autoTurn(45 * rot))

				autoNextState();
			break;

		case 11:
			if (autoForward(73))
				autoNextState();
			break;

		case 12:
			if (autoTurn(0, 0.6, 0.2))
				autoNextState();
			break;

		case 13:
			ElevPosTarget = 0;
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kOff); // Compliant
			IO.DriveBase.Wrist1.Set(0.45);
			if (autoForward(92 + 42, 1.0, 0.2))
				autoNextState();
			break;

		case 14:
			if (autoTurn(58 * rot, 1.0, 0.1))
				autoNextState();
			break;

		case 15:
			IO.DriveBase.SolenoidShifter.Set(true);
			IO.DriveBase.ClawIntake.Set(1.0);
			if (autoForward(-54, 0.4, 0))
				autoNextState();
			break;


		case 16:
			IO.DriveBase.Wrist1.Set(0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward); // Closed
			IO.DriveBase.ClawIntake.Set(0);
			//IO.DriveBase.Wrist1.Set(-0.35);
			if (true)
				autoNextState();
			break;

		case 17:
			IO.DriveBase.Wrist1.Set(-1.0);
			if (autoForward(10))
				autoNextState();
			break;

		case 18:

			if (autoTurn(-30))
				autoNextState();
			ElevPosTarget = 14000;
			break;

		case 19:

			if (autoForward(30))
				autoNextState();
			IO.DriveBase.ClawIntake.Set(-0.65);
			break;

		case 20:

			//190
			if (autoForward(-15)) {
				autoNextState();
				ElevPosTarget = 1000;
			}

			break;

		default:
			stopMotors();

		}

		return;
	}


	void autoCenterFast(bool isGoRight) {

		// Mirror path if starting on right
		double rot = 1;
		double CAoffset = 67;
		if (isGoRight) {
			rot = -1;
			CAoffset = 53;
		}

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);

		// High gear
		IO.DriveBase.SolenoidShifter.Set(false);

		//Hold Cube
		//IO.DriveBase.ClawIntake.Set(0.2);

		switch (autoModeState) {
		case 1:
			if (autoForward(18)) {
				autoNextState();
			}
			break;

		case 2:
			if (autoTurn(45.0 * rot))
				autoNextState();
			break;

		case 3:
			if (autoForward(CAoffset)) {
				autoNextState();
			}
			break;

		case 4:
			ElevPosTarget = 1200;
			if (autoTurn(0.0))
				autoNextState();
			break;

		case 5:
			if (timedDrive(0.7, 0.5, 0.5))
				autoNextState();
			break;

		case 6:
			IO.DriveBase.Wrist1.Set(-0.4);

			if (AutonTimer.Get() > 0.75 ) {
			//if (AutonTimer.Get() > 1.5 || wristPosition(45.0)) {
				IO.DriveBase.Wrist1.Set(0.25);
				//wristPosition(0.0);
				autoNextState();
			}
			break;

		case 7:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.65);

			// keep pushing!
			if (timedDrive(1.0, 0.15, 0.15)) {
				IO.DriveBase.ClawIntake.Set(0.0);
				IO.DriveBase.Wrist1.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());
			}

			break;

		case 8:
			if (autoFinisher == IO.DS.sAutoCube2Get) {
				autoNextState();

			} else {
				autoNextState();
				autoModeState = 0;
			}

			break;
		case 9:
			// Start of wall hug path
			if (autoForward(-50))
				autoNextState();
			break;

		case 10:
			if (autoTurn(45 * rot))

				autoNextState();
			break;

		case 11:
			if (autoForward(73))
				autoNextState();
			break;

		case 12:
			if (autoTurn(0, 0.6, 0.2))
				autoNextState();
			break;

		case 13:
			ElevPosTarget = 0;
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kOff); // Compliant
			IO.DriveBase.Wrist1.Set(0.45);
			if (autoForward(92 + 42, 1.0, 0.2))
				autoNextState();
			break;

		case 14:
			if (autoTurn(58 * rot, 1.0, 0.1))
				autoNextState();
			break;

		case 15:
			IO.DriveBase.SolenoidShifter.Set(true);
			IO.DriveBase.ClawIntake.Set(1.0);
			if (autoForward(-54, 0.4, 0))
				autoNextState();
			break;


		case 16:
			IO.DriveBase.Wrist1.Set(0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward); // Closed
			IO.DriveBase.ClawIntake.Set(0);
			//IO.DriveBase.Wrist1.Set(-0.35);
			if (true)
				autoNextState();
			break;

		case 17:
			IO.DriveBase.Wrist1.Set(-1.0);
			if (autoForward(10))
				autoNextState();
			break;

		case 18:

			if (autoTurn(-30))
				autoNextState();
			ElevPosTarget = 14000;
			break;

		case 19:

			if (autoForward(30))
				autoNextState();
			IO.DriveBase.ClawIntake.Set(-0.65);
			break;

		case 20:

			//190
			if (autoForward(-15)) {
				autoNextState();
				ElevPosTarget = 1000;
			}

			break;

		default:
			stopMotors();

		}

		return;
	}


	void autoScaleFastNear(bool isStartRightPos) {

		// Mirror rotations for right side start
		double direction = 1;
		if (isStartRightPos)
			direction = -1;

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);

		// High gear
		IO.DriveBase.SolenoidShifter.Set(false);

		switch (autoModeState) {
		case 1:
			if (autoForward(262, 0.9, 0.1)) {
				//autoNextState();
				ElevPosTarget = 11000;				//TODO: Set back to Full Height (TESTING)

				if (elevatorPosition(ElevPosTarget)) {
					autoNextState();
				}

			}
			break;

		case 2:
			if (autoTurn(-45 * direction, 0.5, 0.1)) {
				autoNextState();
			}
			break;

		case 3:

			if (autoForward(15)) {
				autoNextState();
			}
			break;


		case 4:
			IO.DriveBase.Wrist1.Set(-0.4);

			if (AutonTimer.Get() > 0.75 ) {
			//if (AutonTimer.Get() > 1.5 || wristPosition(45.0)) {
				IO.DriveBase.Wrist1.Set(0.0);
				//wristPosition(0.0);
				autoNextState();
			}
			break;

		case 5:

			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.85);

			// The Crowd Goes Wild!
			if (AutonTimer.Get() > 1.0) {
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());
			}

			break;

		case 6:

			//190
			if (autoForward(-15)) {
				autoNextState();
				ElevPosTarget = 1000;
			}

			break;

		case 7:
			if (autoFinisher == IO.DS.sAutoCube2Get)
				autoNextState();
			else
				autoModeState = 0; // We are done.
			break;

		case 8:
			IO.DriveBase.Wrist1.Set(0.45);
			ElevPosTarget = 800;

			if (autoTurn(0) && elevatorPosition(ElevPosTarget))
				autoNextState();
			break;

		case 9:
			if (autoForward(-36))
				autoNextState();
			break;

		case 10:
			if (autoTurn(15 * direction))
				autoNextState();
			break;

		case 11:
			if (autoForward(-18))
				autoNextState();
			break;

		case 12:
			if (autoTurn(0))
				autoNextState();
			break;

		case 13:
			// Loose Intake
			IO.DriveBase.ClawIntake.Set(1.0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kOff);

			if (timedDrive(1.5, -0.4, -0.4))
				autoNextState();
			break;

		case 14:
			ElevPosTarget = 2200;
			IO.DriveBase.ClawIntake.Set(0.0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward);
			IO.DriveBase.Wrist1.Set(0.0);

			autoNextState();

			// Trigger the score cube 2 auto mode
			auto2CubeStartRight = isStartRightPos;
			autoModeState = 100;
			break;

		default:
			stopMotors();

		}

		return;
	}

	void autoScaleFastFar(bool isRightSide) {

		// Mirror rotations for right side start
		double direction = 1;
		if (isRightSide)
			direction = -1;

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);

		// High gear
		IO.DriveBase.SolenoidShifter.Set(false);

		switch (autoModeState) {
		case 1:
			// Auto home the elevator
			elevatorSpeed(-0.2);

			//210
			if (autoForward(228, 1.0, 0.1)) {
				autoNextState();
			}
			break;

		case 2:
			if (autoTurn(-90.0 * direction)) {
				autoNextState();
			}
			break;

		case 3:
			//190
			if (autoForward(222, 1.0, 0.1)) {
				autoNextState();

			}
			break;

		case 4:
			if (autoTurn(20 * direction)) {
				autoNextState();
			}
			break;

		case 5:
			ElevPosTarget = 11000;  //TODO: Set back to Full Height (TESTING)
			autoTurn(30 * direction);
			if (elevatorPosition(ElevPosTarget)) {
				autoNextState();

			}
			break;

		case 6:
			IO.DriveBase.SolenoidShifter.Set(true);
			if (autoForward(48, 0.5, 0)) {
				autoNextState();
			}
			break;


		case 7:
			IO.DriveBase.Wrist1.Set(-0.4);

			if (AutonTimer.Get() > 0.75 ) {
			//if (AutonTimer.Get() > 1.5 || wristPosition(45.0)) {
				IO.DriveBase.Wrist1.Set(0.0);
				//wristPosition(0.0);
				autoNextState();
			}
			break;

		case 8:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.85);

			//autoTurn(20 * direction);

			// The Crowd Goes Wild!
			if (AutonTimer.Get() > 1.0) {
				IO.DriveBase.ClawIntake.Set(0.0);
				IO.DriveBase.Wrist1.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());
			}

			break;

		case 9:
			//190
			if (autoForward(-20, 0.5, 0.1)) {
				autoNextState();
				ElevPosTarget = 800;
			}

			break;

		case 10:
			if (autoFinisher == IO.DS.sAutoCube2Get)
				autoNextState();
			else
				autoModeState = 0; // We are done.
			break;

		case 11:
			IO.DriveBase.Wrist1.Set(0.45);
			ElevPosTarget = 800;

			if (elevatorPosition(ElevPosTarget))
				autoNextState();
			break;

		case 12:
			if (autoForward(-36))
				autoNextState();
			break;

		case 13:
			if (autoTurn(15 * direction))
				autoNextState();
			break;

		case 14:
			if (autoForward(-18))
				autoNextState();
			break;

		case 15:
			if (autoTurn(0))
				autoNextState();
			break;

		case 16:
			// Loose Intake
			IO.DriveBase.ClawIntake.Set(1.0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kOff);

			if (timedDrive(1.5, -0.4, -0.4))
				autoNextState();
			break;

		case 17:
			ElevPosTarget = 2200;
			IO.DriveBase.ClawIntake.Set(0.0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward);
			IO.DriveBase.Wrist1.Set(0.0);

			autoNextState();

			// Trigger the score cube 2 auto mode
			auto2CubeStartRight = !isRightSide;
			autoModeState = 100;

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
	void autoSwitchNearSide(bool isStartRightPos) {

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);

		// Rotate based on field start position
		double rotDir = 1.0;
		if (isStartRightPos) {
			rotDir = -1.0;
		}

		// Auto Sequence
		switch (autoModeState) {
		case 1:
			IO.DriveBase.SolenoidShifter.Set(false); //High Gear

			if (autoForward(123))
				autoNextState();

			break;

		case 2:
			if (autoTurn(-90 * rotDir, 0.5, 0.1))
				autoNextState();

			break;

		case 3:
			IO.DriveBase.SolenoidShifter.Set(true); // Low Gear
			if (timedDrive(0.6, 0.5, 0.5))
				autoNextState();

			break;

		case 4:
			IO.DriveBase.ClawIntake.Set(-0.65);

			if (timedDrive(0.75, 0.2, 0.2)) {
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());
			}

			break;

		case 5:
			if (autoFinisher == IO.DS.sAutoCube2Get)
				autoNextState();
			else
				autoModeState = 0; // We are done.
			break;

		case 6:
			IO.DriveBase.Wrist1.Set(0.3);
			ElevPosTarget = 800;

			if (elevatorPosition(ElevPosTarget))
				autoNextState();
			break;

		case 7:
			if (autoForward(-18))
				autoNextState();
			break;

		case 8:
			if (autoTurn(0))
				autoNextState();
			break;

		case 9:
			if (autoForward(36))
				autoNextState();
			break;

		case 10:
			if (autoTurn(-45 * rotDir))
				autoNextState();
			break;

		case 11:
			if (autoForward(28))
				autoNextState();
			break;

		case 12:
			if (autoTurn(0))
				autoNextState();
			break;

		case 13:
			// Loose Intake
			IO.DriveBase.ClawIntake.Set(1.0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kOff);

			if (timedDrive(1.5, -0.4, -0.4))
				autoNextState();
			break;

		case 14:
			ElevPosTarget = 2200;
			IO.DriveBase.ClawIntake.Set(0.0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward);
			IO.DriveBase.Wrist1.Set(0.0);

			autoNextState();

			// Trigger the score cube 2 auto mode
			auto2CubeStartRight = isStartRightPos;
			autoModeState = 100;

			break;

		default:
			stopMotors();

		}

		return;
	}

	void autoSwitchBackShoot(bool isStartRightPos) {

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);

		// Rotate based on field start position
		double rotDir = 1.0;
		if (isStartRightPos) {
			rotDir = -1.0;
		}

		// Auto Sequence
		switch (autoModeState) {
		case 1:
			if (autoForward(223))
				autoNextState();

			break;

		case 2:
			if (autoTurn(-90 * rotDir))
				autoNextState();

			break;

		case 3:
			if (autoForward(105))
				autoNextState();

			break;

		case 4:
			ElevPosTarget = 7500;
			IO.DriveBase.Wrist1.Set(-0.45);

			if (autoTurn((-90 - 25) * rotDir))
				autoNextState();

			break;

		case 5:
			if (timedDrive(0.5, 0.8, 0.8))
				autoNextState();
			break;

		case 6:
			IO.DriveBase.ClawIntake.Set(-0.65);

			if (timedDrive(1.5, 0.3, 0.3))
				autoNextState();
			break;

		case 7:
			IO.DriveBase.Wrist1.Set(0.0);
			IO.DriveBase.ClawIntake.Set(0.0);
			autoNextState();

			// Display auton Time
			SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());

			break;

		default:
			stopMotors();

		}

		return;
	}

	void autoSwitchBackLegal(bool isStartRightPos) {

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);

		// Rotate based on field start position
		double rotDir = 1.0;
		if (isStartRightPos) {
			rotDir = -1.0;
		}

		// Auto Sequence
		switch (autoModeState) {
		case 1:
			if (autoForward(223))
				autoNextState();

			break;

		case 2:
			if (autoTurn(-90 * rotDir))
				autoNextState();

			break;

		case 3:
			if (autoForward(145))
				autoNextState();

			break;

		case 4:
			ElevPosTarget = 6500;
			IO.DriveBase.Wrist1.Set(-0.45);

			if (autoTurn((-90 - 25) * rotDir))
				autoNextState();

			break;

		case 5:
			if (autoForward(36.0))
				autoNextState();

			break;

		case 6:
			if (autoForward(-12.0))
				autoNextState();

			break;

		case 7:
			if (autoTurn(-180 * rotDir))
				autoNextState();

			break;

		case 8:
			if (timedDrive(0.5, 0.8, 0.8))
				autoNextState();
			break;

		case 9:
			IO.DriveBase.ClawIntake.Set(-0.65);
			autoNextState();

			// Display auton Time
			SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());

			break;
		default:
			stopMotors();

		}

		return;
	}

	/*
	 * AUTO PROGRAM - Score Second Cube
	 *
	 * Assumes that the prior auto sequence acquired the cube on the end
	 *
	 */
	void autoCube2ScoreNear(double rot) {

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);
		ElevPosTarget = 800;

		// High gear
		IO.DriveBase.SolenoidShifter.Set(false);

		switch (autoModeState) {
		case 100:
			if (autoForward(14))
				autoNextState();
			break;

		case 101:
			if (autoTurn(45.0 * rot))
				autoNextState();
			break;

		case 102:
			if (autoForward(24))
				autoNextState();
			break;

		case 103:
			ElevPosTarget = 6000; // TODO: Fix this for competition
			IO.DriveBase.Wrist1.Set(-0.45);

			if (autoTurn(0.0) && elevatorPosition(ElevPosTarget))
				autoNextState();
			break;

		case 104:
			if (autoForward(18))
				autoNextState();
			break;

		case 105:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.65);

			// keep pushing!
			if (AutonTimer.Get() > 1.0) {
				IO.DriveBase.ClawIntake.Set(0.0);
				IO.DriveBase.Wrist1.Set(0.0);
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

	void autoCube2ScoreFar(double rot) {

		// Closed Loop control of Elevator
		elevatorPosition(ElevPosTarget);
		ElevPosTarget = 800;

		// High gear
		IO.DriveBase.SolenoidShifter.Set(false);

		switch (autoModeState) {
		case 100:
			if (autoForward(14))
				autoNextState();
			break;

		case 101:
			if (autoTurn(-90.0 * rot))
				autoNextState();
			break;

		case 102:
			if (autoForward(285))
				autoNextState();
			break;

		case 103:
			ElevPosTarget = 6000; // TODO: Fix this for competition
			IO.DriveBase.Wrist1.Set(-0.45);

			if (autoTurn(0.0) && elevatorPosition(ElevPosTarget))
				autoNextState();
			break;

		case 104:
			if (autoForward(28))
				autoNextState();
			break;

		case 105:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.65);

			// keep pushing!
			if (AutonTimer.Get() > 1.0) {
				IO.DriveBase.ClawIntake.Set(0.0);
				IO.DriveBase.Wrist1.Set(0.0);
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

	void elevatorSpeed(double elevMotor) {

		// Limit Switches
		bool ElevatorUpperLimit = IO.DriveBase.SwitchElevatorUpper.Get();
		bool ElevatorLowerLimit = IO.DriveBase.SwitchElevatorLower.Get();

		// Get Current Encoder Value
		double ElevEncoderRead = IO.DriveBase.EncoderElevator.Get();

		// Slow down if approaching limits
		if (ElevEncoderRead < 800 and elevMotor < 0 and (!SensorOverride))
			elevMotor *= 0.3;

		if (ElevEncoderRead > 17500 and elevMotor > 0 and (!SensorOverride))
			elevMotor *= 0.3;

		// Zero the encoder if we hit the lower limit switch
		if (ElevatorLowerLimit == false) {
			IO.DriveBase.EncoderElevator.Reset(); // Reset encoder to 0
		}

		// If a limit switch is pressed, only allow a reverse motion
		if ((!ElevatorUpperLimit) and (elevMotor > 0) and (!SensorOverride)) {
			IO.DriveBase.Elevator1.Set(0);
			IO.DriveBase.Elevator2.Set(0);

		} else if ((!ElevatorLowerLimit) and (elevMotor < 0) and (!SensorOverride)) {
			IO.DriveBase.Elevator1.Set(0);
			IO.DriveBase.Elevator2.Set(0);

		} else {
			IO.DriveBase.Elevator1.Set(elevMotor);
			IO.DriveBase.Elevator2.Set(elevMotor);
		}
	}

#define Elevator_MAXSpeed (1.0)
#define Elevator_KP (0.0005)
#define ElevatorPositionTol (500)

	bool elevatorPosition(double Elev_position) {

		// Get Current Encoder Value
		double ElevEncoderRead = IO.DriveBase.EncoderElevator.Get();

		// Anti-bounce
		bool ElevatorUpperLimit = IO.DriveBase.SwitchElevatorUpper.Get();
		if ((!ElevatorUpperLimit) and (!SensorOverride))
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

	//
	// Wrist Control
	//
	void wristSpeed(double input) {

		// Sensor Override Mode
		if (SensorOverride) {
			IO.DriveBase.Wrist1.Set(input);

			return;
		}

		// Get Current Wrist Angle
		double wristAngle = IO.DriveBase.WristPot.Get();
		double softLimit = 80;
		double hardLimit = 90;

		// Slow down if approaching limits
		if (wristAngle < -softLimit and input < 0) {
			input *= 0.3;
		}

		if (wristAngle > softLimit and input > 0) {
			input *= 0.3;
		}

		// If a limit is reached, only allow motion away from it
		if ((wristAngle > hardLimit) and (input > 0)) {
			IO.DriveBase.Wrist1.Set(0);

		} else if ((wristAngle < -hardLimit) and (input < 0)) {
			IO.DriveBase.Wrist1.Set(0);

		} else {
			IO.DriveBase.Wrist1.Set(input);

		}
	}

#define Wrist_MAXSpeed (1.0)
#define Wrist_KP (0.080)
#define WristPositionTol (5.0)

	bool wristPosition(double input) {

		// Update Target Position
		WristTarget = input;

		// Get Current Encoder Value
		double wristAngle = IO.DriveBase.WristPot.Get();

		// Motor Command Calculation
		double error = input - wristAngle;
		double ElevCmd = error * Wrist_KP;

		//Limit Elevator Max Speed
		ElevCmd = absMax(ElevCmd, Wrist_MAXSpeed);

		// Set the wrist speed
		wristSpeed(ElevCmd);

		// Check if we made it to the target
		return (fabs(error) <= WristPositionTol);
	}

	// Default to holding whatever position was last commanded
	bool wristPosition() {
		return wristPosition(WristTarget);
	}

// Drivetrain functions

	void motorSpeed(double leftMotor, double rightMotor) {

		// Moving Average Filter (slip reduction attempt)
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

#define KP_LINEAR (0.0311)
#define KI_LINEAR (0.000)
#define KD_LINEAR (0.004)
#define LINEAR_TOLERANCE (1.0)

#define ROTATION_kP (0.0609)
#define ROTATION_TOLERANCE (10.0)
#define ROTATIONAL_SETTLING_TIME (0.0)
#define ROTATIONAL_MAX_SPEED (0.40)

	double prevError = 0; // Derivative Calculation
	double sumError = 0;  // Integral Calculation

	int autoForward(double targetDistance, double max_speed, double settle_time) {

		double encoderDistance = getEncoderDistance();

		// P Control
		double error = targetDistance - encoderDistance;        // [Inches]

		// I Control
		if (error < 24) {
			sumError += error / MAIN_LOOP_PERIOD;
		} else {
			sumError = 0;
		}

		// D Control
		double dError = (error - prevError) / MAIN_LOOP_PERIOD; // [Inches/second]
		prevError = error;

		// PID Command
		double driveCommandLinear = error * KP_LINEAR + KI_LINEAR * sumError + KD_LINEAR * dError;


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
		if (abs(error) > LINEAR_TOLERANCE) {
			autoSettleTimer.Reset();

		} else if (autoSettleTimer.Get() > settle_time)
			return 1;

		return 0;
	}

	// Overload for backwards compatibility
	int autoForward(double targetDistance) {
		return autoForward(targetDistance, 1.0, 0.0);
	}


	int autoTurn(float targetYaw, double maxSpeed, double settlingTime) {

		// For linear drive function
		autoHeading = -targetYaw;

		// Calculate motor command
		float currentYaw = IO.DriveBase.ahrs.GetAngle();
		float yawError = -targetYaw - currentYaw;
		float yawCommand = yawError * -ROTATION_kP;

		// Limit max rotation speed
		yawCommand = absMax(yawCommand, maxSpeed);

		// dooo it!
		motorSpeed(-yawCommand, yawCommand);

		// Allow for the robot to settle into position
		if (abs(yawError) > ROTATION_TOLERANCE)
			autoSettleTimer.Reset();

		else if (autoSettleTimer.Get() > settlingTime)
			return 1;

		return 0;
	}

	int autoTurn(float targetYaw) {
		return autoTurn(targetYaw, ROTATIONAL_MAX_SPEED, ROTATIONAL_SETTLING_TIME);
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
		double encoderDistance = 0;		// = AutonTimer.Get() * 48.0;

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

	double getEncoderRate() {

		// If an encoder is available, use it...
		double encoderLeft = IO.DriveBase.EncoderLeft.GetRate();
		double encoderRight = IO.DriveBase.EncoderRight.GetRate();

		if (autoEncoder == IO.DS.EncoderAuto) {
			// Automatically select the larger value (assume one was disconnected)
			if (fabs(encoderLeft) > fabs(encoderRight))
				return encoderLeft;
			else
				return encoderRight;
		}

		if (autoEncoder == IO.DS.EncoderBoth)
			return (encoderLeft + encoderRight) / 2.0;

		if (autoEncoder == IO.DS.EncoderLeft)
			return encoderLeft;

		if (autoEncoder == IO.DS.EncoderRight)
			return encoderRight;

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

	// Dead band function
	// Scales output to accommodate for the loss of the deadband region
	double deadband(double input, double minval) {

		// If less than deadband value, return zero
		if (fabs(input) < minval)
			return 0.0;

		// Transform input so that output has full range [0.0 - 1.0]
		if (input < 0.0)
			return absMax(input * (1 - minval) - minval, 1.0);
		else
			return absMax(input * (1 - minval) + minval, 1.0);

	}

	double cubedControl(double input, double minval) {

		if (input == 0.0) return 0.0;

		// Smoothing algorithm for x^3
		if (input > 0.0)
			return (1 - minval) * pow(input, 3) + minval;

		else
			return (1 - minval) * pow(input, 3) - minval;

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
		SmartDashboard::PutNumber("Wrist PWM", IO.DriveBase.Wrist1.Get());
		SmartDashboard::PutNumber("Inkate PWM", IO.DriveBase.ClawIntake.Get());

		// Drive Joystick Inputs
		SmartDashboard::PutNumber("Speed Linear", IO.DS.DriveStick.GetY(GenericHID::kLeftHand));
		SmartDashboard::PutNumber("Speed Rotate", IO.DS.DriveStick.GetX(GenericHID::kRightHand) * -1);

		// Auto State
		SmartDashboard::PutString(llvm::StringRef("Auto Target"), llvm::StringRef(autoTarget));
		SmartDashboard::PutString(llvm::StringRef("Auto Position"), llvm::StringRef(autoPosition));
		SmartDashboard::PutString(llvm::StringRef("Auto Fin"), llvm::StringRef(autoFinisher));
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
		SmartDashboard::PutString(llvm::StringRef("autoGameData"), llvm::StringRef(autoGameData));

		// State Vars
		SmartDashboard::PutNumber("ElevPosTarget", ElevPosTarget);

		// Elevator Limit Switches
		SmartDashboard::PutBoolean("SwitchElevatorUpper", IO.DriveBase.SwitchElevatorUpper.Get());
		SmartDashboard::PutBoolean("SwitchElevatorLower", IO.DriveBase.SwitchElevatorLower.Get());

		//Wrist Pot
		SmartDashboard::PutNumber("Wrist AI", IO.DriveBase.WristAI.GetVoltage());
		SmartDashboard::PutNumber("Wrist Pot", IO.DriveBase.WristPot.Get());

		// Game State
		SmartDashboard::PutBoolean("Autonomous Running", AutoStateCheck);
		SmartDashboard::PutBoolean("TeleOp Running", TeleopStateCheck);

		// Sensor Override
		SmartDashboard::PutBoolean("Sensor Override", SensorOverride);

	}

}
;

START_ROBOT_CLASS(Robot);

