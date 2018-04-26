//// DETROIT-CHAMPS-B

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

	//States config
	double m_WristOffset = 13 - 23 + 11;
	double WristScale = 1;

	//practice robot
	//double m_WristOffset = 13 - 23 + 33;
	//double WristScale = -1;

	// This number needs to be changed until the wrist reads 0 at top dead center + is toward the front of the robot

	//Autonomous Variables
	Timer AutonTimer, autoSettleTimer, autoTotalTime;
	std::string autoGameData, autoDelay, autoMode, autoEncoder, autoPosition, autoFinisher, PotDisabled,
			GameDataOveride;
	int autoModeState;  // current step in auto sequence
	double autoHeading; // current gyro heading to maintain

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
		autoMode = IO.DS.chooseAutoMode.GetSelected();
		autoEncoder = IO.DS.chooseAutoEncoder.GetSelected();
		PotDisabled = IO.DS.choosePotDisabled.GetSelected();
		GameDataOveride = IO.DS.chooseAutoGameData.GetSelected();

		// Get the game-specific message (ex: RLL)
		autoGameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		std::transform(autoGameData.begin(), autoGameData.end(), autoGameData.begin(), ::toupper);

		// Launchpad data
		DriverStation& DS = frc::DriverStation::GetInstance();
		IO.DS.LaunchPad.SetOutput(3, DS.IsAutonomous() && DS.IsEnabled());
		IO.DS.LaunchPad.SetOutput(4, DS.IsOperatorControl() && DS.IsEnabled());

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

		// Wrist Angle Reminder Rumble
		double wristAngle = (IO.DriveBase.WristPot.Get() + m_WristOffset) * WristScale;

		if (wristAngle > 10 && DS.IsEnabled()) {
			IO.DS.OperatorStick.SetRumble(Joystick::kLeftRumble, 0.25);
		} else {
			IO.DS.OperatorStick.SetRumble(Joystick::kLeftRumble, 0);
		}

		if (wristAngle < -10 && DS.IsEnabled()) {
			IO.DS.OperatorStick.SetRumble(Joystick::kRightRumble, 0.25);
		} else {
			IO.DS.OperatorStick.SetRumble(Joystick::kRightRumble, 0);
		}

	}

	void DisabledPeriodic() {
		// NOP
	}

	void TeleopInit() {
		// drive command averaging filter
		OutputX = 0, OutputY = 0;

		// Hold current elevator position
		ElevPosTarget = IO.DriveBase.EncoderElevator.Get();
		WristTarget = IO.DriveBase.WristPot.Get();

		// High Gear by default
		IO.DriveBase.SolenoidShifter.Set(false);
		gearState = true;

		//Hook Tower in by default
		IO.DriveBase.HookDeploy.Set(false);
	}

	bool bTeleAutoMode = false;

	void TeleopPeriodic() {

		// Tele-Auto-Test
		// Run auto in teleop for testing during practice matches
		if (IO.DS.DriveStick.GetAButton()) {

			// Force game data
			if (GameDataOveride != IO.DS.sGameDataOff) {
				autoGameData = GameDataOveride;
			}

			// Run Auto Init
			if (!bTeleAutoMode) {
				AutonomousInit();
				bTeleAutoMode = true;
			}

			AutonomousPeriodic();
			return;
		} else {
			if (bTeleAutoMode) {
				TeleopInit();
				bTeleAutoMode = false;
			}
		}

		double Control_Deadband = 0.11; // input where the joystick actually starts to move
		double Drive_Deadband = 0.11; // command at which the motors begin to move

		// Drive Control Inputs
		double SpeedLinear = IO.DS.DriveStick.GetY(GenericHID::kLeftHand) * 1; // get Yaxis value (forward)
		double SpeedRotate = IO.DS.DriveStick.GetX(GenericHID::kRightHand) * -1; // get Xaxis value (turn)

		// Power Brake
		//bool bPowerBrake = (fabs(IO.DS.DriveStick.GetTriggerAxis(frc::GenericHID::kRightHand)) > Drive_Deadband);

		//Swap below deadband
		// Bad joystick compensation. :)
		SpeedLinear *= 1.05;
		SpeedRotate *= 1.05;

		if (IO.DS.DriveStick.GetBButton()) {

			IO.DriveBase.Winches.Set(1.0);

		} else if (IO.DS.DriveStick.GetYButton()) {

			IO.DriveBase.Winches.Set(-1.0);

		} else {

			IO.DriveBase.Winches.Set(0.0);
		}

		// Winch Control [DPAD]
		switch (IO.DS.DriveStick.GetPOV()) {
		case 0:
			// Dpad  Up
			IO.DriveBase.HTower.Set(-1.0);
			break;

		case 180:
			// Dpad Down
			IO.DriveBase.HTower.Set(1.0);
			break;

		case 90:
			IO.DriveBase.HookDeploy.Set(true);
			break;

		case 270:
			IO.DriveBase.HookDeploy.Set(false);
			break;

		default:
			IO.DriveBase.HTower.Set(0.08);
			break;
		}

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
		/*
		 if (bPowerBrake) {
		 SpeedLinear *= 0.4;
		 IO.DriveBase.SolenoidShifter.Set(true);
		 } else {
		 */
		IO.DriveBase.SolenoidShifter.Set(gearState);
		//}

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
			ElevPosTarget = 16500;
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

		/*

		 // Controller Rumble if the elevator motor current is high
		 double elevCurrent_m1 = pdp->GetCurrent(8);
		 double elevCurrent_m2 = pdp->GetCurrent(9);

		 IO.DS.OperatorStick.SetRumble(Joystick::kLeftRumble, elevCurrent_m1 > 30.0);
		 IO.DS.OperatorStick.SetRumble(Joystick::kRightRumble, elevCurrent_m2 > 30.0);

		 */

		//
		// Wrist control
		//
		double wristStick = IO.DS.OperatorStick.GetX(frc::GenericHID::kRightHand);
		wristStick = deadband(wristStick, 0.15);
		wristStick = cubedControl(wristStick, Control_Deadband);

		//if (PotDisabled == IO.DS.DisabledPOT) {
		IO.DriveBase.Wrist1.Set(wristStick);
		/*
		 } else {

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
		 }
		 */

		// Wrist Presets
		if (IO.DS.OperatorStick.GetAButton()) {
			wristPosition(0);
		} else if (IO.DS.OperatorStick.GetXButton()) {
			wristPosition(45);
		} else if (IO.DS.OperatorStick.GetYButton()) {
			wristPosition(-45);
		} else {
			IO.DriveBase.Wrist1.Set(wristStick);
		}

		//
		// Intake Control
		//
		double OpRightTrigger = IO.DS.OperatorStick.GetTriggerAxis(frc::GenericHID::kRightHand);
		double OpLeftTrigger = IO.DS.OperatorStick.GetTriggerAxis(frc::GenericHID::kLeftHand);
		bool OpRightBumper = IO.DS.OperatorStick.GetBumper(frc::GenericHID::kRightHand);
		bool OpLeftBumper = IO.DS.OperatorStick.GetBumper(frc::GenericHID::kLeftHand);
		bool OpButtonB = IO.DS.OperatorStick.GetBButton();

		// Ricky request
		double DrRightTrigger = IO.DS.DriveStick.GetTriggerAxis(frc::GenericHID::kRightHand);
		double DrLeftTrigger = IO.DS.DriveStick.GetTriggerAxis(frc::GenericHID::kLeftHand);

		double OpIntakeCommand = (OpRightTrigger - OpLeftTrigger);
		OpIntakeCommand = deadband(OpIntakeCommand, Control_Deadband) * 0.7;

		double DrIntakeCommand = (DrRightTrigger - DrLeftTrigger);
		DrIntakeCommand = deadband(DrIntakeCommand, Control_Deadband) * 0.7;

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

		} else if (DrLeftTrigger > 0.25) {
			// Drop it like it's hot
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open
			IO.DriveBase.ClawIntake.Set(0.0);

		} else if (DrRightTrigger > 0.125) {
			// Loose Intake [Driver]
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kOff); // Compliant
			IO.DriveBase.ClawIntake.Set(1.0);

		} else {
			// Default Hold Cube
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward); // Closed
			IO.DriveBase.ClawIntake.Set(OpIntakeCommand + DrIntakeCommand);
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
		IO.DriveBase.SolenoidShifter.Set(false);

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
	}

	// Reset all the stuff that needs to be reset at each state
	void autoNextState() {
		autoModeState++;

		AutonTimer.Reset();
		autoSettleTimer.Reset();
		IO.DriveBase.EncoderLeft.Reset();
		IO.DriveBase.EncoderRight.Reset();

		stopMotors();
		IO.DriveBase.Wrist1.Set(0.0);
		IO.DriveBase.ClawIntake.Set(0.0);
	}

	bool bAutoNoData = false;
	void AutonomousPeriodic() {

		IO.DriveBase.HTower.Set(0.08);

		// Delay our auton program if required
		if (autoDelay == IO.DS.sAutoDelay3 and autoTotalTime.Get() < 3)
			return;
		if (autoDelay == IO.DS.sAutoDelay5 and autoTotalTime.Get() < 5)
			return;

		/*
		 // If no game data is received, run a fail safe program
		 if ((autoGameData.length() < 3) || bAutoNoData) {
		 if (autoTotalTime.Get() > 10) {
		 bAutoNoData = true;
		 printf("NO DATA FAILSAFE!!!");

		 if (autoMode == IO.DS.sAutoA) {
		 // Go left on switch, but don't eject.
		 autoCenter(false, true);
		 } else {
		 // Just Cross the line
		 autoLine();
		 }
		 }

		 return;
		 }
		 */

		// Cross the Line Auto
		if (autoMode == IO.DS.sAutoLine) {
			autoLine();
		}

		// 2 Cube Switch, Center Start
		if (autoMode == IO.DS.sAutoA) {
			if (autoGameData[0] == 'L') {
				autoCenter(false, true);
			}
			if (autoGameData[0] == 'R') {
				autoCenter(true, true);
			}
		}

		// 1 Cube Scale, Left Start
		if (autoMode == IO.DS.sAutoB) {
			if (autoGameData[1] == 'L')
				autoScaleNearCompat(false);

			if (autoGameData[1] == 'R')
				autoScaleFar(false, false);
		}

		// 2 Cube Near 1 Cube Far, Left Start
		if (autoMode == IO.DS.sAutoC) {
			if (autoGameData[1] == 'L') {
				autoScaleNear(false, true, false);
			}
			if (autoGameData[1] == 'R') {
				autoScaleFar(false, true);
			}
		}

		// 1 Cube Scale, 1 Cube Switch, Finish in center, Left Start
		if (autoMode == IO.DS.sAutoD) {
			if (autoGameData[1] == 'L') {
				if (autoGameData[0] == 'L') {
					autoScaleNear(false, false, true);
				}
				if (autoGameData[0] == 'R') {
					autoScaleNear(false, true, false);
				}
			}
			if (autoGameData[1] == 'R') {
				if (autoGameData[0] == 'L') {
					autoSwitchNearSide(false);
				}
				if (autoGameData[0] == 'R') {
					autoLinePlatformZone(false);
				}
			}
		}

		// 1 Cube Scale, 1 Cube Switch, Finish in center, Right Start
		if (autoMode == IO.DS.sAutoE) {
			if (autoGameData[1] == 'L') {
				if (autoGameData[0] == 'L') {
					autoScaleNear(true, false, true);
				}
				if (autoGameData[0] == 'R') {
					autoScaleNear(true, true, false);
				}
			}
			if (autoGameData[1] == 'R') {
				if (autoGameData[0] == 'L') {
					autoSwitchNearSide(true);
				}
				if (autoGameData[0] == 'R') {
					autoLinePlatformZone(true);
				}
			}
		}

		// F - 3 Cube Near Scale, 2 Far Scale, Left Start
		if (autoMode == IO.DS.sAutoF) {

			if (autoGameData[1] == 'L') {
				autoScaleNear3CUBE(false, false, false);
			}
			if (autoGameData[1] == 'R') {
				autoScaleFar(false, true);
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
			if (autoForward(16.0 * 10))
				autoNextState();
			break;

		default:
			stopMotors();
		}

		return;
	}

	void autoLinePlatformZone(bool isRightSide) {

		double rot = 1;
		if (isRightSide) {
			rot = -1;
		}

		switch (autoModeState) {
		case 1:
			if (autoForward(230))
				autoNextState();
			break;
		case 2:
			if (autoTurn(-90 * rot))
				autoNextState();
			break;
		case 3:
			if (autoForward(115 - 15))

				default:
			stopMotors();
		}

		return;
	}

	void autoCenter(bool isGoRight, bool autoCenter2Cube) {

		// Mirror path if starting on right
		double rot = 1;
		double CAoffset = 75 + 6;
		if (isGoRight) {
			rot = -1;
			CAoffset = 65 + 6;
		}

		// Closed Loop control of Elevator & Wrist
		elevatorPosition();
		wristPosition();

		switch (autoModeState) {
		case 1:
			// High gear
			IO.DriveBase.SolenoidShifter.Set(false);
			elevatorPosition(800);
			wristPosition(-25);

			//18
			if (autoForward(22 + (6) + 4 + 4 + 4)) {
				autoNextState();
			}
			break;

		case 2:
			if (autoTurn(45.0 * rot)) {
				autoNextState();
			}
			break;

		case 3:
			if (autoForward(CAoffset)) {
				autoNextState();
			}
			break;

		case 4:
			if (autoTurn(0.0) & elevatorPosition(5000)) {
				autoNextState();
			}
			break;

		case 5:
			//24 inches
			if (!isGoRight) {
				//Left
				if (autoForward(28 - (10) - 4 - 2 - 5) & wristPosition(-80) & wristNoPot(1.0, -0.57)) {
					autoNextState();
				}
			} else {
				//Right
				if (autoForward(28 - 10 - 4) & wristPosition(-80) & wristNoPot(1.0, -0.57)) {
					autoNextState();
				}
			}

			if (AutonTimer.Get() > 2.0) {
				autoNextState();
			}

			break;

		case 6:
			if (!bAutoNoData) {
				autoNextState();
			}
			break;

		case 7:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.65);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open

			// keep pushing!
			if (timedDrive(0.75, 0.2, 0.2)) {
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());
			}
			break;

		case 8:

			if (autoCenter2Cube == false) {
				autoNextState();
				autoModeState = 0;
			} else if (autoCenter2Cube == true) {
				autoNextState();
				autoModeState = 40;
			} else {

			}

			break;

		case 20:
			//
			// Start of wall hug path
			//
			if (wristPosition(0)) {
				if (elevatorPosition(800)) {
					if (autoForward(-48)) {
						autoNextState();
					}
				}
			}
			break;

		case 21:
			if (autoTurn(45 * rot) & elevatorPosition(800)) {
				autoNextState();
			}
			break;

		case 22:
			if (autoForward(72)) {
				autoNextState();
			}
			break;

		case 23:
			if (autoTurn(0)) {
				autoNextState();
			}
			break;

		case 24:
			if (autoForward(48)) {
				autoNextState();
			}
			break;

		case 40:
			//
			// Start of 2Cube
			//
			CAoffset = -60 - 4 + 4;
			if (isGoRight) {
				CAoffset = -65 + 8;
			}

			if (autoForward(CAoffset)) {
				autoNextState();
			}
			break;

		case 41:
			//-110
			if (!isGoRight) {
				//Left
				if (autoTurn((-45 - 5) * rot) & elevatorPosition(800) & wristPosition(-110 + 10 - 4)
						& wristNoPot(1.25, -0.65)) {
					autoNextState();
				}
			} else {
				//Right
				if (autoTurn((-45 - 10) * rot) & elevatorPosition(800) & wristPosition(-108-8)
						& wristNoPot(1.25, -0.65)) {
					autoNextState();

				}
			}

			break;

		case 42:
			IO.DriveBase.ClawIntake.Set(1.0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kOff); // Compliant
			if (!isGoRight) {
				//Left0
				if (autoForward(38 + 3 + 4)) {
					autoNextState();
				}
			} else {
				//Right
				if (autoForward(38 + 3 - 5 + 2)) {
					autoNextState();
				}
			}
			break;

		case 43:
			IO.DriveBase.ClawIntake.Set(1.0);
			IO.DriveBase.SolenoidShifter.Set(true);  // Low Gear
			if (timedDrive(1.25 - .5, 0.25, 0.25)) {
				IO.DriveBase.SolenoidShifter.Set(false); // High gear
				IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward); // Closed
				autoNextState();
			}
			break;

		case 44:

			if (!isGoRight) {
				//Left
				if (autoForward(-60) & wristPosition(-45) & wristNoPot(0.6, 0.75)) {
					autoNextState();
				}
			} else {
				//Right
				if (autoForward(-60 + 12) & wristPosition(-45) & wristNoPot(0.6, 0.75)) {
					autoNextState();
				}

			}
			break;

		case 45:
			IO.DriveBase.ClawIntake.Set(0.2);

			if (autoTurn(0) & elevatorPosition(5000)) {
				autoNextState();
			}
			break;

		case 46:
			if (!isGoRight) {
				//Left
				if (autoForward(68 - 10 + 2 + 3 + 4) & wristPosition(-80) & wristNoPot(1.0, -0.57)) {
					autoNextState();
				}
			} else {
				//Right
				if (autoForward(68 - 10 + 2 + 3) & wristPosition(-80) & wristNoPot(1.0, -0.57)) {
					autoNextState();
				}
			}
			if (AutonTimer.Get() > 2.0) {
				autoNextState();
			}
			break;

		case 47:
			if (true) {
				autoNextState();
			}
			break;

		case 48:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.65);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open

			// keep pushing!
			if (timedDrive(0.75, 0.2, 0.2)) {
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time2 [S]", autoTotalTime.Get());
			}

			break;

		case 49:
			if (autoForward(-36) && wristPosition(0)) {
				autoNextState();
			}
			break;

		case 50:
			elevatorPosition(800);

			if (autoTurn(45 * rot)) {
				autoNextState();
			}

			break;

		default:
			stopMotors();

		}

		return;
	}

	void autoScaleNearCompat(bool isStartRightPos) {

		// Mirror rotations for right side start
		double rot = 1;
		if (isStartRightPos) {
			rot = -1;
		}

		// Closed Loop control of Elevator & Wrist
		elevatorPosition();
		wristPosition();

		switch (autoModeState) {
		case 1:
			IO.DriveBase.SolenoidShifter.Set(false); // High Gear
			elevatorPosition(800);
			wristPosition(10);

			if (autoForward(275 + (12 + 6 + 18 - 3) + 8 - 8)) {
				autoNextState();
			}
			break;

		case 2:
			if (autoTurn(-80 * rot) & elevatorPosition(14500)) {
				autoNextState();
			}
			break;

		case 3:
			if (autoForward(10 + (12) - 6 - 2 - 6, 0.4, 0.0) & wristNoPot(0.6, -0.75)) {
				autoNextState();
			}

			if (AutonTimer.Get() > 2.0) {
				autoNextState();
			}
			break;

		case 4:
			if (wristPosition(-80)) {
				autoNextState();
			}
			break;

		case 5:
			if (AutonTimer.Get() > 0) {
				autoNextState();
			}
			break;

		case 6:
			// Eject!
			//IO.DriveBase.ClawIntake.Set(-0.65);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse);				// Open

			if (AutonTimer.Get() > 0.5) {
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());
			}

			break;

		case 7:
			if (wristPosition(0)) {
				elevatorPosition(800);
				autoNextState();
			}
			break;

		case 8:
			if (autoForward(-30 - 6, 0.45, 0.0)) {
				autoNextState();
			}
			break;

		default:
			stopMotors();

		}

		return;
	}

	void autoScaleNear(bool isStartRightPos, bool autoScaleNear2Cube, bool autoScaleNearSwitch) {

		// Mirror rotations for right side start
		double rot = 1;
		if (isStartRightPos) {
			rot = -1;
		}

		double scaleElevTarget = 15000;

		// Closed Loop control of Elevator & Wrist
		elevatorPosition();
		wristPosition();

		switch (autoModeState) {
		case 1:
			IO.DriveBase.SolenoidShifter.Set(false); // High Gear
			wristPosition(-25);

			//252
			if (autoForward(250, 1.0, 0.2)) {
				autoNextState();
			}
			break;

		case 2:
			elevatorPosition(scaleElevTarget);

			if (autoTurn(-35)) {
				autoNextState();
			}
			break;

		case 3:
			//28
			if (autoForward(28 + 6, 0.6, 0.1) & elevatorPosition(scaleElevTarget)) {
				autoNextState();
			}
			break;

		case 4:
			if (wristPosition(-80) & wristNoPot(1.0, -0.57)) {
				autoNextState();
			}
			break;

		case 5:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.5);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open

			if (AutonTimer.Get() > 0.2) {
				IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());
			}

			break;

		case 6:

			if (AutonTimer.Get() > 0.05) {
				if (autoForward(-26 + 13, 0.7, 0.1) & wristPosition(0.0)) {
					ElevPosTarget = 800;
					autoNextState();
				}
			}
			break;

		case 7:
			if (autoScaleNear2Cube == true or autoScaleNearSwitch) {
				autoModeState = 20;
			} else {
				autoModeState = 0; // We are done.
			}
			break;

		case 20:
			//
			// 2Cube Start
			//
			wristPosition(110);

			//32 - Was the angle before MSC
			// This controls how far we turn toward the second cube
			// Turns left to point the back (gearbox side of the robot) at the cube
			if (autoTurn((43 - 15 - 7) * rot) && elevatorPosition(800)) {
				autoNextState();
			}
			break;

		case 21:
			// Turns intake on (wheels spinning, clamp compliant
			// Moves the robot 50 inches backwards into the cube
			IO.DriveBase.ClawIntake.Set(1.0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kOff);				// Compliant

			if (autoForward(-57) & wristNoPot(3.0, 0.8)) {
				autoNextState();
			}
			break;

		case 22:
			// Switches robot into low gear
			// Time drives the cube to ensure seating
			// Switches back to high
			//IO.DriveBase.SolenoidShifter.Set(true);  // Low Gear
			//if (timedDrive(1.0, -0.3, -0.3)) {
			//IO.DriveBase.SolenoidShifter.Set(false); // High gear
			autoNextState();
			//}
			break;

		case 23:
			// Clamps claw down, hopefully on cube
			// Turns intake down to 0.7 so we are still pulling the cube in
			// Waits 0.65 seconds for all that stuff to happen.
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward);				// Closed
			IO.DriveBase.ClawIntake.Set(0.7);

			if (timedDrive(0.5, -0.3, -0.3)) {
				if (autoScaleNearSwitch == false) {
					autoNextState();
				}

				if (autoScaleNearSwitch == true) {
					autoModeState = 40;
				}
			}

			break;

		case 24:
			// Continue running intake slowly, drive forward 60 inches, and flip the wrist to 45 degrees.
			IO.DriveBase.ClawIntake.Set(0.5);
			wristPosition(-45);

			if (autoForward(15, 0.6, 0.1)) {
				autoNextState();
			}
			break;

		case 25:
			if (autoTurn(-10 * rot) & elevatorPosition(scaleElevTarget)) {
				autoNextState();
			}
			break;

		case 26: // drive forward toward the scale
			if (autoForward(48, 0.5, 0.1)) {
				autoNextState();
			}
			break;

		case 27:
			if (wristPosition(-80) & wristNoPot(2.5, -0.67)) {
				autoNextState();
			}
			break;

		case 28:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.65);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open

			if (AutonTimer.Get() > 1.0) {
				IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time2 [S]", autoTotalTime.Get());
			}

			break;

		case 29:
			//IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward); // Close

			if (autoTurn(0) & wristPosition(0) & wristNoPot(2.0, 0.35)) {
				autoNextState();
				IO.DriveBase.ClawIntake.Set(0.0);
				elevatorPosition(800);
			}
			break;

		case 40:
			// Begin switch score sequence
			// Back off the switch after grabbing cube
			IO.DriveBase.ClawIntake.Set(0);

			if (autoForward(12, 0.6, 0.1)) {
				autoNextState();
			}
			break;

		case 41:
			// Lift the elevator and turn to the left to prepare to score in switch
			if (autoTurn(25 * rot) & (wristPosition(80) & elevatorPosition(5000))) {
				autoNextState();
			}
			break;

		case 42:
			// drive forward to the switch
			if (autoForward(-25 - 15 + 7, 0.6, 0.1)) {
				autoNextState();
			}
			if (AutonTimer.Get() > 2.0) {
				autoNextState();
			}
			break;

		case 43:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.65);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse);				// Open
			// keep pushing!
			if (timedDrive(0.75, 0.2, 0.2)) {
				IO.DriveBase.ClawIntake.Set(0.0);
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

	void autoScaleNear3CUBE(bool isStartRightPos, bool autoScaleNear2Cube, bool autoScaleNearSwitch) {

		// Mirror rotations for right side start
		/*double rot = 1;
		 if (isStartRightPos) {
		 rot = -1;
		 }
		 */

		double scaleElevTarget = 15000;

		// Closed Loop control of Elevator & Wrist
		elevatorPosition();
		wristPosition();

		double wristAngle = (IO.DriveBase.WristPot.Get() + m_WristOffset) * WristScale;
		switch (autoModeState) {
		case 1:

			IO.DriveBase.SolenoidShifter.Set(false); // High Gear
			IO.DriveBase.ClawIntake.Set(0.3);

			if (getEncoderDistance() > 150) {
				autoHeading = 20;
			}

			if (getEncoderDistance() > 180) {
				elevatorPosition(scaleElevTarget);
			}

			if (IO.DriveBase.EncoderElevator.Get() > 13000) {
				wristPosition(-80);
			} else {
				wristPosition(-25);
			}

			if (autoForward(250 + 24 + 5 - 3) & wristPosition() & elevatorPosition()) {
				autoNextState();
			}
			break;

		case 2:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.6);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open

			if (AutonTimer.Get() > 0.2) {
				IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());

			}

			break;

		case 3:
			wristPosition(110);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open

			if (wristAngle > -45) {
				elevatorPosition(800);
				IO.DriveBase.ClawIntake.Set(1.0);
			}

			autoHeading = -7-5;

			if (autoForward(-40 - 2) & elevatorPosition() & wristPosition()) {
				autoNextState();
			}
			break;

		case 4:
			IO.DriveBase.ClawIntake.Set(1.0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward); // Closed

			if (timedDrive(0.5 + 0.2, -0.4, -0.4)) {
				autoNextState();
			}
			break;

		case 5:
			IO.DriveBase.ClawIntake.Set(0.3);
			elevatorPosition(scaleElevTarget);

			if (IO.DriveBase.EncoderElevator.Get() > 13000) {
				wristPosition(-80);
			} else {
				wristPosition(-15);
			}

			if (getEncoderDistance() > 30) {
				autoHeading = 20;
			}

			if (autoForward(40 + 12 + 8 + 3) & elevatorPosition() & wristPosition()) {
				autoNextState();
			}

			break;

		case 6:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.6);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open

			if (AutonTimer.Get() > 0.2) {
				IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time2 [S]", autoTotalTime.Get());
			}

			break;

		case 7:
			wristPosition(100);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open

			if ((wristAngle > -45) || (getEncoderDistance() > 18)) {
				elevatorPosition(800);
				IO.DriveBase.ClawIntake.Set(1.0);
			}

			autoHeading = -30;

			if (autoForward(-42 - 28 + 2) & elevatorPosition() & wristPosition()) {
				autoNextState();
			}
			break;

		case 8:
			IO.DriveBase.ClawIntake.Set(1.0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward); // Closed

			if (timedDrive(0.6, -0.35, -0.35)) {
				autoNextState();
			}
			break;

		case 9:
			IO.DriveBase.ClawIntake.Set(0.3);
			elevatorPosition(scaleElevTarget);

			if (IO.DriveBase.EncoderElevator.Get() > 13000) {
				wristPosition(-75);
			} else {
				wristPosition(-15);
			}

			if (getEncoderDistance() > 40) {
				autoHeading = 15;
			} else if (getEncoderDistance() > 30) {
				autoHeading = 0;
			}

			if (autoForward(60 + 8 + 8 + 8) & elevatorPosition() & wristPosition()) {
				autoNextState();
			}

			break;

		case 10:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.2);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open

			if (AutonTimer.Get() > 0.2) {
				IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time3 [S]", autoTotalTime.Get());
			}

			break;

		case 11:
			wristPosition(110);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open

			if ((wristAngle > -45) || (getEncoderDistance() > 18)) {
				elevatorPosition(800);
				IO.DriveBase.ClawIntake.Set(1.0);
			}

			autoHeading = -45 - 10 + 3 + 3 - 3;

				if (autoForward(-80 + 6 + 2 - 10) & elevatorPosition() & wristPosition()) {
					autoNextState();
				}

			break;

		case 12:
			IO.DriveBase.ClawIntake.Set(1.0);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open Claw

			if (timedDrive(0.75, -0.45, -0.45)) {
				autoNextState();
				autoModeState = 99;
			}
			break;

		case 13:
			IO.DriveBase.ClawIntake.Set(0.3);
			elevatorPosition(scaleElevTarget);

			if (IO.DriveBase.EncoderElevator.Get() > 13000) {
				wristPosition(-75);
			} else {
				wristPosition(-15);
			}

			if (getEncoderDistance() > 60) {
				autoHeading = 15;
			} else if (getEncoderDistance() > 45) {
				autoHeading = 0;
			}

			if (autoForward(72 + 15 + 10 + 8 - 8) & elevatorPosition() & wristPosition()) {
				autoNextState();
			}

			break;

		case 14:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.4);
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open

			if (AutonTimer.Get() > 0.2) {
				IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time4 [S]", autoTotalTime.Get());
			}

			break;
		default:
			stopMotors();

		}

		return;
	}

	void autoScaleFar(bool isRightSide, bool autoScaleFar2Cube) {

		// Mirror rotations for right side start
		double rot = 1;
		if (isRightSide) {
			rot = -1;
		}

		// Closed Loop control of Elevator & Wrist
		elevatorPosition();
		wristPosition();

		double wristAngle = (IO.DriveBase.WristPot.Get() + m_WristOffset) * WristScale;

		// Auto State Machine
		switch (autoModeState) {
		case 1:
			IO.DriveBase.SolenoidShifter.Set(false); // High gear
			wristPosition(-25);

			//232
			if (autoForward(232, 1.0, 0.2)) {
				autoNextState();
			}
			break;

		case 2:
			if (autoTurn(-90.0 * rot)) {
				autoNextState();
			}
			break;

		case 3:
			//187 (RJ) ? 222 (BullDogs) 218
			if (autoForward(230 - 10 - 4)) {
				autoNextState();
			}
			break;

		case 4:
			elevatorPosition(15500);
			if (autoTurn((30 - 10) * rot)) {
				autoNextState();
			}
			break;

		case 5:
			//32 inches forward
			if (autoForward(32 + 8 + 4, 0.5, 0) & elevatorPosition(15500)) {
				autoNextState();
			}
			break;

		case 6:
			//90 but hit scale
			if (wristPosition(-80) & wristNoPot(1.0, -0.65)) {
				autoNextState();
			}
			break;

		case 7:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.65 + 0.15);

			if (AutonTimer.Get() > 0.5) {
				IO.DriveBase.ClawIntake.Set(0.0);
				IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());
			}
			break;

		case 8:
			// Go to 2CUBE  if requested
			if (autoScaleFar2Cube == true) {
				autoModeState = 20;
			}

			if (autoForward(-36, 0.5, 0.1) && wristPosition(0)) {
				ElevPosTarget = 800;
				autoNextState();
				autoModeState = 0; // We are done.
			}
			break;

		case 20:
			//
			// 2Cube Start
			//
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open
			IO.DriveBase.ClawIntake.Set(1.0);

			autoHeading = 20;
			wristPosition(110);

			if (wristAngle > -45) {
				elevatorPosition(800);
			}

			if (autoForward(-20) & wristPosition(110) & elevatorPosition()) {
				autoNextState();
			}
			break;

		case 21:

			if (true) {
				autoNextState();
			}
			break;

		case 22:
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kForward); // Closed
			IO.DriveBase.ClawIntake.Set(1.0);
			if (timedDrive(0.75, -0.4, -0.4)) {
				autoNextState();
			}
			break;

		case 23:
			elevatorPosition(15500);
			wristPosition(-80);
			IO.DriveBase.ClawIntake.Set(0.3);

			if (getEncoderDistance() > 15) {
				autoHeading = -15;
			}

			if (autoForward(40, 0.6, 0.0)) {
				autoNextState();
			}
			break;

		case 24:
			if (true) {
				autoNextState();
			}
			break;

		case 25:
			// Eject!
			IO.DriveBase.ClawIntake.Set(-0.5);

			if (AutonTimer.Get() > 0.5) {
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time2 [S]", autoTotalTime.Get());
			}
			break;

		case 26:
			if (wristPosition(0)) {
				autoNextState();
				ElevPosTarget = 800;
			}
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
		elevatorPosition();

		// Rotate based on field start position
		double rotDir = 1.0;
		if (isStartRightPos) {
			rotDir = -1.0;
		}

		// Auto Sequence
		switch (autoModeState) {
		case 1:
			IO.DriveBase.SolenoidShifter.Set(false); //High Gear

			if (autoForward(145 + 18)) {
				autoNextState();
			}
			break;

		case 2:
			if (autoTurn(-90 * rotDir, 0.5, 0.1) & elevatorPosition(3000) & wristNoPot(0.6, -0.75)) {
				autoNextState();
			}
			break;

		case 3:
			IO.DriveBase.SolenoidShifter.Set(true); // Low Gear

			if (timedDrive(0.6, 0.65, 0.65)) {
				autoNextState();
			}
			break;

		case 4:
			wristPosition(-60);

			if (AutonTimer.Get() > 0.5) {
				autoNextState();
			}
			break;

		case 5:
			IO.DriveBase.ClawClamp.Set(frc::DoubleSolenoid::kReverse); // Open
			IO.DriveBase.ClawIntake.Set(-0.65);

			if (timedDrive(0.75, 0.2, 0.2)) {
				IO.DriveBase.ClawIntake.Set(0.0);
				autoNextState();

				// Display auton Time
				SmartDashboard::PutNumber("Auto Time [S]", autoTotalTime.Get());
			}
			break;

		case 6:
			wristPosition(0);

			if (autoForward(-26)) {
				autoNextState();
			}
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

		// Set target to the commanded position
		ElevPosTarget = Elev_position;

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

	// Default to hold last commanded position
	bool elevatorPosition() {
		return elevatorPosition(ElevPosTarget);
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
		double wristAngle = (IO.DriveBase.WristPot.Get() + m_WristOffset) * WristScale;
		double softLimit = 115;
		double hardLimit = 120;

		// Slow down if approaching limits
		if (wristAngle < -softLimit and input < 0) {
			input *= 0.5;
		}

		if (wristAngle > softLimit and input > 0) {
			input *= 0.5;
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
#define Wrist_KD (0)
	double prevError_wrist = 0;
	bool wristPosition(double input) {

		if (PotDisabled == IO.DS.DisabledPOT) {
			return true;
		}

		// Update Target Position
		WristTarget = input;

		// Get Current Encoder Value
		double wristAngle = (IO.DriveBase.WristPot.Get() + m_WristOffset) * WristScale;

		// Motor Command Calculation
		double error = input - wristAngle;

		//D Term
		double dError = (error - prevError_wrist) / MAIN_LOOP_PERIOD;		// [Inches/second]
		prevError_wrist = error;

		double cmd = error * Wrist_KP + dError * Wrist_KD;

		//Limit Elevator Max Speed
		cmd = absMax(cmd, Wrist_MAXSpeed);

		// Set the wrist speed
		wristSpeed(cmd);

		// Check if we made it to the target
		return (fabs(error) <= WristPositionTol);
	}

	// Default to holding whatever position was last commanded
	bool wristPosition() {
		return wristPosition(WristTarget);
	}

	bool wristNoPot(double time, double speed) {
		if (PotDisabled == IO.DS.DisabledPOT) {
			if (AutonTimer.Get() < time) {
				IO.DriveBase.Wrist1.Set(speed);
			} else {
				IO.DriveBase.Wrist1.Set(0.0);
				return true;
			}
		} else {
			return true;
		}
		return false;
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
		IO.DriveBase.MotorsLeft.Set(0);
		IO.DriveBase.MotorsRight.Set(0);
		OutputY = 0;
		OutputX = 0;
		return 1;
	}

	// Go AutoForward autonomously...

#define KP_LINEAR (0.0311)
#define KI_LINEAR (0.000)
#define KD_LINEAR (0.008-0.002)
#define LINEAR_TOLERANCE (1.0)

#define KP_ROTATION (0.0700+0.01)
#define KI_ROTATION (0.0000)
#define KD_ROTATION (0.0080)

#define ROTATION_TOLERANCE (10.0)
#define ROTATIONAL_SETTLING_TIME (0.0)
#define ROTATIONAL_MAX_SPEED (0.7)

	double prevError_linear = 0; // Derivative Calculation
	double sumError_linear = 0;  // Integral Calculation

	double prevError_rotation = 0; // Derivative Calculation
	double sumError_rotation = 0;  // Integral Calculation

	int autoForward(double targetDistance, double max_speed, double settle_time) {

		double encoderDistance = getEncoderDistance();

		// P Control
		double error = targetDistance - encoderDistance;  // [Inches]

		// I Control
		if (error < 24) {
			sumError_linear += error / MAIN_LOOP_PERIOD;
		} else {
			sumError_linear = 0;
		}

		// D Control
		double dError = (error - prevError_linear) / MAIN_LOOP_PERIOD;  // [Inches/second]
		prevError_linear = error;

		// PID Command
		double driveCommandLinear = error * KP_LINEAR + KI_LINEAR * sumError_linear + KD_LINEAR * dError;

		// limit max drive speed
		driveCommandLinear = absMax(driveCommandLinear, max_speed);

		// Min speed
		driveCommandLinear = absMin(driveCommandLinear, 0.11);

		// Use Gyro to drive straight
		double gyroAngle = IO.DriveBase.ahrs.GetAngle();
		double error_rot = gyroAngle - autoHeading;

		// I Control
		if (error_rot < 15) {
			sumError_rotation += error_rot / MAIN_LOOP_PERIOD;
		} else {
			sumError_rotation = 0;
		}

		// D Control
		double dError_rot = (error_rot - prevError_rotation) / MAIN_LOOP_PERIOD;  // [Inches/second]
		prevError_rotation = error_rot;

		double driveCommandRotation = error_rot * KP_ROTATION + KI_ROTATION * sumError_rotation
				+ KD_ROTATION * dError_rot;
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

		// Use Gyro to drive straight
		double gyroAngle = IO.DriveBase.ahrs.GetAngle();
		double error_rot = gyroAngle - autoHeading;

		// I Control
		if (error_rot < 15) {
			sumError_rotation += error_rot / MAIN_LOOP_PERIOD;
		} else {
			sumError_rotation = 0;
		}

		// D Control
		double dError_rot = (error_rot - prevError_rotation) / MAIN_LOOP_PERIOD;  // [Inches/second]
		prevError_rotation = error_rot;

		double driveCommandRotation = error_rot * KP_ROTATION + KI_ROTATION * sumError_rotation
				+ KD_ROTATION * dError_rot;
		driveCommandRotation = absMax(driveCommandRotation, maxSpeed);

		// dooo it!
		motorSpeed(-driveCommandRotation, driveCommandRotation);

		// Allow for the robot to settle into position
		if (abs(error_rot) > ROTATION_TOLERANCE)
			autoSettleTimer.Reset();

		else if (autoSettleTimer.Get() > settlingTime)
			return 1;

		return 0;
	}

	int autoTurn(float targetYaw) {
		return autoTurn(targetYaw, ROTATIONAL_MAX_SPEED, ROTATIONAL_SETTLING_TIME);
	}

	int autoTurn() {
		return autoTurn(autoHeading, ROTATIONAL_MAX_SPEED, ROTATIONAL_SETTLING_TIME);
	}

	int timedDrive(double driveTime, double leftMotorSpeed, double rightMotorSpeed) {

		if (AutonTimer.Get() < driveTime) {

			// Use Gyro to drive straight
			double gyroAngle = IO.DriveBase.ahrs.GetAngle();
			double error_rot = gyroAngle - autoHeading;

			// I Control
			if (error_rot < 15) {
				sumError_rotation += error_rot / MAIN_LOOP_PERIOD;
			} else {
				sumError_rotation = 0;
			}

			// D Control
			double dError_rot = (error_rot - prevError_rotation) / MAIN_LOOP_PERIOD;  // [Inches/second]
			prevError_rotation = error_rot;

			double driveCommandRotation = error_rot * KP_ROTATION + KI_ROTATION * sumError_rotation
					+ KD_ROTATION * dError_rot;
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

		if (input == 0.0)
			return 0.0;

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
		SmartDashboard::PutString(llvm::StringRef("Auto Mode Selection"), llvm::StringRef(autoMode));
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
		SmartDashboard::PutString("POT State", IO.DS.choosePotDisabled.GetSelected());
		SmartDashboard::PutNumber("Wrist AI", IO.DriveBase.WristAI.GetVoltage());
		SmartDashboard::PutNumber("Wrist Pot", (IO.DriveBase.WristPot.Get() + m_WristOffset) * WristScale);

		// Sensor Override
		SmartDashboard::PutBoolean("Sensor Override", SensorOverride);

	}

}
;

START_ROBOT_CLASS(Robot);

