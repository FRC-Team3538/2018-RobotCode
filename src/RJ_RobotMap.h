#ifndef _RJ_ROBOTMAP_H_
#define _RJ_ROBOTMAP_H_

#include "WPILib.h"
#include "AHRS.h"
#include "ctre/phoenix.h"
#include "math.h"

class RJ_RobotMap {

public:

	// Driver's Station
	struct structDS {

		XboxController DriveStick { 0 };
		XboxController OperatorStick { 1 };
		Joystick LaunchPad { 4 };

		SendableChooser<llvm::StringRef> chooseController;
		const llvm::StringRef PS = "PS4";
		const llvm::StringRef xBox = "xBox";

		SendableChooser<llvm::StringRef> chooseAutoGameData;
		const llvm::StringRef sGameDataOff = "Off";
		const llvm::StringRef sGameDataLefts = "LLL";
		const llvm::StringRef sGameDataRights = "RRR";

		SendableChooser<llvm::StringRef> chooseAutoDelay;
		const llvm::StringRef sAutoDelayOff = "No_Delay";
		const llvm::StringRef sAutoDelay3 = "3 Seconds";
		const llvm::StringRef sAutoDelay5 = "5 Seconds";

		SendableChooser<llvm::StringRef> chooseAutoMode;
		const llvm::StringRef sAutoNone = "1 - None";
		const llvm::StringRef sAutoLine = "2 - Line";
		const llvm::StringRef sAutoA = "A - 2 Cube Switch, Center Start";
		const llvm::StringRef sAutoB = "B - 1 Cube Scale, Left Start";
		const llvm::StringRef sAutoC = "C - 2 Cube Near Scale 1 Far Scale, Left Start";
		const llvm::StringRef sAutoD = "D - Compatible Near Objectives, Left Start";
		const llvm::StringRef sAutoE = "E - Compatible Near Objectives, Right Start";
		const llvm::StringRef sAutoF = "F - 3 Cube Near Scale 2 Far Scale, Left Start";
		const llvm::StringRef sAutoG = "G - Single compatible cube (priority: scale, switch, platform zone, Left Start";
		const llvm::StringRef sAutoH = "H - Single compatible cube (priority: scale, switch, platform zone, Right Start";
		const llvm::StringRef sAutoTEST = "Z - TEST CASE AUTO";
		const llvm::StringRef sAutoTEST2 = "Y - TEST CASE AUTO 2";

		SendableChooser<llvm::StringRef> chooseAutoEncoder;
		const llvm::StringRef EncoderAuto = "Enc Auto";
		const llvm::StringRef EncoderLeft = "Enc Left";
		const llvm::StringRef EncoderRight = "Enc Right";
		const llvm::StringRef EncoderBoth = "Enc Both";
		const llvm::StringRef EncoderNone = "Enc None";

		SendableChooser<llvm::StringRef> choosePotDisabled;
		const llvm::StringRef EnabledPOT = "1 Enable Pot";
		const llvm::StringRef DisabledPOT = "2 Disable Pot";

	};
	structDS DS;

	// Drive Base
	struct structDriveBase {
		// Left Motors
		VictorSPX L1 { 0 };
		WPI_TalonSRX L2 { 1 };
		VictorSPX L3 { 2 };
		//SpeedControllerGroup MotorsLeft { L1, L2, L3 };

		// Right Motors
		VictorSPX R1 { 3 };
		WPI_TalonSRX R2 { 4 };
		VictorSPX R3 { 5 };
		//SpeedControllerGroup MotorsRight { R1, R2, R3 };

		// Drive Base Encoders
		Encoder EncoderLeft { 0, 1 };
		Encoder EncoderRight { 2, 3 };

		// Shifting Solenoid
		Solenoid SolenoidShifter { 0 };
		//Solenoid HookDeploy { 3 };

		// NavX MXP board (Gryo)
		//AHRS *ahrs;
		AHRS ahrs { SPI::Port::kMXP, 200 };
		//AHRS ahrs { I2C::Port::kMXP, 200 };

		//Rest of the robot

		//Elevator
		WPI_TalonSRX Elevator1 { 6 };
		VictorSPX Elevator2 { 7 };

		//Elevator Sensing Package
		Encoder EncoderElevator { 4, 5, false, Encoder::k4X };
		DigitalInput SwitchElevatorLower { 8 };
		DigitalInput SwitchElevatorUpper { 9 };
		AnalogInput PotentiometerElevator { 0 };

		//Wrist
		VictorSPX Wrist1 { 8 };

		// Wrist Sensing Package

		Encoder EncoderWrist { 6, 7 };
		DigitalInput SwitchWrist1 { 15 };
		DigitalInput SwitchWrist2 { 16 };

		//AnalogInput PotentiometerWrist { 1 };
		// Dio are faster that pwm so make sure that these do not have the same port as victors
		// when talking to the navx other wise the pwm signal will not be sent.
		AnalogInput WristAI { 3 };
		AnalogPotentiometer WristPot { &WristAI, 270, -270 / 2 };

		//Claw
		VictorSPX ClawIntake { 9 };
		VictorSPX ClawIntake2 { 10 };
		//DigitalInput IntakeSwitch1 { 6 };
		//DigitalInput IntakeSwitch2 { 7 };
		//SpeedControllerGroup ClawIntake { ClawIntake1, ClawIntake2 };
		DoubleSolenoid ClawClamp { 1, 2 };

		//VictorSP Winch1 { 11 };
		//VictorSP Winch2 { 12 };
		//SpeedControllerGroup Winches { Winch1, Winch2 };

		//VictorSP HTower { 13 };


		//LED Control
		Relay LED0 { 0 };
		Relay LED1 { 1 };
		Relay LED2 { 2 };
		Relay LED3 { 3 };

	};
	structDriveBase DriveBase;

	// Drive Base

	 struct structVision {
	 cs::UsbCamera cam0 = CameraServer::GetInstance()->StartAutomaticCapture("Camera 0", 0);
	// cs::UsbCamera cam1 = CameraServer::GetInstance()->StartAutomaticCapture("Camera 1", 1);

	 cs::VideoSink server = CameraServer::GetInstance()->GetServer();

	 cs::CvSink sink0;
	// cs::CvSink sink1;
	 };
	 structVision Vision;


	// Default Constructor
	RJ_RobotMap();

	// Send all of the NavX data to the SmartDashboard
	void NavXDebugger();

};

#endif
