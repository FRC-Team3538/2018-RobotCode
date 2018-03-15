#ifndef _RJ_ROBOTMAP_H_
#define _RJ_ROBOTMAP_H_

#include "WPILib.h"
#include "AHRS.h"

#include "math.h"
#include "Encoder.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

class RJ_RobotMap {

public:

	// Driver's Station
	struct structDS {

		XboxController DriveStick { 0 };
		XboxController OperatorStick { 1 };
		Joystick LaunchPad { 4 };


		SendableChooser<llvm::StringRef> chooseAutoDelay;
		const llvm::StringRef sAutoDelayOff = "No_Delay";
		const llvm::StringRef sAutoDelay3 = "3 Seconds";
		const llvm::StringRef sAutoDelay5 = "5 Seconds";

		SendableChooser<llvm::StringRef> chooseAutoPosStart;
		const llvm::StringRef sAutoCenter = "Center";
		const llvm::StringRef sAutoLeft = "Left";
		const llvm::StringRef sAutoRight = "Right";

		SendableChooser<llvm::StringRef> chooseAutoProgram;
		const llvm::StringRef AutoNone = "1 None";
		const llvm::StringRef AutoLine = "2 Line";
		const llvm::StringRef AutoSwitch = "3 Switch";
		const llvm::StringRef AutoScale = "4 Scale";
		const llvm::StringRef AutoNearSide = "5 NSc NSw L";
		const llvm::StringRef AutoNNF = "6 NSc NSw FSc";
		const llvm::StringRef AutoArcSwitch = "7 ArcSwitch";

		SendableChooser<llvm::StringRef> chooseAutoFinisher;
		const llvm::StringRef sAutoNo = "1 None";
		const llvm::StringRef sAutoCube2Get = "2 Cube2Get";
		const llvm::StringRef sAutoCube2Score = "3 Cube2Score";
		const llvm::StringRef sAutoWallHug = "4 Wall Hug";

		SendableChooser<llvm::StringRef> chooseAutoEncoder;
		const llvm::StringRef EncoderAuto  = "Enc Auto";
		const llvm::StringRef EncoderLeft  = "Enc Left";
		const llvm::StringRef EncoderRight = "Enc Right";
		const llvm::StringRef EncoderBoth  = "Enc Both";
		const llvm::StringRef EncoderNone  = "Enc None";
	};
	structDS DS;

	// Drive Base
	struct structDriveBase {
		// Left Motors
		VictorSP L1 { 0 };
		VictorSP L2 { 1 };
		VictorSP L3 { 2 };
		SpeedControllerGroup MotorsLeft { L1, L2, L3 };

		// Right Motors
		VictorSP R1 { 3 };
		VictorSP R2 { 4 };
		VictorSP R3 { 5 };
		SpeedControllerGroup MotorsRight { R1, R2, R3 };



		// Drive Base Encoders
		Encoder EncoderLeft { 0, 1 };
		Encoder EncoderRight { 2, 3 };

		// Shifting Solenoid
		Solenoid SolenoidShifter { 0 };
		Solenoid Zbar { 3 };
		Solenoid Zbar1 { 4 };

		// NavX MXP board (Gryo)
		//AHRS *ahrs;
		AHRS ahrs { SPI::Port::kMXP, 200 };

		//Rest of the robot

		//Elevator
		VictorSP Elevator1 { 6 };
		VictorSP Elevator2 { 7 };



		//Elevator Sensing Package
		Encoder EncoderElevator { 4, 5, false, Encoder::k4X };
		DigitalInput SwitchElevatorLower { 8 };
		DigitalInput SwitchElevatorUpper { 9 };
		AnalogInput PotentiometerElevator { 0 };

		//Wrist
		VictorSP Wrist1 { 8 };

		// Wrist Sensing Package
		AnalogPotentiometer WristPot { 0, 270, -270 / 2};

		//Claw
		VictorSP ClawIntake1 { 9 };
		VictorSP ClawIntake2 { 10 };
		DigitalInput IntakeSwitch1 { 6 };
		DigitalInput IntakeSwitch2 { 7 };
		SpeedControllerGroup ClawIntake { ClawIntake1, ClawIntake2 };
		DoubleSolenoid ClawClamp { 1, 2 };

		VictorSP Winch1 { 11 };
		VictorSP Winch2 { 12 };
		SpeedControllerGroup Winches { Winch1, Winch2 };

		//LED Control
		Relay LED0 { 0 };
		Relay LED1 { 1 };
		Relay LED2 { 2 };
		Relay LED3 { 3 };

	};
	structDriveBase DriveBase;

	// Default Constructor
	RJ_RobotMap();

	// Send all of the NavX data to the SmartDashboard
	void NavXDebugger();

};

#endif
