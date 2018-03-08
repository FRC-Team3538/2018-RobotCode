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


		SendableChooser<std::string> chooseAutoDelay;
		const std::string sAutoDelayOff = "No_Delay";
		const std::string sAutoDelay3 = "3 Seconds";
		const std::string sAutoDelay5 = "5 Seconds";

		SendableChooser<std::string> chooseAutoFinisher;
		const std::string sAutoYes = "Hell Yeah";
		const std::string sAutoNo = "No";

		SendableChooser<std::string> chooseAutoPosStart;
		const std::string sAutoCenter = "Center";
		const std::string sAutoLeft = "Left";
		const std::string sAutoRight = "Right";

		SendableChooser<std::string> chooseAutoProgram;
		const std::string AutoNone = "None";
		const std::string AutoLine = "Line";
		const std::string AutoSwitch = "Switch";
		const std::string AutoScale = "Scale";
		const std::string AutoNearest = "Nearest";

		SendableChooser<std::string> chooseAutoEncoder;
		const std::string EncoderAuto  = "Enc Auto";
		const std::string EncoderLeft  = "Enc Left";
		const std::string EncoderRight = "Enc Right";
		const std::string EncoderBoth  = "Enc Both";
		const std::string EncoderNone  = "Enc None";

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
		Encoder EncoderWrist { 6, 7 };
		DigitalInput SwitchWrist1 { 10 };
		DigitalInput SwitchWrist2 { 11 };
		AnalogInput PotentiometerWrist { 1 };

		//Claw
		VictorSP ClawIntake1 { 9 };
		VictorSP ClawIntake2 { 10 }; // TODO: Probably Unused
		DoubleSolenoid ClawClamp { 1, 2 };

		//LED Control
		Relay LED0 { 0 };
		Relay LED1 { 1 };
		Relay LED2 { 2 };
		Relay LED3 { 3 };

	};
	structDriveBase DriveBase;


	// Default junk for testing
	struct structTesting {
		VictorSP RightStick1 { 6 };
		VictorSP RightStick2 { 7 };
		VictorSP Dpad1 { 8 };
		VictorSP Dpad2 { 9 };

		// Solenoids
		Solenoid IntakeButton { 1 };
		Solenoid Abutton { 2 };
		Solenoid Bbutton { 3 };
		Solenoid XYbutton { 4 };

		// Limit Switches
		//DigitalInput DiIn8 { 8 }, DiIn9 { 9 };

		//Encoder Init
		//bool useRightEncoder = false;
	};
	structTesting TestJunk;


	// Default Constructor
	RJ_RobotMap();

	// Send all of the NavX data to the SmartDashboard
	void NavXDebugger();

};

#endif
