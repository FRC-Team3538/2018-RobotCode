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




		SendableChooser<std::string> chooseAutoProgram;
		const std::string sAuto0 = "No_Auto";
		const std::string sAuto1 = "Auto_1";
		const std::string sAuto2 = "Auto_2";
		const std::string sAuto3 = "Auto_3";

		SendableChooser<std::string> chooseAutoDelay;
		const std::string sAutoDelayOff = "No_Delay";
		const std::string sAutoDelay3 = "3 Seconds";
		const std::string sAutoDelay5 = "5 Seconds";

		SendableChooser<std::string> chooseAutoSelected;
		const std::string AutoLeftSpot = "Left";
		const std::string AutoCenterSpot = "Center";
		const std::string AutoRightSpot = "Right";

		SendableChooser<std::string> chooseDriveEncoder;
		const std::string  EncoderLeft  = "Left_Encoder";
		const std::string EncoderRight = "Right_Encoder";

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

		// NavX MXP board (Gryo)
		//AHRS *ahrs;
		AHRS ahrs { SPI::Port::kMXP, 200 };

		//Rest of the robot

		//Elevator
		VictorSP Elevator1 { 6 };
		VictorSP Elevator2 { 7 };



		//Elevator Sensing Package
		Encoder EncoderElevator { 4, 5 };
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
		VictorSP Claw1 { 9 };
		VictorSP Claw2 { 10 };
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
		DigitalInput DiIn8 { 8 }, DiIn9 { 9 };

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
