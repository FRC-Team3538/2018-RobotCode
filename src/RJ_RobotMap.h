#ifndef _RJ_ROBOTMAP_H_
#define _RJ_ROBOTMAP_H_

#include "WPILib.h"
#include "AHRS.h"

class RJ_RobotMap {

public:

	// Driver's Station
	struct structDS {
		// TODO: Change to xbox controller datatype
		Joystick DriveStick { 0 };
		Joystick OperatorStick { 1 };

		SendableChooser<std::string> chooseAutoProgram;
		const std::string sAuto0 = "No_Auto";
		const std::string sAuto1 = "Auto_1";
		const std::string sAuto2 = "Auto_2";
		const std::string sAuto3 = "Auto_3";


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
		AHRS ahrs {SerialPort::kMXP};
	};
	structDriveBase DriveBase;


	// Default junk for testing
	struct structTesing {
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
		DigitalInput DiIn8{8}, DiIn9{9};
	};
	structTesing TestJunk;

	// Default Constructor
	RJ_RobotMap();

	// Send all of the NavX data to the SmartDashboard
	void NavXDebugger();

};

#endif
