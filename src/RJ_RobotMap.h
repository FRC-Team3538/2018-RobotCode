
#ifndef _RJ_ROBOTMAP_H_
#define _RJ_ROBOTMAP_H_

#include "WPILib.h"

class RJ_RobotMap {

public:

	// Driver's Station
	struct structDS{
		Joystick* DriveStick;
		Joystick* OperatorStick;

		frc::SendableChooser<std::string> chooseAutoProgram;
		const std::string sAuto0 = "No_Auto";
		const std::string sAuto1 = "Auto_1";
		const std::string sAuto2 = "Auto_2";
		const std::string sAuto3 = "Auto_3";
	};
	structDS DS;


	// Drive Base
	struct structDriveBase{
		VictorSP* MotorLeft[3];
		VictorSP* MotorRight[3];
		Encoder* EncoderLeft;
		Encoder* EncoderRight;
		Solenoid* SolenoidShifter;
	};
	structDriveBase DriveBase;


	// Default Constructor
	RJ_RobotMap();

};



#endif
