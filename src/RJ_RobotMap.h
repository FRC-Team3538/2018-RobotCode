
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
		const std::string sAuto0 = "Auto 0";
		const std::string sAuto1 = "Auto 1";
		const std::string sAuto2 = "Auto 2";
		const std::string sAuto3 = "Auto 3";
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
