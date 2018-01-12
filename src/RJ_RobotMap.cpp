#include "RJ_RobotMap.h"

RJ_RobotMap::RJ_RobotMap() {

	//
	// Drive Base
	//

	// Set Motor directions
	DriveBase.MotorsLeft.SetInverted(true);
	DriveBase.MotorsRight.SetInverted(false);

	// Set Encoder Direction & Scale
	DriveBase.EncoderLeft.SetReverseDirection(true);
	DriveBase.EncoderRight.SetReverseDirection(false);

	DriveBase.EncoderLeft.SetDistancePerPulse(98.0 / 3125.0 * 4.0);
	DriveBase.EncoderRight.SetDistancePerPulse(98.0 / 3125.0 * 4.0);

	// Set Default Gear
	DriveBase.SolenoidShifter.Set(false);

	//
	// Smart Dashboard
	//

	// Auto Program Chooser
	DS.chooseAutoProgram.AddDefault(DS.sAuto0, DS.sAuto0);
	DS.chooseAutoProgram.AddObject(DS.sAuto1, DS.sAuto1);
	DS.chooseAutoProgram.AddObject(DS.sAuto2, DS.sAuto2);
	DS.chooseAutoProgram.AddObject(DS.sAuto3, DS.sAuto3);
	frc::SmartDashboard::PutData("Auto Mode", &DS.chooseAutoProgram);

}

