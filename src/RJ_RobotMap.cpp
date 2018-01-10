#include "RJ_RobotMap.h"


RJ_RobotMap::RJ_RobotMap(){
	DS.DriveStick = new Joystick(0);
	DS.OperatorStick = new Joystick(1);

	DriveBase.MotorLeft[0] = new VictorSP(0);
	DriveBase.MotorLeft[1] = new VictorSP(1);
	DriveBase.MotorLeft[2] = new VictorSP(2);
	DriveBase.MotorRight[0] = new VictorSP(3);
	DriveBase.MotorRight[1] = new VictorSP(4);
	DriveBase.MotorRight[2] = new VictorSP(5);

	// Drive Base Encoders
	DriveBase.EncoderLeft = new Encoder(0,1);
	DriveBase.EncoderRight = new Encoder(2,3);

	DriveBase.EncoderLeft->SetReverseDirection(true);
	DriveBase.EncoderRight->SetReverseDirection(false);

	DriveBase.EncoderLeft->SetDistancePerPulse(98.0 / 3125.0 * 4.0);
	DriveBase.EncoderRight->SetDistancePerPulse(98.0 / 3125.0 * 4.0);

	// Drive Base Shifter
	DriveBase.SolenoidShifter = new Solenoid(0);

	DriveBase.SolenoidShifter->Set(false);


	// Smart Dashboard
	DS.chooseAutoProgram.AddDefault(DS.sAuto0, DS.sAuto0);
	DS.chooseAutoProgram.AddObject(DS.sAuto1, DS.sAuto1);
	DS.chooseAutoProgram.AddObject(DS.sAuto2, DS.sAuto2);
	DS.chooseAutoProgram.AddObject(DS.sAuto3, DS.sAuto3);
	frc::SmartDashboard::PutData("Auto Mode", &DS.chooseAutoProgram);


}

