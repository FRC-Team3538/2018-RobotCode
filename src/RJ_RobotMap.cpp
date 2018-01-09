#include "RJ_RobotMap.h"


RJ_RobotMap::RJ_RobotMap(){
	DS.DriveStick = new Joystick(0);
	DS.OperatorStick = new Joystick(1);
}

