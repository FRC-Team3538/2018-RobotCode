
#ifndef _RJ_ROBOTMAP_H_
#define _RJ_ROBOTMAP_H_

#include "WPILib.h"

class RJ_RobotMap {

public:

	// Driver's Station Stuff
	struct structDS{
		Joystick DriveStick;
		Joystick OperatorStick;
	};
	structDS DS;


	// Default Constructor
	RJ_RobotMap();


	int testdata = 5;
	int testfun(int input);

};



#endif
