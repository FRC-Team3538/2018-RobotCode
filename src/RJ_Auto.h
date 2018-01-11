
#ifndef _RJ_AUTO_H_
#define _RJ_AUTO_H_

#include "RJ_RobotMap.h"

class RJ_Auto {

private:
	// Access to all robot IO
	RJ_RobotMap* IO;

	// State Variables
	int Step;
	Timer StepTimer;

public:
	// Default Constructor
	RJ_Auto(RJ_RobotMap*);

	// Call from the Main Robot task's Auto functions
	void Initalize();
	void Periodic();


private:
	//Auto Programs
	void Program1();

};

#endif
