#include "RJ_Auto.h"


// Default Constructor
RJ_Auto::RJ_Auto(RJ_RobotMap* map){
	IO = map;
}


// Initialize and Reset all Auto Programs
void RJ_Auto::Initalize(){

	// restart Auto Programs
	Step = 0;
}

// Called Periodicly from the main robot task
// Calls the desired robot program
void RJ_Auto::Periodic(){
	// Select and auto program from
	if (IO->DS.chooseAutoProgram.GetSelected() == IO->DS.sAuto0) {
		// Default Auto Program

	}
	if (IO->DS.chooseAutoProgram.GetSelected() == IO->DS.sAuto1) {
		RJ_Auto::Program1();
	}

}


// Demo Auto Program 1
// Just drive forward based on time
void RJ_Auto::Program1(){

	if (Step == 0)
	{
		// Start the timer
		StepTimer.Reset();
		StepTimer.Start();
		Step++;
	}
	if(Step == 1){
		// Drive Forward
		IO->DriveBase.MotorLeft[0]->Set(0.5);
		IO->DriveBase.MotorRight[0]->Set(0.5);
		Step++;
	}
	if(Step == 2 && StepTimer.HasPeriodPassed(5.0))	{
		// Stop
		IO->DriveBase.MotorLeft[0]->Set(0);
		IO->DriveBase.MotorRight[0]->Set(0);
		Step++;
	}


}
