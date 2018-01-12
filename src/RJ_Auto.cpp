#include "RJ_Auto.h"
#include "math.h"

// Default Constructor
RJ_Auto::RJ_Auto(RJ_RobotMap* map) {
	IO = map;
}

// Initialize and Reset all Auto Programs
void RJ_Auto::Initalize() {
	// restart Auto Programs
	m_Step = 0;
}

// Called Periodicly from the main robot task
// Calls the desired robot program
void RJ_Auto::Periodic() {
	// Select and auto program from
	if (IO->DS.chooseAutoProgram.GetSelected() == IO->DS.sAuto0) {
	}
	// Default Auto Program

	if (IO->DS.chooseAutoProgram.GetSelected() == IO->DS.sAuto1)
		RJ_Auto::Program1();

	if (IO->DS.chooseAutoProgram.GetSelected() == IO->DS.sAuto2)
		RJ_Auto::Program2();

}

//
// Encoder Helpers
//
double RJ_Auto::readEncoder() {

	// We assume that the 'failure mode' of the encoder is to
	// not generate any pulses. So lets just use the data that
	// is the farthest from zero. Technically, it could
	// generate too many pulses, but that seems less likely...
	double l = fabs(IO->DriveBase.EncoderLeft.GetDistance());
	double r = fabs(IO->DriveBase.EncoderRight.GetDistance());

	if (l > r)
		return l;
	else
		return r;

	// Notify the user if the encoders don't match.
	if (fabs(l - r) > 3.0)
		printf("RJ FAULT: ENCODER FAILURE!!!!!");
}

void RJ_Auto::resetEncoder() {
	IO->DriveBase.EncoderLeft.Reset();
	IO->DriveBase.EncoderRight.Reset();
}

// Demo Auto Program 1
// Just drive forward based on time
void RJ_Auto::Program1() {

	if (m_Step == 0) {
		// Start the timer
		m_StepTimer.Reset();
		m_StepTimer.Start();
		m_Step++;
	}
	if (m_Step == 1) {
		// Drive Forward
		IO->DriveBase.MotorsLeft.Set(0.5);
		IO->DriveBase.MotorsRight.Set(0.5);
		m_Step++;
	}
	if (m_Step == 2 && m_StepTimer.HasPeriodPassed(5.0)) {
		// Stop
		IO->DriveBase.MotorsLeft.Set(0);
		IO->DriveBase.MotorsRight.Set(0);
		m_Step++;
	}

}

// Demo Auto Program 2
// Just drive forward based on encoders
void RJ_Auto::Program2() {

	if (m_Step == 0) {
		// Start the timer
		m_StepTimer.Reset();
		m_StepTimer.Start();
		resetEncoder();
		m_Step++;
	}
	if (m_Step == 1) {
		// Drive Forward
		IO->DriveBase.MotorsLeft.Set(0.5);
		IO->DriveBase.MotorsRight.Set(0.5);
		m_Step++;
	}
	if (m_Step == 2) {
		if (m_StepTimer.HasPeriodPassed(5.0)) {
			printf("RJ FAULT: TIMEOUT!!! PROGRAM 2, STEP %d", m_Step);
			IO->DriveBase.MotorsLeft.Set(0);
			IO->DriveBase.MotorsRight.Set(0);
			m_Step = -1;
		}
		if (readEncoder() > 36.0) {
			IO->DriveBase.MotorsLeft.Set(0);
			IO->DriveBase.MotorsRight.Set(0);
			m_Step++;
		}
	}

}
