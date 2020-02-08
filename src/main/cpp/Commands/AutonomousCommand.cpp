// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/AutonomousCommand.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

AutonomousCommand::AutonomousCommand(): frc::Command() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void AutonomousCommand::Initialize() {
    	// switch 1 determines where the robot is starting from against the wall
    // see AutoScenarioHelpers.cpp  transformConsoleSwitch1 - values there must match below

	if (Robot::oi->getAutoSwitch()->GetRawButton(14)) {
		autoStartNum += 1;
	}
	if (Robot::oi->getAutoSwitch()->GetRawButton(15)) {
		autoStartNum += 2;
	}
	//TODO another set of numbers for autoFinishNum

    // switch 2 determines the objective during autonomous
    // see AutoScenarioHelpers.cpp  transformConsoleSwitch2 - values there must match below

	if (Robot::oi->getAutoSwitch()->GetRawButton(13)) {
		autoMode += 1;
	}
	if (Robot::oi->getAutoSwitch()->GetRawButton(12)) {
		autoMode += 2;
	}
	if (Robot::oi->getAutoSwitch()->GetRawButton(11)) {
		autoMode += 4;
	}
    if (autoStartNum == 0 ) {
        startX = 0;
        startY = 0;
    } else if (autoStartNum == 1) {
        startX = 1;
        startY = 0;
    }

    if (autoMode == 0) {
        autonomousMode = new AutoTrenchRun();
    }
	// frc::SmartDashboard::PutNumber("Auto Mode", autoMode);
	// frc::SmartDashboard::PutNumber("Auto Side", autoSide);
    autonomousMode->Start();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCommand::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool AutonomousCommand::IsFinished() {
    return true;
}

// Called once after isFinished returns true
void AutonomousCommand::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutonomousCommand::Interrupted() {

}
