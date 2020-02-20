// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/Shoot.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

Shoot::Shoot(): frc::Command() {
    // Use Requires() here to declare subsystem dependencies
    // eg. Requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::shooter.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

const double acceptableAimError = 2;

// Called just before this Command runs the first time
void Shoot::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute() {
    switch (shootState){
        case 0:
            if (Robot::drivetrain->autoAim(Robot::shooter->getAimTarget()) <= acceptableAimError){
                count++;
            } 
            else {
                count = 0;
            }
            if (count > 10 && isHoodInPos == true){
                shootState++;
                count = 0;
            }
            if (isHoodInPos == false){
                isHoodInPos = Robot::shooter->goToHoodPos(Robot::shooter->getHoodTarget());
            }
            Robot::shooter->SetShooterSpeed(Robot::shooter->getTargetShootVel());
        break;
        case 1:
           Robot::shooter->shoot();
        break;
        case 2:

        break;
    }
}

// Make this return true when this Command no longer needs to run execute()
bool Shoot::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void Shoot::End() {
    Robot::shooter->stopShooter();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Shoot::Interrupted() {
    End();
}
