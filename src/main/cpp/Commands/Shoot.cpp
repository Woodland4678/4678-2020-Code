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
    Requires(Robot::drivetrain.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

const double acceptableAimError = 2;

// Called just before this Command runs the first time
void Shoot::Initialize() {
    done = false;
    shootState = 0;
    count = 0;
    aimCount = 0;
    isAimed = false;
    shootRPM = 0;
    if(Robot::oi->getAutoSwitch()->GetRawButton(14)) {
        Robot::drivetrain->setLimeLED(true);
        shootMode = target;
    } else {
        shootMode = demo;
    }
}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute() {
    //close shot
    
    if(shootMode == target) {
        if(!isAimed){
            if (Robot::drivetrain->autoAim(aimToNum) < 0.2) {
                aimCount++;
            }
            if(Robot::drivetrain->getLimeVertical() >= 14 && Robot::drivetrain->getLimeVertical() < 21) {
                Robot::shooter->goToHoodPos(-840);
                shootRPM = 4000;
                Robot::intakes->setMagSpeed(-0.9);
                aimToNum = -0.35;
                
            } 
            else if(Robot::drivetrain->getLimeVertical() <= 30 && Robot::drivetrain->getLimeVertical() > 22) {
                //old cells -2450 and 5500 works from 8.5ft to 15ft
                //
                Robot::shooter->goToHoodPos(-100);
                shootRPM = 3750;
                Robot::intakes->setMagSpeed(-0.9);
                aimToNum = 0;
            }       
            //medium shot
            else if(Robot::drivetrain->getLimeVertical() <= 4 && Robot::drivetrain->getLimeVertical() > 1) {
                //old cells -2450 and 5500 works from 8.5ft to 15ft
                Robot::shooter->goToHoodPos(-3010); //
                shootRPM = 5300;
                Robot::intakes->setMagSpeed(-0.9);
                aimToNum = -0.35;
            }                                  
            //far shot                                                                      
            else if(Robot::drivetrain->getLimeVertical() <= -4 && Robot::drivetrain->getLimeVertical() > -5.5) {
                Robot::shooter->goToHoodPos(-3490);
                shootRPM = 5500;
                Robot::intakes->setMagSpeed(-0.9);
                aimToNum = 1.0;
            } 
            else if(Robot::drivetrain->getLimeVertical() <= -8.5) {
                Robot::shooter->goToHoodPos(-3780);
                shootRPM = 5900;
                Robot::intakes->setMagSpeed(-0.9);
                aimToNum = 1.0;
            } else {
                Robot::shooter->goToHoodPos(-1000);
                shootRPM = 3800;
                Robot::intakes->setMagSpeed(-0.9);
                aimToNum = 0;
            }
        } 
        if (aimCount > 55) {
            //Set drive wheels to zero power here to stop drifting
            Robot::drivetrain->setLeftPower(0);
            Robot::drivetrain->setRightPower(0);
            isAimed = true;
        };
    } else {
        shootRPM = 4000;
        Robot::shooter->goToHoodPos(-1200);
        isAimed = true;
    }
    Robot::shooter->shoot(shootRPM, isAimed, false);
    
    // switch (shootState){
    //     case 0:
    //         if (abs(Robot::drivetrain->autoAim(Robot::shooter->getAimTarget())) <= acceptableAimError){
    //             count++;
    //         } 
    //         else {
    //             count = 0;
    //         }
    //         if (count > 10 && isHoodInPos == true){
    //             shootState++;
    //             count = 0;
    //         }
    //         if (isHoodInPos == false){
    //             isHoodInPos = Robot::shooter->goToHoodPos(Robot::shooter->getHoodTarget());
    //         }
    //         Robot::shooter->SetShooterSpeed(Robot::shooter->getTargetShootVel());
    //         break;
    //     case 1:
    //         if (Robot::shooter->shoot()){
    //             done = true;
    //         }
    //         break;
    // }
}

// Make this return true when this Command no longer needs to run execute()
bool Shoot::IsFinished() {
    return done;
}

// Called once after isFinished returns true
void Shoot::End() {
    Robot::shooter->stopShooter();
    Robot::intakes->stopMag();
    Robot::intakes->resetMagazinePosition();
    Robot:: shooter->disableHood();
    Robot::shooter->shootState = 0;
    shootState = 0;
    count = 0;
    aimCount = 0;
    isAimed = false;
    Robot::intakes->setCellNum(0);
    Robot::drivetrain->setLimeLED(false);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Shoot::Interrupted() {
    End();
}
