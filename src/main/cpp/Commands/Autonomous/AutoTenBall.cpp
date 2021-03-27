// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/Autonomous/AutoTenBall.h"

const double ALLOWABLE_AUTOAIM_ERROR = 1;
const double AUTOAIM_WAIT_COUNT = 30;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

AutoTenBall::AutoTenBall(): frc::Command() {
    // Use Requires() here to declare subsystem dependencies
    // eg. Requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    Requires(Robot::drivetrain.get());
}// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void AutoTenBall::Initialize() {
    // path1 = new PathFinder(0.02,0,4,4,3.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path1->createNewPath();
    // path1->addWayPoint(0, 0, 0,0.007);  // X is in front of robot, -X is behind, -Y is left, +Y is right
    // path1->addWayPoint(0.5, 0, 0,0.007); //2.44, 0, 0 - meters
    // path1->addWayPoint(2.3, 0.8, 50,0.007); //2.44, 0, 0 - meters
    // path1->makePath();

    // path2 = new PathFinder(0.02,0,4,4,3.5,1,0.7112);
    // path2->createNewPath();
    // path2->addWayPoint(2.3, 0.8, 50,0.007);
    // //path2->addWayPoint(1.2, 0, 0);
    // path2->addWayPoint(0.5,0.5,0,0.007);
    // path2->makePath();

    // path3 = new PathFinder(0.02,0,2,2,1.5,1,0.7112); //3.5, 2
    // path3->createNewPath();
    // path3->addWayPoint(0.5, 0, 0,0.007);
    // path3->addWayPoint(2,-1.15,0,0.007); //-1.7 for y if we are right in the middle of the target
    // path3->addWayPoint(4, -1.15,0,0.007); //6.57 is for full trench length)
    // path3->makePath();

    // path4 = new PathFinder(0.02,0,4.5,4.5,4,1,0.7112);
    // path4->createNewPath();
    // path4->addWayPoint(4, -1, 0,0.007);
    // path4->addWayPoint(1,1.5,0,0.007);
    // //path4->addWayPoint(0,0,0);
    // path4->makePath();

    // cnt = 0;
    // Robot::drivetrain->resetGyro();
    // path1->startTraverse(frc::Timer::GetFPGATimestamp());
}

// Called repeatedly when this Command is scheduled to run
void AutoTenBall::Execute() {
    // switch(autoStep) {
    //     case grabTwoBalls:
    //         //Robot::shooter->SetShooterSpeed(2500); //will need to adjust shooter speed in the future
    //         Robot::shooter->goToHoodPos(0); //will need to adjust this value
    //         //Robot::intakes->deployIntakes();
    //         if (path1->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
    //             failCount++;
    //             if (Robot::intakes->countCells() == 5 || failCount > 100) {
    //                 autoStep = delay;
    //                 nextAutoStep = returnToShootFirstVolley;
    //                 //Robot::intakes->retractIntakes();
    //                 delayCount = 5;
    //                 failCount = 0;
    //             }
    //             //path2->startTraverse(frc::Timer::GetFPGATimestamp());
    //             rVel = 0;
    //             lVel = 0;
    //             cnt = 0;
    //         }

    //         Robot::drivetrain->setRightVelocity(rVel);
    //         Robot::drivetrain->setLeftVelocity(lVel);
    //         cnt++;
    //     break;
    //     case returnToShootFirstVolley:
    //         Robot::shooter->SetShooterSpeed(5750);
    //         if (path2->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
    //         //if (path1->inverse_traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
    //             autoStep = autoAim;
    //             //delayCount = 50;
    //             //nextAutoStep = runTrench;
    //             Robot::drivetrain->setLimeLED(true);
    //             rVel = 0;
    //             lVel = 0;
    //             cnt = 0;
    //         }
    //         Robot::drivetrain->setRightVelocity(rVel);
    //         Robot::drivetrain->setLeftVelocity(lVel);
    //         cnt++;
    //     break;
    //     case autoAim:
    //         if (std::abs(Robot::drivetrain->autoAim(0)) < ALLOWABLE_AUTOAIM_ERROR) {
    //             cnt++;
    //         } else {
    //             cnt = 0;
    //         }
    //         if (cnt > AUTOAIM_WAIT_COUNT) {
    //             autoStep = shoot;
    //             Robot::drivetrain->setLimeLED(false);
    //             Robot::drivetrain->setRightPower(0);
    //             Robot::drivetrain->setLeftPower(0);
    //             failCount = 0;
    //             cnt = 0;
    //         }
    //     break;
    //     case shoot:
    //         Robot::intakes->spinMag();
    //         //Robot::intakes->deployIntakes();
    //         failCount++;
    //         if (failCount > 80) {
    //             Robot::shooter->stopShooter();
    //             Robot::intakes->stopMag();
    //             failCount = 0;
    //             path3->startTraverse(frc::Timer::GetFPGATimestamp());
    //             autoStep = runTrench;
    //         }

    //     break;
    //     case runTrench:
    //         Robot::intakes->deployIntakes();
    //         if (path3->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
    //             autoStep = delay;
    //             delayCount = 25;
    //             nextAutoStep = returnToShootSecondVolley;
    //             Robot::shooter->SetShooterSpeed(5750);
    //             //path4->startTraverse(frc::Timer::GetFPGATimestamp());
    //             rVel = 0;
    //             lVel = 0;
    //             cnt = 0;
    //         }
    //         Robot::drivetrain->setRightVelocity(rVel);
    //         Robot::drivetrain->setLeftVelocity(lVel);
    //         cnt++;
    //     break;
    //     case returnToShootSecondVolley:

    //         if (path4->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
    //             autoStep = finalAutoAim;
    //             Robot::drivetrain->setLimeLED(true);
    //             rVel = 0;
    //             lVel = 0;
    //             cnt = 0;
    //         }
    //         Robot::drivetrain->setRightVelocity(rVel);
    //         Robot::drivetrain->setLeftVelocity(lVel);
    //         cnt++;
    //     break;
    //     case finalAutoAim:
    //         if (std::abs(Robot::drivetrain->autoAim(0)) < ALLOWABLE_AUTOAIM_ERROR) {
    //             cnt++;
    //         } else {
    //             cnt = 0;
    //         }
    //         if (cnt > AUTOAIM_WAIT_COUNT) {
    //             autoStep = finalShoot;
    //             Robot::drivetrain->setLimeLED(false);
    //             Robot::drivetrain->setRightPower(0);
    //             Robot::drivetrain->setRightPower(0);
    //             failCount = 0;
    //             cnt = 0;
    //         }
    //     break;
    //     case finalShoot:     
    //         if (cnt > 80) {
    //             Robot::shooter->stopShooter();
    //             Robot::intakes->stopMag();
    //             failCount = 0;
    //         } else {
    //             Robot::intakes->spinMag();
    //         }
    //         cnt++;
    //     break;
    //     case delay:        
    //         if (cnt >= delayCount) {
    //             currentFPGATime = frc::Timer::GetFPGATimestamp();
    //             autoStep = nextAutoStep;
    //             path2->startTraverse(currentFPGATime);
    //             path3->startTraverse(currentFPGATime);
    //             path4->startTraverse(currentFPGATime);
    //             cnt = 0;
    //         }
    //         cnt++;
    //     break;
    // }
}

// Make this return true when this Command no longer needs to run execute()
bool AutoTenBall::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void AutoTenBall::End() {
    Robot::shooter->stopShooter();
    Robot::intakes->stopMag();
    Robot::drivetrain->setLimeLED(false);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoTenBall::Interrupted() {
    End();
}
