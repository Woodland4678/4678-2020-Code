// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/Autonomous/AutoTrenchRun.h"
#include "Subsystems/PathFinder/Path.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

AutoTrenchRun::AutoTrenchRun(): frc::Command() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    Requires(Robot::drivetrain.get());
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void AutoTrenchRun::Initialize() {
   	if (Robot::oi->getAutoSwitch()->GetRawButton(14)) {
		autoStartNum += 1;
	}
	if (Robot::oi->getAutoSwitch()->GetRawButton(15)) {
		autoStartNum += 2;
	}

    // if (autoStartNum == 0) {
    //     startX = 0;
    //     startY = 0;
    //     autoStep = autoAim;
    // } else if (autoStartNum == 1) {
    //     startX = 0;
    //     startY = -1.2; //-1 is in meters
    //     autoStep = shoot;
    // }
    // path1 = new PathFinder(0.02,3,2,1,0.545);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path1->createNewPath();
    // path1->addWayPoint(startX, startY, 0);  // X is in front of robot, -X is behind, -Y is left, +Y is right
    // path1->addWayPoint(2.44, 0, 6); //2.44, 0, 0 - meters
    // path1->addWayPoint(4, 0, 0); //2.44, 0, 0 - meters
    // path1->addWayPoint(8, 0, 0); //2.44, 0, 0 - meters
    // path1->addWayPoint(10.0, 0, 0); //6.16, 0, 0 - meters
    // path1->makePath();
    // path1->debug();

    path1 = new PathFinder(0.02,1,2,1,0.545);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    path1->createNewPath();
    path1->addWayPoint(0, 0, 0);  // X is in front of robot, -X is behind, -Y is left, +Y is right
    path1->addWayPoint(1, 0, 0); //2.44, 0, 0 - meters
    path1->makePath();
    //path1->debug();

    path2 = new PathFinder(0.02,4,4,1,0.545);
    path2->createNewPath();
    path2->addWayPoint(4.16, 0, 0); //6.16, 0, 0
    path2->addWayPoint(3.5,0,0); //3.5,0,0
    path2->addWayPoint(0,0,0); //finishX, finishY, 0
    path2->makePath();

    cnt = 0;
    Robot::drivetrain->resetGyro();
}

// Called repeatedly when this Command is scheduled to run
void AutoTrenchRun::Execute() {
    switch(autoStep) {
        case autoAim:
            //Auto Aim
//            if(Robot::drivetrain->AutoAim() < 0.5){
//
//            }
            if (true) {
                autoStep = shoot;
                //path1->debug();
            }
        break;
        case shoot:
            //Shoot
            //path1->debug();
            path1->startTraverse(frc::Timer::GetFPGATimestamp());
            autoStep = runFirstPath;
        break;
        case runFirstPath:
            if (path1->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
                autoStep = pause;
                rVel = 0;
                lVel = 0;
                cnt = 0;
            }

            Robot::drivetrain->setRightVelocity(rVel);
            Robot::drivetrain->setLeftVelocity(lVel);
            cnt++;
        break;
        case pause:
            cnt++;
            if (cnt > 50) {
                //autoStep = runSecondPath;
                path2->startTraverse(frc::Timer::GetFPGATimestamp());
                cnt = 0;
            }
            
        break;
        case runSecondPath:
            if (path2->traverse(cnt,&rVel,&lVel,Robot::drivetrain->getGyroReading())) {
                autoStep = pause;
                rVel = 0;
                lVel = 0;
                cnt = 0;
            }

            Robot::drivetrain->setRightVelocity(rVel);
            Robot::drivetrain->setLeftVelocity(lVel);
            cnt++;
        break;
        case finalAutoAim:
            //autoAim
        break;
        case finalShoot:
            //shoot
        break;
    }
    
    //frc::SmartDashboard::PutNumber("AutoCount", cnt);
}

// Make this return true when this Command no longer needs to run execute()
bool AutoTrenchRun::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void AutoTrenchRun::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoTrenchRun::Interrupted() {

}
