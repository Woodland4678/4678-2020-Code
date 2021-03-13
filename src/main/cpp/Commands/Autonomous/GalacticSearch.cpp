/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Autonomous/GalacticSearch.h"

GalacticSearch::GalacticSearch(): frc::Command() {
  Requires(Robot::drivetrain.get());
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void GalacticSearch::Initialize() {
    path1 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    path1->createNewPath();
    path1->addWayPoint(-1.113, 2.098, 0);  // -X is in front of robot, X is behind, Y is left, +Y is right
    path1->addWayPoint(-2.119, 2.516, -45); //2.44, 0, 0 - meters
    path1->addWayPoint(-3.755, 2.259, 60);
    path1->addWayPoint(-4.358, 1.252, 60);
    path1->makePath();

    path2 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    path2->createNewPath();
    path2->addWayPoint(-4.358, 1.252, 60);  // -X is in front of robot, X is behind, Y is left, -Y is right
    path2->addWayPoint(-3.621, 4.393, 45);
    path2->makePath();

    path3 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    path3->createNewPath();
    path3->addWayPoint(-3.621, 4.393, 45);  // -X is in front of robot, X is behind, Y is left, +Y is right
    path3->addWayPoint(-5.000, 3.592, 45);
    path3->addWayPoint(-5.591, 3.132, 45);
    path3->addWayPoint(-8.107, 1.065, 45);
    path3->makePath();

    cnt = 0;

}

// Called repeatedly when this Command is scheduled to run
void GalacticSearch::Execute() {
  switch(autoStep) {
    case 0:
      if (path1->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
        if(cnt > 25) {
          autoStep++;
          cnt = 0;
        }
        rVel = 0;
        lVel = 0;
        cnt++;
      }
      Robot::drivetrain->setRightVelocity(rVel);
      Robot::drivetrain->setLeftVelocity(lVel);
    break;
    case 1:
      if (path2->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
        if(cnt > 25) {
          autoStep++;
          cnt = 0;
        }
        rVel = 0;
        lVel = 0;
        cnt++;
      }
      Robot::drivetrain->setRightVelocity(rVel);
      Robot::drivetrain->setLeftVelocity(lVel);
    break;
    case 2:
      if (path3->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
        if(cnt > 25) {
          autoStep++;
          cnt = 0;
        }
        rVel = 0;
        lVel = 0;
        cnt++;
      }
      Robot::drivetrain->setRightVelocity(rVel);
      Robot::drivetrain->setLeftVelocity(lVel);
    break;
    case 3:
      done = true;
    break;
  }
}

// Called once the command ends or is interrupted.
void GalacticSearch::End() {}

// Returns true when the command should end.
bool GalacticSearch::IsFinished() { return done; }
