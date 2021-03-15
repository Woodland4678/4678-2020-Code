/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Autonomous/BouncePath.h"

BouncePath::BouncePath(): frc::Command() {
  Requires(Robot::drivetrain.get());
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void BouncePath::Initialize() {
    path1 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    path1->createNewPath();
    path1->addWayPoint(-1.080, 2.275, 0,0.007);  // -X is in front of robot, X is behind, Y is left, -Y is right
    path1->addWayPoint(-1.539, 2.275, 0,0.007); //2.44, 0, 0 - meters
    path1->addWayPoint(-2.286, 3.152, -90,0.007); //2.44, 0, 0 - meters
    path1->makePath();
    
    path2 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    path2->createNewPath();
    path2->addWayPoint(-2.286, 3.152, -90,0.007); 
    path2->addWayPoint(-3.048, 1.524, -120,0.007); 
    path2->addWayPoint(-3.810, 0.762, -180,0.007); 
    path2->addWayPoint(-4.571, 1.524, -270,0.007); 
    path2->addWayPoint(-4.575, 3.258, -270,0.007); 
    path2->makePath();

    path3 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    path3->createNewPath();
    path3->addWayPoint(-4.575, 3.258, -270,0.007); 
    path3->addWayPoint(-4.571, 1.524, -270,0.007); 
    path3->addWayPoint(-5.336, 0.762, -360,0.007); 
    path3->addWayPoint(-6.096, 0.762, -360,0.007); 
    path3->addWayPoint(-6.858, 1.524, -450,0.007); 
    path3->addWayPoint(-6.858, 3.186, -450,0.007); 
    path3->makePath();

    path4 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    path4->createNewPath();
    path4->addWayPoint(-6.858, 3.186, -450,0.007); 
    path4->addWayPoint(-7.277, 2.261, -480,0.007); 
    path4->makePath();

    cnt = 0;

}

// Called repeatedly when this Command is scheduled to run
void BouncePath::Execute() {
    switch(autoStep) {
    case 0:
      if (path1->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
        if(cnt > 25) {
          autoStep++;
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
      if (path4->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
        if(cnt > 25) {
          autoStep++;
        }
        rVel = 0;
        lVel = 0;
        cnt++;
      }
      Robot::drivetrain->setRightVelocity(rVel);
      Robot::drivetrain->setLeftVelocity(lVel);
    break;
    case 4:
      done = true;
    break;
  }
}

// Called once the command ends or is interrupted.
void BouncePath::End() {}

// Returns true when the command should end.
bool BouncePath::IsFinished() { return done; }
