/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Autonomous/Slalom.h"

Slalom::Slalom(): frc::Command() {
  Requires(Robot::drivetrain.get());
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Slalom::Initialize() {
    //Original Path
    // path1 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path1->createNewPath();
    // path1->addWayPoint(-1.054,0.600,0,0.007);  // -X is in front of robot, X is behind, Y is left, -Y is right
    // path1->addWayPoint(-2.286, 1.524, -60,0.007); //2.44, 0, 0 - meters
    // path1->addWayPoint(-3.325, 2.228, 0,0.007); //2.44, 0, 0 - meters
    // path1->addWayPoint(-5.713, 2.250, 0,0.007);
    // path1->addWayPoint(-6.857, 1.521, 60,0.007);
    // path1->addWayPoint(-7.621, 0.761, 0,0.007);
    // path1->addWayPoint(-8.387, 1.525, -90,0.007);
    // path1->addWayPoint(-7.605, 2.285, -180,0.007);
    // path1->addWayPoint(-6.857, 1.521, -240,0.007);
    // path1->addWayPoint(-5.706, 0.800, -180,0.007);
    // path1->addWayPoint(-4.000, 0.800, -180,0.007);
    // path1->addWayPoint(-3.243, 0.800, -180,0.007);
    // path1->addWayPoint(-2.293, 1.521, -120,0.007);
    // path1->addWayPoint(-1.775, 2.461, -120,0.007);
    // path1->makePath();
    
    path1 = new PathFinder(0.02,0,2.1,2.1,2.2,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    path1->createNewPath();
    path1->addWayPoint(-1.054,0.6,0, 0.007);
    path1->addWayPoint(-2.286,1.524,-60, 0.007);
    path1->addWayPoint(-3.325,2.228,0, 0.007);
    path1->addWayPoint(-5.713,2.25,0, 0.007);
    path1->addWayPoint(-6.875,1.521,60, 0.007);
    path1->addWayPoint(-7.621,0.761,0, 0.007);
    path1->addWayPoint(-8.387,1.252,-89, 0.007);
    path1->addWayPoint(-7.605,2.285,-178, 0.007);
    path1->addWayPoint(-6.857, 1.521, -240, 0.007);
    path1->addWayPoint(-5.706, 0.802, -180, 0.007);
    path1->addWayPoint(-3.243, 0.800, -180, 0.007);
    path1->addWayPoint(-2.293, 1.521, -120, 0.007);
    path1->addWayPoint(-1.775, 2.461, -120, 0.007);
    path1->makePath();

    //start path 2
    // path2 = new PathFinder(0.02,2,2,2,1.5,1,0.7112);
    // path2->addWayPoint(-8.387, 1.525, -89);
    // path2->addWayPoint(-7.605, 2.285, -178);
    // path2->makePath(); 

    // //start path 3
    // path3 = new PathFinder(0.02,2,2,2,1.5,1,0.7112);
    // path3->addWayPoint(-7.605, 2.285, -180);
    // path3->addWayPoint(-6.857, 1.521, -240);
    // path3->addWayPoint(-5.706, 0.802, -180);
    // path3->addWayPoint(-3.243, 0.800, -180);
    // path3->addWayPoint(-2.293, 1.521, -120);
    // path3->addWayPoint(-1.775, 2.461, -120);
    // path3->makePath();
    path1->debug();
    Robot::drivetrain->resetGyro();
    path1->startTraverse(frc::Timer::GetFPGATimestamp());
}

// Called repeatedly when this Command is scheduled to run
void Slalom::Execute() {

  switch(autoStep) {
    case 0:
      if (path1->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
        rVel = 0;
        lVel = 0;
        autoStep = 3;
        //path2->startTraverse(frc::Timer::GetFPGATimestamp());
      }
      Robot::drivetrain->setRightVelocity(rVel);
      Robot::drivetrain->setLeftVelocity(lVel);
    break;
    case 1:
      if (path2->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
        rVel = 0;
        lVel = 0;
        autoStep++;
        path3->startTraverse(frc::Timer::GetFPGATimestamp());
      }
      Robot::drivetrain->setRightVelocity(rVel);
      Robot::drivetrain->setLeftVelocity(lVel);
    break;
    case 2:
    if (path3->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
        rVel = 0;
        lVel = 0;
        autoStep++;
      }
      Robot::drivetrain->setRightVelocity(rVel);
      Robot::drivetrain->setLeftVelocity(lVel);
    break;
    case 3:
    //done = true;

    break;
  }
}

// Called once the command ends or is interrupted.
void Slalom::End() {
  Robot::drivetrain->setRightVelocity(0);
  Robot::drivetrain->setLeftVelocity(0);

}

// Returns true when the command should end.
bool Slalom::IsFinished() { 
    return done; 
  }
