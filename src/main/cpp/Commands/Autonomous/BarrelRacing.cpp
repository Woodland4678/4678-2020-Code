/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Autonomous/BarrelRacing.h"

BarrelRacing::BarrelRacing(): frc::Command() {
  Requires(Robot::drivetrain.get());
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void BarrelRacing::Initialize() {
    path1 = new PathFinder(0.02,0,2,1.5,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    path1->createNewPath();
    path1->addWayPoint(-1.050, 2.281, 0,0.007);  // -X is in front of robot, X is behind, Y is left, -Y is right
    path1->addWayPoint(-3.819, 2.245, 0,0.007); //2.44, 0, 0 - meters
    path1->addWayPoint(-4.572, 1.524, 90,0.007); //2.44, 0, 0 - meters
    path1->addWayPoint(-3.810, 0.762, 180,0.007);
    path1->addWayPoint(-3.048, 1.524, 270,0.007);
    path1->addWayPoint(-3.819, 2.245, 360,0.007);
    path1->addWayPoint(-6.086, 2.289, 360,0.007);
    path1->addWayPoint(-6.858, 3.048, 270,0.007);
    path1->addWayPoint(-6.096, 3.810, 180,0.007);
    path1->addWayPoint(-5.334, 3.048, 90,0.007);
    path1->addWayPoint(-7.620, 0.762, 0,0.007);
    path1->addWayPoint(-8.382, 1.524, -90,0.007);
    path1->addWayPoint(-7.620, 2.286, -180,0.007);
    path1->addWayPoint(-1.050, 2.281, -180,0.007);
    path1->makePath();
}

// Called repeatedly when this Command is scheduled to run
void BarrelRacing::Execute() {
  switch(autoStep) {
    case 0:
      if (path1->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
        rVel = 0;
        lVel = 0;
        autoStep++;
      }
      Robot::drivetrain->setRightVelocity(rVel);
      Robot::drivetrain->setLeftVelocity(lVel);
    break;
    case 1:
      done = true;
    break;
  }

}

// Called once the command ends or is interrupted.
void BarrelRacing::End() {}

// Returns true when the command should end.
bool BarrelRacing::IsFinished() { return done; }
