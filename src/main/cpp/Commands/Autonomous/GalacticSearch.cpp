/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Autonomous/GalacticSearch.h"
#include "Subsystems/LidarViewer.h"

GalacticSearch::GalacticSearch(): frc::Command() {
  Requires(Robot::drivetrain.get());
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void GalacticSearch::Initialize() {
    // path1 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path1->createNewPath();
    // path1->addWayPoint(-1.113, 2.098, 0,0.007);  // -X is in front of robot, X is behind, Y is left, +Y is right
    // path1->addWayPoint(-2.119, 2.516, -45,0.007); //2.44, 0, 0 - meters
    // path1->addWayPoint(-3.755, 2.259, 60,0.007);
    // path1->addWayPoint(-4.358, 1.252, 60,0.007);
    // path1->makePath();

    // path2 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path2->createNewPath();
    // path2->addWayPoint(-4.358, 1.252, 60,0.007);  // -X is in front of robot, X is behind, Y is left, -Y is right
    // path2->addWayPoint(-3.621, 4.393, 45,0.007);
    // path2->makePath();

    // path3 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path3->createNewPath();
    // path3->addWayPoint(-3.621, 4.393, 45,0.007);  // -X is in front of robot, X is behind, Y is left, +Y is right
    // path3->addWayPoint(-5.000, 3.592, 45,0.007);
    // path3->addWayPoint(-5.591, 3.132, 45,0.007);
    // path3->addWayPoint(-8.107, 1.065, 45,0.007);
    // path3->makePath();

    path1 = new PathFinder(1.5,1.5,0.7112,1);
    path1->setStartPoint(-1.113, 2.098, 0); 
    path1->splineTo(1,-2.119, 2.516, -45,2.0,2.0,0,5000); //int segmentID, double x (m), double y (m), double angle (degrees), double targetVelocity (m/s), double finalVelocity (m/s), int useActual, int samples
    path1->splineTo(2,-3.755, 2.259, 60,2.0,2.0,0,5000); //2.44, 0, 0 - meters
    path1->splineTo(3,-4.358, 1.252, 60,2.0,2.0,0,5000);

    path2 = new PathFinder(1.5,1.5,0.7112,1);
    path2->setStartPoint(-4.358, 1.252, 60); 
    path2->splineTo(1,-3.621, 4.393, 45,2.0,2.0,0,5000); 

    path3 = new PathFinder(1.5,1.5,0.7112,1);
    path3->setStartPoint(-3.621, 4.393, 45); 
    path3->splineTo(1,-5.000, 3.592, 45,2.0,2.0,0,5000); 
    path3->splineTo(2,-5.591, 3.132, 45,2.0,2.0,0,5000); 
    path3->splineTo(3,-8.107, 1.065, 45,2.0,2.0,0,5000);
    //LidarViewer::Get()->m_numScoring = 0;

    cnt = 0;

}

// Called repeatedly when this Command is scheduled to run
void GalacticSearch::Execute() {
  switch(autoStep) {
    case 0:
      // if (path1->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
      //   if(cnt > 25) {
      //     autoStep++;
      //     cnt = 0;
      //   }
      //   rVel = 0;
      //   lVel = 0;
      //   cnt++;
      // }
      // Robot::drivetrain->setRightVelocity(rVel);
      // Robot::drivetrain->setLeftVelocity(lVel);
      if(path1->processPath()) {
        autoStep++;
      }
    break;
    case 1:
      if(path2->processPath()) {
        autoStep++;
      }
    break;
    case 2:
      if(path3->processPath()) {
        autoStep++;
      }
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
