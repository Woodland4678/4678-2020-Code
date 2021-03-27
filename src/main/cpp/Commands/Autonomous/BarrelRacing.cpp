/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Autonomous/BarrelRacing.h"
#include "Subsystems/LidarViewer.h"

BarrelRacing::BarrelRacing(): frc::Command() {
  Requires(Robot::drivetrain.get());
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void BarrelRacing::Initialize() {
    // path1 = new PathFinder(0.02,0,1,1,1,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path1->createNewPath();
    // path1->addWayPoint(-1.050, 2.281, 0, 0.015);  // -X is in front of robot, X is behind, Y is left, -Y is right
    // path1->addWayPoint(-3.819, 2.245, 0.5, 0.015); //2.44, 0, 0 - meters
    // path1->addWayPoint(-4.572, 1.524, 90, 0.015); //2.44, 0, 0 - meters
    // path1->addWayPoint(-3.810, 0.762, 179.5, 0.015);
    // path1->addWayPoint(-3.048, 1.524, 269, 0.015);
    // path1->addWayPoint(-3.819, 2.245, 358.5, 0.015);
    // path1->addWayPoint(-6.086, 2.289, 360, 0.015);
    // path1->addWayPoint(-6.858, 3.048, 270.5, 0.015);
    // path1->addWayPoint(-6.096, 3.810, 181, 0.015);
    // path1->addWayPoint(-5.334, 3.048, 91.5, 0.015);
    // path1->addWayPoint(-7.620, 0.762, 0, 0.015);
    // path1->addWayPoint(-8.382, 1.524, -90, 0.015);
    // path1->addWayPoint(-7.620, 2.286, -180, 0.015);
    // path1->addWayPoint(-1.050, 2.281, -180, 0.015);
    // path1->makePath();
    // path1->startTraverse(frc::Timer::GetFPGATimestamp());
    path1 = new PathFinder(3,3,0.7112,1); // acceleration (m/s^2), deceleration (m/s^2), distance between wheels (m), run on robot on/off (0 is off, 1 is on)

    Robot::drivetrain->setStart(-1.050,2.281,1); // Initiate robot measured location.
    path1->setStartPoint(-1.050, 2.281, 0); //start x,y,angle
    path1->splineTo(1,-3.610, 2.145, 0,2.5,2,0,5000); //int segmentID, double x (m), double y (m), double angle (degrees), double targetVelocity (m/s), double finalVelocity (m/s), int useActual, int samples
    path1->circleAround(123,-3.610,1.364,340,2,2,0,5000);
    path1->splineTo(6,-5.386, 2.505, 360,2,2,0,5000);    
    path1->circleAround(321,-5.396,3.325,-290,2,2,0,5000);
    path1->splineTo(10,-6.020, 1.412, 0.4,2.2,2,0,5000);
    path1->circleAround(333,-6.020,2.224,-165,2,2,0,5000);
    path1->splineTo(13,-0.350, 3.671, -180,4,0.5,0,5000);

//    path1->splineTo(1,-3.819, 2.245, 0,1.5,1.5,0,5000); //int segmentID, double x (m), double y (m), double angle (degrees), double targetVelocity (m/s), double finalVelocity (m/s), int useActual, int samples
//    path1->splineTo(2,-4.572, 1.524, 89.9,1.5,1.5,0,5000); //2.44, 0, 0 - meters
//    path1->splineTo(3,-3.810, 0.762, 179.8,1.5,1.5,0,5000);
//    path1->splineTo(4,-3.048, 1.524, 269.7,1.5,1.5,0,5000);
//    path1->splineTo(5,-3.819, 2.245, 359.6,1.5,1.5,0,5000);
//    path1->splineTo(6,-6.086, 2.289, 360,1.5,1.5,0,5000);    
//    path1->splineTo(7,-6.858, 3.048, 270.1,1.5,1.5,0,5000);
//    path1->splineTo(8,-6.096, 3.810, 180.2,1.5,1.5,0,5000);
//    path1->splineTo(9,-5.334, 3.048, 90.3,1.5,1.5,0,5000);
//    path1->splineTo(10,-7.620, 0.762, 0.4,1.5,1.5,0,5000);
//    path1->splineTo(11,-8.382, 1.524, -89.5,1.5,1.5,0,5000);
//    path1->splineTo(12,-7.620, 2.286, -179.6,1.5,1.5,0,5000);
//    path1->splineTo(13,-1.050, 2.281, -180,1.5,1.5,0,5000);
    //LidarViewer::Get()->m_numScoring = 0;
    autoStep = 0;
    Robot::drivetrain->resetGyro();
}

// Called repeatedly when this Command is scheduled to run
void BarrelRacing::Execute() {
  switch(autoStep) {
    case 0:
      // if (path1->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
      //   rVel = 0;
      //   lVel = 0;
      //   autoStep++;
      // }
      // Robot::drivetrain->setRightVelocity(rVel);
      // Robot::drivetrain->setLeftVelocity(lVel);
      if (path1->processPath()) {
        autoStep++;
      }
    break;
    case 1:
      done = true;
    break;
  }
  done = false;

}

// Called once the command ends or is interrupted.
void BarrelRacing::End() {}

// Returns true when the command should end.
bool BarrelRacing::IsFinished() { return done; }
