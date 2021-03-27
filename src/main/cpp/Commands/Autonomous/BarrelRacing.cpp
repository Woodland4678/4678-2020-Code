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
    double wideCircleFactor = 1.1;
    double wideCircleFactorY = 0.9;
    path1 = new PathFinder(1.5,1.5,0.78,1); // acceleration (m/s^2), deceleration (m/s^2), distance between wheels (m), run on robot on/off (0 is off, 1 is on)

    path1->setStartPoint(-1.050, 2.281, 0); //start x,y,angle
    path1->splineTo(1,-3.819 * wideCircleFactor, 2.245, 0,2.0,2.0,0,5000); //int segmentID, double x (m), double y (m), double angle (degrees), double targetVelocity (m/s), double finalVelocity (m/s), int useActual, int samples
    path1->splineTo(2,-4.3564 * wideCircleFactor, 2.0184, 45,2.0,2.0,0,5000);
    path1->splineTo(3,-4.572 * wideCircleFactor, 1.524 * wideCircleFactorY, 90,2.0,2.0,0,5000); //2.44, 0, 0 - meters
    path1->splineTo(4,-4.3454 * wideCircleFactor, 0.9866 * wideCircleFactorY, 135,2.0,2.0,0,5000);
    path1->splineTo(5,-3.810, 0.762 * wideCircleFactorY, 180,2.0,2.0,0,5000);
    path1->splineTo(6,-3.2726 * 0.8, 0.9886 *1.1, 225,2.0,2.0,0,5000);
    path1->splineTo(7,-3.048 * 0.8, 1.524 * 1.1, 270,2.0,2.0,0,5000);
    path1->splineTo(8,-3.2746 * 0.8, 2.0614, 315,2.0,2.0,0,5000);
    path1->splineTo(9,-3.819 * 0.8, 2.245, 360,2.0,2.0,0,5000);

    path1->splineTo(10,-6.086, 2.289, 360,2.0,2.0,0,5000);
    path1->splineTo(11,-6.6234, 2.5156, 315,2.0,2.0,0,5000);    
    path1->splineTo(12,-6.858, 3.048, 270,2.0,2.0,0,5000);
    path1->splineTo(13,-6.6314, 3.5854, 225.1,2.0,2.0,0,5000);
    path1->splineTo(14,-6.096, 3.810, 180,2.0,2.0,0,5000);
    path1->splineTo(15,-5.5586, 3.5834, 135,2.0,2.0,0,5000);
    path1->splineTo(16,-5.334, 3.048, 90,2.0,2.0,0,5000);

    path1->splineTo(17,-7.620, 0.762, 0.4,2.0,2.0,0,5000);
    path1->splineTo(17,-8.1574, 0.9886, -45,2.0,2.0,0,5000);
    path1->splineTo(18,-8.382, 1.524, -90,2.0,2.0,0,5000);
    path1->splineTo(18,-8.1554, 2.0614, -135,2.0,2.0,0,5000);
    path1->splineTo(19,-7.620, 2.286, -180,2.0,2.0,0,5000);
    path1->splineTo(20,-1.050, 2.281, -180,2.0,2.0,0,5000);
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
