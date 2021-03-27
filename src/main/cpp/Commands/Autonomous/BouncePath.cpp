/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/Autonomous/BouncePath.h"
#include "Subsystems/LidarViewer.h"

BouncePath::BouncePath(): frc::Command() {
  Requires(Robot::drivetrain.get());
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void BouncePath::Initialize() {
    // path1 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path1->createNewPath();
    // path1->addWayPoint(-1.080, 2.275, 0,0.007);  // -X is in front of robot, X is behind, Y is left, -Y is right
    // path1->addWayPoint(-1.539, 2.275, 0,0.007); //2.44, 0, 0 - meters
    // path1->addWayPoint(-2.286, 3.152, -90,0.007); //2.44, 0, 0 - meters
    // path1->makePath();
    
    // path2 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path2->createNewPath();
    // path2->addWayPoint(-2.286, 3.152, -90,0.007); 
    // path2->addWayPoint(-3.048, 1.524, -120,0.007); 
    // path2->addWayPoint(-3.810, 0.762, -180,0.007); 
    // path2->addWayPoint(-4.571, 1.524, -270,0.007); 
    // path2->addWayPoint(-4.575, 3.258, -270,0.007); 
    // path2->makePath();

    // path3 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path3->createNewPath();
    // path3->addWayPoint(-4.575, 3.258, -270,0.007); 
    // path3->addWayPoint(-4.571, 1.524, -270,0.007); 
    // path3->addWayPoint(-5.336, 0.762, -360,0.007); 
    // path3->addWayPoint(-6.096, 0.762, -360,0.007); 
    // path3->addWayPoint(-6.858, 1.524, -450,0.007); 
    // path3->addWayPoint(-6.858, 3.186, -450,0.007); 
    // path3->makePath();

    // path4 = new PathFinder(0.02,0,2,2,1.5,1,0.7112);  // cycle time (s), max velocity (m/s), max acceleration (m/s^2), max jerk (m/s^3), distance between wheels (m)
    // path4->createNewPath();
    // path4->addWayPoint(-6.858, 3.186, -450,0.007); 
    // path4->addWayPoint(-7.277, 2.261, -480,0.007); 
    // path4->makePath();
    path1 = new PathFinder(2,2,0.4,0);
    path1->setStartPoint(-1.080, 2.275, 0); 
    //path1->splineTo(1,-1.4, 2.275, 0,2.0,2,0,5000); //int segmentID, double x (m), double y (m), double angle (degrees), double targetVelocity (m/s), double finalVelocity (m/s), int useActual, int samples
    path1->splineTo(1,-2.286, 4.052, -89.9,2.0,2.0,0,5000); //2.44, 0, 0 - meters

    path2 = new PathFinder(2.0,2.0,0.25,0);
    path2->setStartPoint(-2.286, 4.052, -89.9); 
    path2->splineTo(1,-3.148, 1.624, -120,-2.5,-2.0,0,5000); //-3.248, 1.524
    path2->splineTo(2,-3.810, 0.962, -180,-2.0,-2.0,0,5000); //-3.810, 0.762, -180
    path2->splineTo(3,-4.471, 1.524, -269.9,-2.0,-2.0,0,5000); //-4.571, 1.524, -269.9
    path2->splineTo(4,-4.475, 4.858, -270,-3,-0.5,0,5000); //-4.575, 3.258, -270

    path3 = new PathFinder(2,2,0.20,0);
    path3->setStartPoint(-4.475, 4.858, -270); 
    path3->splineTo(1,-4.571, 2.924, -270,3,2.0,0,5000); 
    path3->splineTo(2,-5.336, 1.662, -359.9,2.0,2.0,0,5000); 
    path3->splineTo(3,-5.796, 1.662, -360,2.0,2.0,0,5000);
    path3->splineTo(4,-6.158, 2.024, -449.9,2.0,2.0,0,5000);
    path3->splineTo(5,-6.158, 5.986, -450,3,0.75,0,5000);

    path4 = new PathFinder(2,2,0.7112,0);
    path4->setStartPoint(-6.458, 5.986, -450); 
    path4->splineTo(1,-7.277, 4.461, -480,-3.5,-2.0,0,5000); 
    LidarViewer::Get()->m_numScoring = 0;
    Robot::drivetrain->resetGyro();
    cnt = 0;

}

// Called repeatedly when this Command is scheduled to run
void BouncePath::Execute() {
    switch(autoStep) {
    case 0:
      // if (path1->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,Robot::drivetrain->getGyroReading())) {   // cnt = how far down the path are you, right velocity (m/s), left velocity (m/s)
      //   if(cnt > 25) {
      //     autoStep++;
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
      if(path4->processPath()) {
        autoStep++;
      }
    break;
    case 4:
      done = true;
      Robot::drivetrain->setLeftVelocity(0);
      Robot::drivetrain->setRightVelocity(0);
    break;
  }
}

// Called once the command ends or is interrupted.
void BouncePath::End() {}

// Returns true when the command should end.
bool BouncePath::IsFinished() { return done; }
