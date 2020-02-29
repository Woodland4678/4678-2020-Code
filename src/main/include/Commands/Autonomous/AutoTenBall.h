// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#pragma once

#include "frc/commands/Command.h"
#include "frc/commands/Subsystem.h"
#include "Subsystems/PathFinder/Path.h"
#include "Robot.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class AutoTenBall: public frc::Command {
public:
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
AutoTenBall();
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

	PathFinder *path1;	
	PathFinder *path2;
	PathFinder *path3;	
	PathFinder *path4;	
	int autoStartNum = 0;
	int autoFinishNum = 0;
	double startX = 0;
	double startY = 0;
	double finishX = 0;
	double finishY = 0;

	double rVel = 0;
	double lVel = 0;
	int cnt = 0;

	enum AutonomousSteps {grabTwoBalls, returnToShootFirstVolley, autoAim, shoot, runTrench, returnToShootSecondVolley, finalAutoAim, finalShoot, delay};
	AutonomousSteps autoStep;

private:
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES

	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLES
};
