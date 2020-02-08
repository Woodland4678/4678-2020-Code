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

#include "frc/commands/Subsystem.h"
#include "Subsystems/PathFinder/Path.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "rev/CANSparkMax.h"

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

/**
 *
 *
 * @author ExampleAuthor
 */
class Drivetrain: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<rev::CANSparkMax> leftMaster;
	std::shared_ptr<rev::CANSparkMax> leftSlave;
	std::shared_ptr<rev::CANSparkMax> rightMaster;
	std::shared_ptr<rev::CANSparkMax> rightSlave;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public:
Drivetrain();
	void InitDefaultCommand() override;
	void Periodic() override;
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

	//Set Functions
	void setLeftRPM(double rpm);
	void setRightRPM(double rpm);
	void setLeftVelocity(double mps);
	void setRightVelocity(double mps);
	void setLeftPosition(double encoder);
	void setRightPosition(double encoder);
	void setRightPower(double pwr);
	void setLeftPower(double pwr);

	//Get Functions
	double getLeftEncoder();
	double getRightEncoder();
	double getLeftRPM();
	double getRightRPM();
	double getLeftVelocity();
	double getRightVelocity();

	//Limelight
	std::shared_ptr<NetworkTable> limelight;
	bool ml_ValidTarget = false;
	double ml_targetHorizontial;
	double ml_targetVertical;

	void setLimeLED(bool state);
	void SetLimeFar();
	void SetLimeZoomed();
	bool getLimeValidObject();
	double getLimeHorizontial();
	double getLimeVertical();

	//Advanced Control Functions
	bool turnAmount(double degrees, int direction, double vel, double acc);
	double mt_acc;
	double mt_vel;
	double mt_tarDeg;
	double mt_tarDir;
	double mt_OEncLeft;
	double mt_OEncRight;
	int mt_state = 0;
	int mt_Cycles = 0;
	double d[400];
	int traverseCnt = 0;
	double encPrevLeft,encPrevRight,encLeft,encRight;

	PathFinder *m_Path;
	bool testPath();
	void initPath();
	int pathState = 0;
	double rVel = 0;
	double lVel = 0;
	int tCnt = 0;

	//Auto Controls
	bool AutoAim();
	double mAA_p = 0;
	double mAA_d = 0;
	double mAA_i = 0;
};
