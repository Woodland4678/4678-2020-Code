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

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "frc/DigitalInput.h"
#include "frc/Solenoid.h"
#include "rev/CANSparkMax.h"

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

/**
 *
 *
 * @author ExampleAuthor
 */
class Intakes: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
std::shared_ptr<rev::CANSparkMax> middleRoller;
std::shared_ptr<rev::CANSparkMax> magazine;
std::shared_ptr<frc::DigitalInput> magazineSensorLow;
std::shared_ptr<frc::DigitalInput> magazineSensorHigh;
std::shared_ptr<frc::Solenoid> intakeSolenoid;
std::shared_ptr<rev::CANSparkMax> leftRoller;
std::shared_ptr<rev::CANSparkMax> rightRoller;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<frc::DigitalInput> intakeLeftSensor;
	std::shared_ptr<frc::DigitalInput> intakeRightSensor;

	//Intake State Variables
	bool m_deployed = false;

	//Magazine variables
	int m_powerCellCount = 3;
	bool m_prevDetect = false;
	int m_idxState = 0;
	int m_idxcnt = 0;
	bool m_idxrtn = false;
	int cellCounter = 0;
	bool slowRight = false;

	int cellNum = 0;

	bool cells[5];

	enum {
		INDEXWAITING,
		INDEXCELL,
		INDEXCOMPLETE
	};


public:
Intakes();
	void InitDefaultCommand() override;
	void Periodic() override;

	//--Intake State Functions--
	
	//deployIntakes: Takes out the intakes. Spins the wheels up. Calling this will automatically start the indexing process.
	void deployIntakes();
	//retractIntakes: Brings the intakes back into the robot, also stops the wheels spinning
	void retractIntakes();
	//isDeployed: returns true if the intakes are out and spinning to collect power cells
	bool isDeployed();

	//--Intake Speed Control Functions--
	
	//stopLeft: Stops the left side of the intakes
	void stopLeft();
	//stopRight: Stops the right side of the intakes
	void stopRight();
	//stopMiddle: Stops the middle of the intakes, do note stopping this will also stop the outer most black rollers.
	void stopMiddle();

	//setLeft: Sets the speed of the intakes left side (1 to -1)
	void setLeft(double speed);
	//setRight: Sets the speed of the intakes right side (1 to -1)
	void setRight(double speed);
	//setMiddle: Sets the speed of the intakes middle side (1 to -1)
	void setMiddle(double speed);

	//setIntakeSpeed: For those lazy buggers, sets the speeds for each section of the intakes (1 to -1), paramters are in this order: right, left, and middle.
	void setIntakeSpeed(double rightPower, double leftPower, double middlePower);
	//stopIntakes: Stops all three sections of the intakes
	void stopIntakes();
	//spitoutIntakes: Reverses all three sections of the intakes, The speed is set by a const found at the top of intakes.cpp
	void spitoutIntakes();

	bool getLeftSensor();
	bool getRightSensor();
	
	//--Magazine Control--

	void clearMagEncoder();

	//to change the speed we shoot the cells
	void setMagSpeed(double speed);
	//spinMag: Sets the magazine to spin. The speed is set by a const found at the top of intakes.cpp
	void spinMag();
	//stopMag: Stops the magazine from spinning
	void stopMag();
	//spitoutMag: Reverses the magazine. The speed is set by a const found at the top of intakes.cpp
	void spitoutMag();
	//setMagazinePosition: Moves the magazine to a specific position determined by the encoder value passed in
	void setMagazinePosition(double encoder);
	//getMagazinePosition: Returns the current magazine position (encoder pulses)
	double getMagazinePosition();
	//index: brings a cell into the magazine once one is detected. Continuously call until it returns true.
	bool index(bool c1Override = false);
	bool getMagHighSensor();
	void resetMagazinePosition();
	//for moving cells to top after intake raised
	bool moveCellsToTop();

	//--Power Cell Functions--
	//setCellPosition: Set position in magazine to true (ball)
	void setCellPosition(int position);
	//clearCellPosition: set position in magazine to false (no ball)
	void clearCellPosition(int position);
	//countCells: Returns the number of power cells in the magazine
	int countCells();
	//shiftCells: shifts the position of the balls in the magazine
	bool shiftCells(bool firstCell);

	int getCellNum();
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS


};

