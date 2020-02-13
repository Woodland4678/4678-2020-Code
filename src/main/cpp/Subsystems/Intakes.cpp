// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/Intakes.h"
#include <frc/SmartDashboard/SmartDashboard.h>

const double INTAKESPEED = 0.70;
const double MAGSPEED = 1;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

Intakes::Intakes() : frc::Subsystem("Intakes") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
middleRoller.reset(new rev::CANSparkMax(6, rev::CANSparkMax::MotorType::kBrushless));


magazine.reset(new rev::CANSparkMax(8, rev::CANSparkMax::MotorType::kBrushless));


magazineSensor1.reset(new frc::DigitalInput(0));
AddChild("magazineSensor1", magazineSensor1);

magazineSensor2.reset(new frc::DigitalInput(1));
AddChild("magazineSensor2", magazineSensor2);

intakeSolenoid.reset(new frc::Solenoid(0, 1));
AddChild("intakeSolenoid", intakeSolenoid);

leftRoller.reset(new rev::CANSparkMax(4, rev::CANSparkMax::MotorType::kBrushless));


rightRoller.reset(new rev::CANSparkMax(7, rev::CANSparkMax::MotorType::kBrushless));


leftSensor.reset(new frc::DigitalInput(2));
AddChild("leftSensor", leftSensor);

rightSensor.reset(new frc::DigitalInput(3));
AddChild("rightSensor", rightSensor);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    double intakeSpeed = 0.3;
	double magSpeed = 0.4;

    frc::SmartDashboard::PutNumber("Intake Speed",intakeSpeed);
    frc::SmartDashboard::PutNumber("Magazine Speed",magSpeed);
}

void Intakes::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void Intakes::Periodic() {
    // Put code here to be run every loop
    intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed",0.5);
    magSpeed = frc::SmartDashboard::GetNumber("Magazine Speed",0.5);
    frc::SmartDashboard::PutBoolean("Index1",magazineSensor1->Get());
    //index();
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

void Intakes::stopLeft(){
    leftRoller->Set(0);
}

void Intakes::stopRight(){
    rightRoller->Set(0);
}

void Intakes::setLeft(double speed){
    leftRoller->Set(-speed);
}

void Intakes::setRight(double speed){
    rightRoller->Set(speed);
}

void Intakes::spinIntakes(){
    setRight(INTAKESPEED);
    setLeft(INTAKESPEED);
    middleRoller->Set(INTAKESPEED);
}

void Intakes::stopIntakes(){
    setRight(0);
    setLeft(0);
    middleRoller->Set(0);
}

void Intakes::spitoutIntakes(){
    setRight(-INTAKESPEED);
    setLeft(-INTAKESPEED);
    middleRoller->Set(-INTAKESPEED);
}

void Intakes::deployIntakes(){
    intakeSolenoid->Set(true);
}

void Intakes::retractIntakes(){
    intakeSolenoid->Set(false);
}

void Intakes::spinMag(){
    magazine->Set(MAGSPEED);
}

void Intakes::stopMag(){
    magazine->Disable();
}

void Intakes::spitoutMag(){
    magazine->Set(-MAGSPEED);
}

bool Intakes::index(){
    if(idxState > 0)
        idxcnt++;
    switch(idxState){
        case 0:
            if(!magazineSensor2->Get())
                idxState = 1;
            break;
        case 1:
            magazine->Set(-0.4);
            prevDetect = magazineSensor2->Get();
            idxcnt = 0;
            if(!prevDetect)
                idxState = 1;
            else
                idxState = 2;
            break;
        case 2:
            //Wait until magazineSensor2 finds a gap
            prevDetect = magazineSensor2->Get();
            if((prevDetect)){
                idxState = 3;
                idxcnt = 0;
            }
            break;
        case 3:
            prevDetect = magazineSensor2->Get();
            if((!prevDetect)){
                //We have the ball in the right place, stop the motor
                magazine->Set(0);
                idxState = 0;
                return true;
            }
            break;
    }
    return false;
}
