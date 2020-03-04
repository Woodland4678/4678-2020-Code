// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/Shooter.h"
#include "Robot.h"
#include <frc/SmartDashboard/SmartDashboard.h>
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    //Shooter PID values
    const double shooter_P = 0.08;
    const double shooter_I = 0.0001;
    const double shooter_D = 0.0002;

    const double hood_P = 0.5;
    const double hood_I = 0;
    const double hood_D = 0;

    const double hoodLowPos = 0;
    const double hoodMedPos = 0;
    const double hoodHighPos = 0;

    //acceptable hood error from target position
    const double acceptableError = 100; //change later
    const double acceptableVelError = 500;


Shooter::Shooter() : frc::Subsystem("Shooter") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
shooterMotor1.reset(new WPI_TalonSRX(1));


shooterMotor2.reset(new WPI_TalonSRX(2));


shooterMotor3.reset(new WPI_TalonSRX(3));


shooterMotor4.reset(new WPI_TalonSRX(4));


hoodMotor.reset(new WPI_TalonSRX(16));



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    // Slavery of the motors
    shooterMotor2->Follow(*shooterMotor1);
    shooterMotor3->Follow(*shooterMotor1);
    shooterMotor4->Follow(*shooterMotor1);
    // setting PID values of master
    shooterMotor1->Config_kP(0,shooter_P);
    shooterMotor1->Config_kI(0,shooter_I);
    shooterMotor1->Config_kD(0,shooter_D);

    shooterMotor1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0);
    shooterMotor1->SetSelectedSensorPosition(0);
    shooterMotor1->SetSensorPhase(true);
    shooterMotor1->ConfigPeakOutputForward(1);
	shooterMotor1->ConfigPeakOutputReverse(-1);
    shooterMotor1->ConfigAllowableClosedloopError(0,0);
    //shooterMotor1->ConfigVelocityMeasurementWindow();

    hoodMotor->Config_kP(0,hood_P);
    hoodMotor->Config_kI(0,hood_I);
    hoodMotor->Config_kD(0,hood_D);
    
    hoodMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0);
    hoodMotor->SetSelectedSensorPosition(0);

    frc::SmartDashboard::PutNumber("Shooter Set Point",0);
}

void Shooter::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void Shooter::Periodic() {
    frc::SmartDashboard::PutNumber("Shooter Speed", getShooterVel());
    frc::SmartDashboard::PutNumber("Shoot State", shootState);
    //frc::SmartDashboard::PutNumber("Ball Counter", ballCounter);
    frc::SmartDashboard::PutNumber("Target Velocity", getTargetShootVel());
    frc::SmartDashboard::PutNumber("Velocity Error", getVelError());
    frc::SmartDashboard::PutNumber("Aim Target", getAimTarget());
    frc::SmartDashboard::PutNumber("Hood Target", getHoodTarget());
    frc::SmartDashboard::PutNumber("Shooter Integrator",shooterMotor1->GetIntegralAccumulator());
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

void Shooter::SetShooterSpeed(double speed) {
    frc::SmartDashboard::PutNumber("Shooter Set Point",speed);
    shooterMotor1->Set(ControlMode::Velocity,(speed * 4096) / 600);
    //shooterMotor1->Set(speed);
}

void Shooter::stopShooter(){
    shooterMotor1->Disable();
}

/*
void Shooter::hoodLow(){
    hoodMotor->Set(ControlMode::Position,hoodLowPos);
    targetPos = hoodLowPos;
}

void Shooter::hoodMedium(){
    hoodMotor->Set(ControlMode::Position,hoodMedPos);
    targetPos = hoodMedPos;
}

void Shooter::hoodHigh(){
    hoodMotor->Set(ControlMode::Position,hoodHighPos);
    targetPos = hoodHighPos;
}
*/

bool Shooter::goToHoodPos(double target){
    //releaseHood();
    if (delayCount > 10) {
        hoodMotor->Set(ControlMode::Velocity,target);
        hoodTargetPos = target;
        if(findHoodError() <= acceptableError){
            count++;
        } else {count = 0;}
        if(count >= 10){
            //lockHood();
            count = 0;
            delayCount = 0;
            return true;
        }
    }
    delayCount++;
    return false;
}

double Shooter::readEncoderPos(){
    return hoodMotor->GetSelectedSensorPosition();
}

double Shooter::findHoodError(){
    return hoodTargetPos - readEncoderPos();
}

void Shooter::configHoodTarget(double target){
    hoodTargetPos = target;
}

void Shooter::configAimTarget(double target){
    autoAimTarget = target;
}

void Shooter::configShootVel(double target){
    shootTargetVel = target;
}

double Shooter::getHoodTarget(){
    return hoodTargetPos;
}

double Shooter::getAimTarget(){
    return autoAimTarget;
}

double Shooter::getTargetShootVel(){
    return shootTargetVel;
}

double Shooter::getShooterVel(){
    return ((shooterMotor1->GetSelectedSensorVelocity()* 600) / 4096);
}

double Shooter::getVelError(){
    return abs(shootTargetVel - shooterMotor1->GetSelectedSensorVelocity()); 
}

bool Shooter::shoot(){
    switch(shootState) {
        case 0:
            if (getVelError() >= acceptableVelError){
                SetShooterSpeed(getTargetShootVel());
            }
            else {
                shootState++;
            }
        break;
        case 1:
            if (Robot::intakes->index(true)){
                Robot::intakes->clearCellPosition(4);
                Robot::intakes->shiftCells(false);
                shootState++;
                if (Robot::intakes->countCells() == 0){
                    shootState = 0;
                    shootDelay = 0;
                    return true;
                }
            }
        break;
        case 2:
            shootDelay++;
            if (shootDelay >= 20){
                shootDelay = 0;
                shootState--;
            }
        break;
    }
    return false;
    
}

