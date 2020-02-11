// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "OI.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "frc/smartdashboard/SmartDashboard.h"
#include "Commands/AutoAim.h"
#include "Commands/AutoModeOne.h"
#include "Commands/AutoModeTwo.h"
#include "Commands/AutonomousCommand.h"
#include "Commands/CloseShot.h"
#include "Commands/ColourControl.h"
#include "Commands/DeployClimber.h"
#include "Commands/DeployWheelWheel.h"
#include "Commands/DriveRobot.h"
#include "Commands/IntakeDeploy.h"
#include "Commands/IntakeUp.h"
#include "Commands/LongShot.h"
#include "Commands/ManualMagazine.h"
#include "Commands/MediumShot.h"
#include "Commands/PullUp.h"
#include "Commands/RetractWheelWheel.h"
#include "Commands/RotationControl.h"
#include "Commands/ShiftDownGear.h"
#include "Commands/ShiftUpGear.h"
#include "Commands/Shoot.h"


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

OI::OI() {
    // Process operator interface input here.
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
autoSwitch.reset(new frc::Joystick(2));

operatorGamepad.reset(new frc::Joystick(1));

operator_Btn2_A.reset(new frc::JoystickButton(operatorGamepad.get(), 2));
operator_Btn2_A->WhenPressed(new LongShot());
operator_Btn3_B.reset(new frc::JoystickButton(operatorGamepad.get(), 3));
operator_Btn3_B->WhenPressed(new MediumShot());
operator_Btn4_Y.reset(new frc::JoystickButton(operatorGamepad.get(), 4));
operator_Btn4_Y->WhenPressed(new CloseShot());
operator_Btn1_X.reset(new frc::JoystickButton(operatorGamepad.get(), 1));
operator_Btn1_X->WhenPressed(new ManualMagazine());
operator_Btn6_RB.reset(new frc::JoystickButton(operatorGamepad.get(), 6));
operator_Btn6_RB->WhenPressed(new RetractWheelWheel());
operator_Btn8_RT.reset(new frc::JoystickButton(operatorGamepad.get(), 8));
operator_Btn8_RT->WhenPressed(new PullUp());
operator_Btn10_Start.reset(new frc::JoystickButton(operatorGamepad.get(), 10));
operator_Btn10_Start->WhenPressed(new ColourControl());
operator_Btn9_Back.reset(new frc::JoystickButton(operatorGamepad.get(), 9));
operator_Btn9_Back->WhenPressed(new RotationControl());
operator_Btn5_LB.reset(new frc::JoystickButton(operatorGamepad.get(), 5));
operator_Btn5_LB->WhenPressed(new DeployWheelWheel());
operator_Btn7_LT.reset(new frc::JoystickButton(operatorGamepad.get(), 7));
operator_Btn7_LT->WhenPressed(new DeployClimber());
driverGamepad.reset(new frc::Joystick(0));

driver_Btn8_RT.reset(new frc::JoystickButton(driverGamepad.get(), 8));
driver_Btn8_RT->WhenPressed(new Shoot());
driver_Btn3_B.reset(new frc::JoystickButton(driverGamepad.get(), 3));
driver_Btn3_B->WhenPressed(new IntakeUp());
driver_Btn2_A.reset(new frc::JoystickButton(driverGamepad.get(), 2));
driver_Btn2_A->WhenPressed(new IntakeDeploy());
driver_Btn6_RB.reset(new frc::JoystickButton(driverGamepad.get(), 6));
driver_Btn6_RB->WhenPressed(new ShiftUpGear());
driver_Btn5_LB.reset(new frc::JoystickButton(driverGamepad.get(), 5));
driver_Btn5_LB->WhenPressed(new ShiftDownGear());
driver_Btn7_LT.reset(new frc::JoystickButton(driverGamepad.get(), 7));
driver_Btn7_LT->WhenPressed(new AutoAim());

    // SmartDashboard Buttons
    frc::SmartDashboard::PutData("ManualMagazine", new ManualMagazine());
    frc::SmartDashboard::PutData("LongShot", new LongShot());
    frc::SmartDashboard::PutData("MediumShot", new MediumShot());
    frc::SmartDashboard::PutData("CloseShot", new CloseShot());
    frc::SmartDashboard::PutData("RetractWheelWheel", new RetractWheelWheel());
    frc::SmartDashboard::PutData("PullUp", new PullUp());
    frc::SmartDashboard::PutData("ColourControl", new ColourControl());
    frc::SmartDashboard::PutData("RotationControl", new RotationControl());
    frc::SmartDashboard::PutData("DeployWheelWheel", new DeployWheelWheel());
    frc::SmartDashboard::PutData("DeployClimber", new DeployClimber());
    frc::SmartDashboard::PutData("Shoot", new Shoot());
    frc::SmartDashboard::PutData("IntakeUp", new IntakeUp());
    frc::SmartDashboard::PutData("IntakeDeploy", new IntakeDeploy());
    frc::SmartDashboard::PutData("ShiftUpGear", new ShiftUpGear());
    frc::SmartDashboard::PutData("ShiftDownGear", new ShiftDownGear());
    frc::SmartDashboard::PutData("AutoAim", new AutoAim());
    frc::SmartDashboard::PutData("AutoModeTwo", new AutoModeTwo());
    frc::SmartDashboard::PutData("AutoModeOne", new AutoModeOne());
    frc::SmartDashboard::PutData("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

std::shared_ptr<frc::Joystick> OI::getDriverGamepad() {
   return driverGamepad;
}

std::shared_ptr<frc::Joystick> OI::getOperatorGamepad() {
   return operatorGamepad;
}

std::shared_ptr<frc::Joystick> OI::getAutoSwitch() {
   return autoSwitch;
}


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
