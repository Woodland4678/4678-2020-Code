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

#include "frc/Joystick.h"
#include "frc/buttons/JoystickButton.h"

class OI {
private:
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS




std::shared_ptr<frc::Joystick> autoSwitch;
std::shared_ptr<frc::Joystick> operatorGamepad;
std::shared_ptr<frc::JoystickButton> operator_Btn2_A;
std::shared_ptr<frc::JoystickButton> operator_Btn3_B;
std::shared_ptr<frc::JoystickButton> operator_Btn4_Y;
std::shared_ptr<frc::JoystickButton> operator_Btn1_X;
std::shared_ptr<frc::JoystickButton> operator_Btn6_RB;
std::shared_ptr<frc::JoystickButton> operator_Btn8_RT;
std::shared_ptr<frc::JoystickButton> operator_Btn10_Start;
std::shared_ptr<frc::JoystickButton> operator_Btn9_Back;
std::shared_ptr<frc::JoystickButton> operator_Btn5_LB;
std::shared_ptr<frc::JoystickButton> operator_Btn7_LT;
std::shared_ptr<frc::Joystick> driverGamepad;
std::shared_ptr<frc::JoystickButton> driver_Btn8_RT;
std::shared_ptr<frc::JoystickButton> driver_Btn3_B;
std::shared_ptr<frc::JoystickButton> driver_Btn2_A;
std::shared_ptr<frc::JoystickButton> driver_Btn6_RB;
std::shared_ptr<frc::JoystickButton> driver_Btn5_LB;
std::shared_ptr<frc::JoystickButton> driver_Btn7_LT;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public:
	OI();

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PROTOTYPES

std::shared_ptr<frc::Joystick> getDriverGamepad();
std::shared_ptr<frc::Joystick> getOperatorGamepad();
std::shared_ptr<frc::Joystick> getAutoSwitch();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PROTOTYPES
};

