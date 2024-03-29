// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include <frc/smartdashboard/smartdashboard.h>
#include <frc/util/color.h>

#include "Subsystems/ControlPanel.h"
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

ControlPanel::ControlPanel() : frc::Subsystem("ControlPanel") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
controlPanelSolenoid.reset(new frc::Solenoid(0, 2));
AddChild("controlPanelSolenoid", controlPanelSolenoid);

wheelMotor.reset(new rev::CANSparkMax(12, rev::CANSparkMax::MotorType::kBrushless));



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    m_ColourMatcher.AddColorMatch(m_BlueTarget);
    m_ColourMatcher.AddColorMatch(m_GreenTarget);
    m_ColourMatcher.AddColorMatch(m_RedTarget);
    m_ColourMatcher.AddColorMatch(m_YellowTarget);
    
}

void ControlPanel::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void ControlPanel::Periodic() {
    // Put code here to be run every loop
    frc::Color detectedColour = m_ColourSensor.GetColor();
    frc::Color matchedColour = m_ColourMatcher.MatchClosestColor(detectedColour, m_Confidence);

    if (matchedColour == m_BlueTarget) {
      m_ColourString = "Blue";
    } else if (matchedColour == m_RedTarget) {
      m_ColourString = "Red";
    } else if (matchedColour == m_GreenTarget) {
      m_ColourString = "Green";
    } else if (matchedColour == m_YellowTarget) {
      m_ColourString = "Yellow";
    } else {
      m_ColourString = "Unknown";
    }

    m_Red = detectedColour.red;
    m_Green = detectedColour.green;
    m_Blue = detectedColour.blue;
    m_IR = m_ColourSensor.GetIR();

    m_Proximity = m_ColourSensor.GetProximity();

    frc::SmartDashboard::PutNumber("Red", m_Red);
    frc::SmartDashboard::PutNumber("Blue", m_Blue);
    frc::SmartDashboard::PutNumber("Green", m_Green);
    frc::SmartDashboard::PutNumber("Infrared", m_IR);
    frc::SmartDashboard::PutNumber("Proximity", m_Proximity);
    frc::SmartDashboard::PutString("Detecting:", m_ColourString);
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.

void ControlPanel::deployWheelManipulator(){
    controlPanelSolenoid->Set(true);
}

void ControlPanel::retractWheelManipulator(){
    controlPanelSolenoid->Set(false);
    wheelMotor->Disable();
}
void ControlPanel::spinControlPanel() {
  //wheelMotor->GetPIDController().SetReference(2000,rev::ControlType::kPosition,0);
  //wheelMotor->Set(0.5);
  //if (wheelMotor->GetEncoder().GetPosition() > 1900) {
  //  return true;
 // }
 // return false;
}
