// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "Subsystems/Drivetrain.h"
#include "Subsystems/PathFinder/Path.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "Commands/DriveRobot.h"

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

//RPM PID for NEO Motor
const double V_leftP = 0.000130;
const double V_leftI = 1.05e-6;
const double V_leftD = 0.0;
const double V_leftF = 0.0;

const double V_rightP = 0.000130;
const double V_rightI = 1.05e-6;
const double V_rightD = 0.0;
const double V_rightF = 0.0;

//Position PID for NEO Motors
const double P_leftP = 0.05;
const double P_leftI = 0;
const double P_leftD = 0;

const double P_rightP = 0.05;
const double P_rightI = 0;
const double P_rightD = 0;

const unsigned int CurrenLimit = 50;

const double SpeedConvert = (1 / 0.0007563645);

Drivetrain::Drivetrain() : frc::Subsystem("Drivetrain") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
leftMaster.reset(new rev::CANSparkMax(1, rev::CANSparkMax::MotorType::kBrushless));


leftSlave.reset(new rev::CANSparkMax(2, rev::CANSparkMax::MotorType::kBrushless));


rightMaster.reset(new rev::CANSparkMax(3, rev::CANSparkMax::MotorType::kBrushless));


rightSlave.reset(new rev::CANSparkMax(5, rev::CANSparkMax::MotorType::kBrushless));


shifter.reset(new frc::Solenoid(0, 4));
AddChild("shifter", shifter);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    m_Gyro = new frc::ADXRS450_Gyro();

    limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    
    //Slave setup
    leftSlave->Follow(*leftMaster);
    rightSlave->Follow(*rightMaster);

    //PID Setup for RPM
    leftMaster->GetPIDController().SetP(V_leftP,0);
    leftMaster->GetPIDController().SetI(V_leftI,0);
    leftMaster->GetPIDController().SetD(V_leftD,0);
    leftMaster->GetPIDController().SetFF(V_leftF,0);
    rightMaster->GetPIDController().SetP(V_rightP,0);
    rightMaster->GetPIDController().SetI(V_rightI,0);
    rightMaster->GetPIDController().SetD(V_rightD,0);
    rightMaster->GetPIDController().SetFF(V_rightF,0);

    //PID Setup for Position
    leftMaster->GetPIDController().SetP(P_leftP,1);
    leftMaster->GetPIDController().SetI(P_leftI,1);
    leftMaster->GetPIDController().SetD(P_leftD,1);
    rightMaster->GetPIDController().SetP(P_rightP,1);
    rightMaster->GetPIDController().SetI(P_rightI,1);
    rightMaster->GetPIDController().SetD(P_rightD,1);

    //Output Setup
    leftMaster->GetPIDController().SetOutputRange(-1, 1);
    rightMaster->GetPIDController().SetOutputRange(-1, 1);

    //Current Limiting
    leftMaster->SetSmartCurrentLimit(CurrenLimit);
    rightMaster->SetSmartCurrentLimit(CurrenLimit);

    m_Path = new PathFinder(0.02,3,2,1,0.545);

    mAA_p = 0.19;
    mAA_i = 0.001;
	mAA_d = 0;

    frc::SmartDashboard::PutNumber("Left Set Speed",0);
    frc::SmartDashboard::PutNumber("Right Set Speed",0);
    frc::SmartDashboard::PutNumber("cnt",0);
    frc::SmartDashboard::PutNumber("Element",0);
}

void Drivetrain::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        SetDefaultCommand(new DriveRobot());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void Drivetrain::Periodic() {
    // Put code here to be run every loop
    frc::SmartDashboard::PutNumber("Left Speed",getLeftRPM());
    frc::SmartDashboard::PutNumber("Right Speed",getRightRPM());
    frc::SmartDashboard::PutNumber("Gyro",getGyroReading());

    frc::SmartDashboard::PutNumber("Left Encoder",getLeftEncoder());
    frc::SmartDashboard::PutNumber("Right Encoder",getRightEncoder());

    double SpeedErrorL = l_Set - getLeftRPM();
    double SpeedErrorR = r_Set - getRightRPM();

    frc::SmartDashboard::PutNumber("Left I Accum",leftMaster->GetPIDController().GetIAccum());
    frc::SmartDashboard::PutNumber("Right I Accum",rightMaster->GetPIDController().GetIAccum());

    frc::SmartDashboard::PutNumber("Left Error",SpeedErrorL);
    frc::SmartDashboard::PutNumber("Right Error",SpeedErrorR);

    ml_ValidTarget = limelight->GetNumber("tv",0.0);
    if(ml_ValidTarget){
        ml_targetHorizontial = limelight->GetNumber("tx",0.0);
        ml_targetVertical = limelight->GetNumber("ty",0.0);

        frc::SmartDashboard::PutNumber("Hor",ml_targetHorizontial);
        frc::SmartDashboard::PutNumber("Ver",ml_targetVertical);
    }
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.
#pragma region Gyro Function

double Drivetrain::getGyroReading(){
    return m_Gyro->GetAngle();
}

void Drivetrain::resetGyro(){
    m_Gyro->Reset();
}

void Drivetrain::calibrateGyro(){
    m_Gyro->Calibrate();
}

#pragma endregion

#pragma region Motor Low Level Control Functions
//------------------------------------------Set Functions------------------------------------------
void Drivetrain::setLeftRPM(double rpm){
    frc::SmartDashboard::PutNumber("Left Set Speed",-rpm);
    l_Set = -rpm;
    leftMaster->GetPIDController().SetReference(-rpm, rev::ControlType::kVelocity,0);
}

void Drivetrain::setRightRPM(double rpm){
    frc::SmartDashboard::PutNumber("Right Set Speed",rpm);
    r_Set = rpm;
    rightMaster->GetPIDController().SetReference(rpm, rev::ControlType::kVelocity,0);
}

void Drivetrain::setLeftVelocity(double mps){
    double rpm = mps * SpeedConvert;  // speed convert relates wheel diameter & gear ratio to speed
    setLeftRPM(rpm);
}

void Drivetrain::setRightVelocity(double mps){
    double rpm = mps * SpeedConvert;
    setRightRPM(rpm);
}

void Drivetrain::setLeftPosition(double encoder) {
    leftMaster->GetPIDController().SetReference(encoder,rev::ControlType::kPosition,1);
}

void Drivetrain::setRightPosition(double encoder) {
    rightMaster->GetPIDController().SetReference(encoder, rev::ControlType::kPosition,1);
}

void Drivetrain::setRightPower(double pwr) {
    rightMaster->Set(pwr);
}
void Drivetrain::setLeftPower(double pwr) {
    leftMaster->Set(-pwr);
}
#pragma endregion

#pragma region Motor Get Functions
//------------------------------------------Get Functions------------------------------------------
double Drivetrain::getLeftEncoder() {
   return leftMaster->GetEncoder().GetPosition();
}

double Drivetrain::getRightEncoder() {
    return rightMaster->GetEncoder().GetPosition();
}

//Returns RPM
double Drivetrain::getLeftRPM(){
    return leftMaster->GetEncoder().GetVelocity();
}

double Drivetrain::getRightRPM(){
    return rightMaster->GetEncoder().GetVelocity();
}

double Drivetrain::getLeftVelocity(){
    double rpm = leftMaster->GetEncoder().GetVelocity();
    return rpm / SpeedConvert;
}

double Drivetrain::getRightVelocity(){
    double rpm = rightMaster->GetEncoder().GetVelocity();
    return rpm / SpeedConvert;
}
#pragma endregion

#pragma region Limelight
//--------------------------------------------Limelight-----------------------------------------------
void Drivetrain::setLimeLED(bool state){
    if(state == 0)
        limelight->PutNumber("ledMode",1);//Turn off
    else
        limelight->PutNumber("ledMode",3);//Turn on
}

void Drivetrain::SetLimeFar(){
    limelight->PutNumber("pipeline", 0.0);
}

void Drivetrain::SetLimeZoomed(){
    limelight->PutNumber("pipeline", 1.0);
}


bool Drivetrain::getLimeValidObject(){
    return ml_ValidTarget; //Read in the periodic function 
}

double Drivetrain::getLimeHorizontial(){
    return ml_targetHorizontial;//Read in the periodic function
}

double Drivetrain::getLimeVertical(){
    return ml_targetVertical;//Read in the periodic function
}
#pragma endregion

#pragma region Main Control Functions
//-----------------------------------------Advanced Controls------------------------------------------
bool Drivetrain::turnAmount(double degrees, int direction, double vel, double acc){
    if((mt_tarDeg != degrees) || (mt_tarDir != direction) || (mt_vel != vel))
        mt_state = 0;
    //frc::SmartDashboard::PutNumber("State",mt_state);
    switch(mt_state) {
        case 0:
            {
            mt_acc=acc;
            mt_vel=vel;
            mt_tarDeg=degrees;
            mt_tarDir=direction;

            double cycleTime = 1.0/50.0;
            double dist;
            
			//Calculate distance based on angle to move
			//wheel base = 0.545m
			//	PI * 0.545 = 360 degrees
			//	distance to move each wheel = pi * 0.545 * degrees / 360
			
			dist = (M_PI * 0.545 * degrees / 360) * 1.25;
			
            
            //Trapizodal Calculations
            double t1 = mt_vel / mt_vel, t2, t3;
            double d1 = 0.5 * mt_vel * (t1*t1), d2, d3 = d1;
            if(2*d1 > dist) { //No cruise
                d1 = dist / 2;
                d2 = 0;
                d3 = d1;
                t1 = sqrt(2 * d1 / mt_vel);
                t2 = t1;
                t3 = t2 + t1;
            }
            else {
                d2 = dist - 2 * d1;
                t2 = t1 + d2 / mt_vel;
                t3 = t2 + t1;
            }

            //Path generation of one wheel
            d[0] = 0;
            int i=0;
            
            for(double tcnt = 0;tcnt<t3;tcnt += cycleTime,i++) {
                if(tcnt < t1)
                    d[i] = 0.5 * mt_vel * (tcnt*tcnt);
                else if(tcnt < t2)
                    d[i] = d1 + (mt_vel * (tcnt-t1));
                else if(tcnt < t3)
                    d[i] = d1+d2+d3 - (0.5 * mt_vel * ((t3-tcnt)*(t3-tcnt)));
            }
            d[i++] = d1+d2+d3;
            mt_Cycles = i;
            printf("\nTURN: Run: cycles=%i",i);

            traverseCnt = 0;
            mt_state = 1;
            mt_OEncLeft = getLeftEncoder();
            mt_OEncRight = getRightEncoder();
            }
            break;
        case 1:
            //frc::SmartDashboard::PutNumber("cnt",traverseCnt);
            //frc::SmartDashboard::PutNumber("Total",mt_Cycles);
            if(traverseCnt >= mt_Cycles) {
                mt_state = 0;
                return true;
            }
            else {
                //Main path velocity
                double distance = d[traverseCnt];
                if(direction)
                    distance *= -1;
                double enDist = (distance * 20.81); //velocity is per second not per cycle
                //left wheel
                encLeft = enDist + mt_OEncLeft;
                encRight = enDist + mt_OEncRight;
                
                //printf("\nTURN: Cnt = %i | dist = %f | EncLeft = %f EncRight = %f",traverseCnt, distance, encLeft,encRight);
                setLeftPosition(encLeft);
                setRightPosition(encRight);
                traverseCnt++;
            }
            break;
    }
    return false;
}


void Drivetrain::initPath() {
    pathState = 0;
}

bool Drivetrain::testPath() {
    switch(pathState) {
        case 0:
            //printf("\nStarting Path Weaving");
            m_Path->createNewPath();
            m_Path->addWayPoint(0.0,0.0,0.0);
            m_Path->addWayPoint(4,4,89); //2 meters x direction
            tCnt = 0;

            if(m_Path->makePath()) {
                m_Path->debug();
                m_Path->startTraverse(frc::Timer::GetFPGATimestamp());
                pathState++;
            }
            else{
                pathState = 0;
                return true;
            }
            break;
        case 1:
            {
            if(!tCnt)
                m_Path->debug();
            bool tdone = m_Path->traverse(frc::Timer::GetFPGATimestamp(),&rVel,&lVel,getGyroReading());
            frc::SmartDashboard::PutNumber("cnt",tCnt);

            frc::SmartDashboard::PutNumber("tr_R",rVel);
            frc::SmartDashboard::PutNumber("tr_L",lVel);
            setRightVelocity(rVel);
            setLeftVelocity(lVel);
            tCnt++;
            if(tdone){
                setRightVelocity(0);
                setLeftVelocity(0);
            }
            return tdone;
            }
            break;
    }
    return false;
}

//-------------------------------------------Auto Controls--------------------------------------------
double Drivetrain::autoAim(double target){
    double error = 0;
	static double past = 0;
	static double iValue = 0;
	//static int counter = 0;
	double current = ml_targetHorizontial;//Target position calculated in periodic function
    //TODO: If we lose the target, reset the I value

	error = target - (current / 30); //Divided by 30 because that value is the maximum given the resolution of the camera
	double pValue = mAA_p*error;
	iValue += mAA_i*(error);
	double dValue = mAA_d*(past - current);
	double totalValue = pValue + iValue + dValue;

	if(totalValue > 0.8)
		totalValue = 0.8;
	if(totalValue < -0.8)
		totalValue = -0.8;
	setRightPower(-totalValue);
	setLeftPower(totalValue);

	past = current;
	/*if ((std::abs(error) < 1.5)||((fabs(totalValue) < 0.1)&&(fabs(error) < 8))) {
		counter++;
	} else {
		counter = 0;
	}
	if(counter >= 10){
		past = 0;
		iValue = 0;
		counter = 0;
		setRightPower(0);
		setLeftPower(0);
		return true;
	}*/
	return error;
}
#pragma endregion

void Drivetrain::shiftUp(){
    shifter->Set(true);
}

void Drivetrain::shiftDown(){
    shifter->Set(false);
}
