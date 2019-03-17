#include "RobotMap.h"

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

#include "frc/liveWindow/LiveWindow.h"

#include "cyborg_talons.h"	// all Talon constants live here
#include "subsytem_definitions.h" //Definitions for Sensor Values

using namespace frc;




WPI_TalonSRX* RobotMap::driveTrainFrontLeftDrive = NULL;
WPI_TalonSRX* RobotMap::driveTrainFrontRightDrive = NULL;
WPI_TalonSRX* RobotMap::driveTrainRearLeftDrive = NULL;
WPI_TalonSRX* RobotMap::driveTrainRearRightDrive = NULL;

AnalogInput* RobotMap::driveTrainFrontLeftPos = NULL;
WPI_TalonSRX* RobotMap::driveTrainFrontLeftSteer = NULL;
PIDController* RobotMap::driveTrainFrontLeft = NULL;

AnalogInput* RobotMap::driveTrainFrontRightPos = NULL;
WPI_TalonSRX* RobotMap::driveTrainFrontRightSteer = NULL;
PIDController* RobotMap::driveTrainFrontRight = NULL;

AnalogInput* RobotMap::driveTrainRearLeftPos = NULL;
WPI_TalonSRX* RobotMap::driveTrainRearLeftSteer = NULL;
PIDController* RobotMap::driveTrainRearLeft = NULL;

AnalogInput* RobotMap::driveTrainRearRightPos = NULL;
WPI_TalonSRX* RobotMap::driveTrainRearRightSteer = NULL;
PIDController* RobotMap::driveTrainRearRight = NULL;
Pigeon* RobotMap::pigeon = NULL;
WPI_TalonSRX* RobotMap::ramp = NULL;
WPI_TalonSRX* RobotMap::lift = NULL;
WPI_TalonSRX* RobotMap::robotArmClaw = NULL;
WPI_TalonSRX* RobotMap::robotArmExtension = NULL;
WPI_TalonSRX* RobotMap::robotArmFulcrum = NULL;
Potentiometer* RobotMap::armangle = NULL;


/*WPI_TalonSRX* RobotMap::elevatorMotor = NULL;
DigitalInput* RobotMap::elevatorUpperLimitSwitch = NULL;
DigitalInput* RobotMap::elevatorBottomLimitSwitch = NULL; */

void RobotMap::init() {
    LiveWindow* lw = LiveWindow::GetInstance();
    ramp = new WPI_TalonSRX(TALON_RAMPS);
    //lift = new WPI_TalonSRX(TALON_LIFT);
    robotArmClaw = new WPI_TalonSRX(TALON_ARM_CLAW);
    robotArmExtension = new WPI_TalonSRX(TALON_ARM_EXTEND);
    robotArmFulcrum = new WPI_TalonSRX(TALON_LIFT);
    armangle = new AnalogPotentiometer(POT_ANALOG_INPUT, POT_ANGULAR_RANGE, POT_ANGLE_HORIZ_OFFSET); //Set analog input slot for potentiometer, Angle set for between 0 and 270 degrees

    driveTrainFrontLeftDrive = new WPI_TalonSRX(TALON_FL_DRIVE);
    driveTrainFrontLeftSteer = new WPI_TalonSRX(TALON_FL_STEER);
    // driveTrainFrontLeftDrive->ConfigOpenloopRamp(TALON_DRIVE_RAMP_TIME, TALON_CONFIG_TIMEOUT);

    driveTrainFrontRightDrive = new WPI_TalonSRX(TALON_FR_DRIVE);
    driveTrainFrontRightSteer = new WPI_TalonSRX(TALON_FR_STEER);

    // driveTrainFrontRightDrive->ConfigOpenloopRamp(TALON_DRIVE_RAMP_TIME, TALON_CONFIG_TIMEOUT);

    driveTrainRearLeftDrive = new WPI_TalonSRX(TALON_RL_DRIVE);
    driveTrainRearLeftSteer = new WPI_TalonSRX(TALON_RL_STEER);
    // driveTrainRearLeftDrive->ConfigOpenloopRamp(TALON_DRIVE_RAMP_TIME, TALON_CONFIG_TIMEOUT);

    driveTrainRearRightDrive = new WPI_TalonSRX(TALON_RR_DRIVE);
     driveTrainRearRightSteer = new WPI_TalonSRX(TALON_RR_STEER);
    // driveTrainRearRightDrive->ConfigOpenloopRamp(TALON_DRIVE_RAMP_TIME, TALON_CONFIG_TIMEOUT);

    driveTrainRearRightPos = new AnalogInput(RR_ENCODER);
    /*lw->AddSensor("DriveTrain", "RearRightPos", driveTrainRearRightPos);*/  
   
    driveTrainRearRight = new PIDController(0.6, 0.0, 0.0, driveTrainRearRightPos, driveTrainRearRightSteer, 0.02);
    /*lw->AddActuator("DriveTrain", "RearRight", driveTrainRearRight);*/
    driveTrainRearRight->SetContinuous(true);
    driveTrainRearRight->SetAbsoluteTolerance(0.1);
    driveTrainRearRight->SetInputRange(0.0, 5.0);
    driveTrainRearRight->SetOutputRange(-1, 1);

    driveTrainFrontLeftPos = new AnalogInput(FL_ENCODER);
    /*lw->AddSensor("DriveTrain", "FrontLeftPos", driveTrainFrontLeftPos);*/
    
    driveTrainFrontLeft = new PIDController(0.6, 0.05, 0.0, driveTrainFrontLeftPos, driveTrainFrontLeftSteer, 0.02);
    /*lw->AddActuator("DriveTrain", "FrontLeft", driveTrainFrontLeft);*/
    driveTrainFrontLeft->SetContinuous(true);
    driveTrainFrontLeft->SetAbsoluteTolerance(0.1);
    driveTrainFrontLeft->SetInputRange(0.0, 5.0);
    driveTrainFrontLeft->SetOutputRange(-1, 1);

    driveTrainFrontRightPos = new AnalogInput(FR_ENCODER);
    /*lw->AddSensor("DriveTrain", "FrontRightPos", driveTrainFrontRightPos);*/
    driveTrainFrontRight = new PIDController(0.6, 0.0, 0.0, driveTrainFrontRightPos, driveTrainFrontRightSteer, 0.02);
    /*lw->AddActuator("DriveTrain", "FrontRight", driveTrainFrontRight);*/
    driveTrainFrontRight->SetContinuous(true);
    driveTrainFrontRight->SetAbsoluteTolerance(0.1);
    driveTrainFrontRight->SetInputRange(0.0, 5.0);
    driveTrainFrontRight->SetOutputRange(-1, 1);

    driveTrainRearLeftPos = new AnalogInput(RL_ENCODER);
    /*lw->AddSensor("DriveTrain", "RearLeftPos", driveTrainRearLeftPos);*/
    
    driveTrainRearLeft = new PIDController(0.6, 0.0, 0.0, driveTrainRearLeftPos, driveTrainRearLeftSteer, 0.02);
    /*lw->AddActuator("DriveTrain", "RearLeft", driveTrainRearLeft);*/
    driveTrainRearLeft->SetContinuous(true);
    driveTrainRearLeft->SetAbsoluteTolerance(0.1);
    driveTrainRearLeft->SetInputRange(0.0, 5.0);
    driveTrainRearLeft->SetOutputRange(-1, 1);

    driveTrainFrontLeftPos->SetAverageBits(256);
    driveTrainFrontRightPos->SetAverageBits(256);
    driveTrainRearLeftPos->SetAverageBits(256);
    driveTrainRearRightPos->SetAverageBits(256);
}
