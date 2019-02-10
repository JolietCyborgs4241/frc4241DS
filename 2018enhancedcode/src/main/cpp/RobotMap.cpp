#include "RobotMap.h"

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

#include "frc/liveWindow/LiveWindow.h"

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
WPI_TalonSRX* RobotMap::robotArmFulcrum = NULL;
WPI_TalonSRX* RobotMap::robotArmExtension = NULL;
WPI_TalonSRX* RobotMap::robotArmClaw = NULL;

/*WPI_TalonSRX* RobotMap::elevatorMotor = NULL;
DigitalInput* RobotMap::elevatorUpperLimitSwitch = NULL;
DigitalInput* RobotMap::elevatorBottomLimitSwitch = NULL; */

void RobotMap::init() {
    LiveWindow* lw = LiveWindow::GetInstance();

    robotArmFulcrum = new WPI_TalonSRX(1);
    robotArmExtension = new WPI_TalonSRX(2);
    robotArmClaw = new WPI_TalonSRX(3);
    /*elevatorMotor = new WPI_TalonSRX(4);
    elevatorMotor->ConfigOpenloopRamp(0.5, 10);
    elevatorMotor->ConfigClosedloopRamp(0.5, 10);

    elevatorUpperLimitSwitch = new DigitalInput(22);
    elevatorBottomLimitSwitch = new DigitalInput(1); */

    driveTrainFrontLeftDrive = new WPI_TalonSRX(60);
    // driveTrainFrontLeftDrive->ConfigOpenloopRamp(RAMP_TIME_TO_FULL, 10);

    driveTrainFrontRightDrive = new WPI_TalonSRX(61);
    // driveTrainFrontRightDrive->ConfigOpenloopRamp(RAMP_TIME_TO_FULL, 10);

    driveTrainRearLeftDrive = new WPI_TalonSRX(62);
    // driveTrainRearLeftDrive->ConfigOpenloopRamp(RAMP_TIME_TO_FULL, 10);

    driveTrainRearRightDrive = new WPI_TalonSRX(63);
    // driveTrainRearRightDrive->ConfigOpenloopRamp(RAMP_TIME_TO_FULL, 10);

    driveTrainRearRightPos = new AnalogInput(3);
    /*lw->AddSensor("DriveTrain", "RearRightPos", driveTrainRearRightPos);*/ //Syntax update for "lw->" fix it 
    driveTrainRearRightSteer = new WPI_TalonSRX(51);
    driveTrainRearRight = new PIDController(0.6, 0.0, 0.0, driveTrainRearRightPos, driveTrainRearRightSteer, 0.02);
    /*lw->AddActuator("DriveTrain", "RearRight", driveTrainRearRight);*/
    driveTrainRearRight->SetContinuous(true);
    driveTrainRearRight->SetAbsoluteTolerance(0.1);
    driveTrainRearRight->SetInputRange(0.0, 5.0);
    driveTrainRearRight->SetOutputRange(-1, 1);

    driveTrainFrontLeftPos = new AnalogInput(0);
    /*lw->AddSensor("DriveTrain", "FrontLeftPos", driveTrainFrontLeftPos);*/
    driveTrainFrontLeftSteer = new WPI_TalonSRX(52);
    driveTrainFrontLeft = new PIDController(0.6, 0.0, 0.0, driveTrainFrontLeftPos, driveTrainFrontLeftSteer, 0.02);
    /*lw->AddActuator("DriveTrain", "FrontLeft", driveTrainFrontLeft);*/
    driveTrainFrontLeft->SetContinuous(true);
    driveTrainFrontLeft->SetAbsoluteTolerance(0.1);
    driveTrainFrontLeft->SetInputRange(0.0, 5.0);
    driveTrainFrontLeft->SetOutputRange(-1, 1);

    driveTrainFrontRightPos = new AnalogInput(1);
    /*lw->AddSensor("DriveTrain", "FrontRightPos", driveTrainFrontRightPos);*/
    driveTrainFrontRightSteer = new WPI_TalonSRX(53);
    driveTrainFrontRight = new PIDController(0.6, 0.0, 0.0, driveTrainFrontRightPos, driveTrainFrontRightSteer, 0.02);
    /*lw->AddActuator("DriveTrain", "FrontRight", driveTrainFrontRight);*/
    driveTrainFrontRight->SetContinuous(true);
    driveTrainFrontRight->SetAbsoluteTolerance(0.1);
    driveTrainFrontRight->SetInputRange(0.0, 5.0);
    driveTrainFrontRight->SetOutputRange(-1, 1);

    driveTrainRearLeftPos = new AnalogInput(2);
    /*lw->AddSensor("DriveTrain", "RearLeftPos", driveTrainRearLeftPos);*/
    driveTrainRearLeftSteer = new WPI_TalonSRX(59);
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
