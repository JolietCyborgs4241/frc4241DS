#include "Robot.h"
#include "subsystems/Ramp.h"
#include <iostream>
#include <frc/Timer.h>
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "WPILib.h"

using namespace frc;

OI* Robot::oi = NULL;
RobotArm* Robot::robotArm = NULL;
//Lift* Robot::lift = NULL;
Ramp* Robot::ramp = NULL;
DriveTrain* Robot::driveTrain = NULL;
Pigeon* Robot::pigeon = NULL;
/*Elevator* Robot::elevator = NULL;
Pneumatics* Robot::pneumatics = NULL;

LIDARLite* Robot::leftLidarLite = NULL;
LIDARLite* Robot::rightLidarLite = NULL;
*/
bool Robot::gyroAssist = false; 
//PigeonPID* Robot::gyroAssistPID = NULL;

bool Robot::fieldCentric = false;

// Pigeon* Robot::pigeon = NULL;
// Elevator* Robot::elevator = NULL;
// Pneumatics* Robot::pneumatics = NULL;

// LIDARLite* Robot::leftLidarLite = NULL;
// LIDARLite* Robot::rightLidarLite = NULL;

// bool Robot::gyroAssist = false;
// PigeonPID* Robot::gyroAssistPID = NULL;

// bool Robot::fieldCentric = true;
// bool Robot::elevatorPositionControl = false;
// bool Robot::useUpperLimitSwitch = true;

// MB1013Sensor* Robot::mb1013Sensor = NULL;

// std::string Robot::gameData = "";
// bool Robot::recievedGameData = false;
// Timer* Robot::autoTimer = new Timer();

void Robot::RobotInit() {
    RobotMap::init();
    CameraServer::GetInstance()->StartAutomaticCapture(0);
    oi = new OI();
   //lift = new Lift();
    ramp = new Ramp();
    driveTrain = new DriveTrain();
    pigeon = new Pigeon();
    // elevator = new Elevator();
    // pneumatics = new Pneumatics();

    
    robotArm = new RobotArm();
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

//     pigeon = new Pigeon();

//     gyroAssistPID = new PigeonPID();
//     gyroAssistPID->SetSetpoint(0);

//     mb1013Sensor = new MB1013Sensor();

//     leftLidarLite = new LIDARLite(13);
//     rightLidarLite = new LIDARLite(14);

//     chooser.AddDefault("NoAuto", 1);
//     chooser.AddObject("DriveForward-2", 2);
//     chooser.AddObject("LeftAuto", 3);
//     chooser.AddObject("RightAuto", 4);
//     chooser.AddObject("StraightSwitch", 5);
//     chooser.AddObject("CenterSwitch", 6);
//     chooser.AddObject("MidAuto", 7);
//     chooser.AddObject("LeftAutoSwitch", 8);
//     chooser.AddObject("RightAutoSwitch", 9);

//     SmartDashboard::PutData("Auto Modes", &chooser);

//     CameraServer::GetInstance()->StartAutomaticCapture(0);

     lw = LiveWindow::GetInstance();

      driveTrain->SetWheelbase(23, 23);
    FLOffset = 0;
    FROffset = 0;
    RLOffset = 0;
    RROffset = 0;

    // FL, FR, RL, RR
    driveTrain->SetOffsets(FLOffset, FROffset, RLOffset, RROffset);

    driveTrain->frontLeft->Enable();
    driveTrain->frontRight->Enable();
    driveTrain->rearLeft->Enable();
    driveTrain->rearRight->Enable();

//     driveTrain->SetWheelbase(24, 22, 24);
//     FLOffset = 0;
//     FROffset = 0;
//     RLOffset = 0;
//     RROffset = 0;

//     // FL, FR, RL, RR
//     driveTrain->SetOffsets(FLOffset, FROffset, RLOffset, RROffset);

//     driveTrain->frontLeft->Enable();
//     driveTrain->frontRight->Enable();
//     driveTrain->rearLeft->Enable();
//     driveTrain->rearRight->Enable();

//     pneumatics->Start();
 }

 void Robot::DisabledInit() {
//     // Makes sure that enabling the robot doesn't
//     // make the elevator shoot to the last position
//     elevatorPositionControl = false;
//     // RobotMap::elevatorMotor->Set(0);
 }

 void Robot::DisabledPeriodic() {
}

void Robot::AutonomousInit() {
    // pigeon->Update();
    // pigeon->SaveTilt();

    // driveTrain->EnablePIDs();

   //gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

    //autoTimer->Reset();
    //autoTimer->Start();
}

void Robot::Autonomous() {
  /*std::string autoSelected = m_chooser.GetSelected();
  // std::string autoSelected = frc::SmartDashboard::GetString(
  // "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << autoSelected << std::endl;

  */

}

void Robot::TeleopInit() {
//     // This makes sure that the autonomous stops running when
//     // teleop starts running. If you want the autonomous to
//     // continue until interrupted by another command, remove
//     // this line or comment it out.

     Scheduler::GetInstance()->RemoveAll();

     cycleTime = Timer::GetFPGATimestamp();

     pigeon->Update();
     pigeon->SaveTilt();

    driveTrain->EnablePIDs();

//     pneumatics->CloseClaw();
//     pneumatics->RetractPiston();
 }

 void Robot::TeleopPeriodic() {
     SmartDashboard::PutNumber("CycleTime", Timer::GetFPGATimestamp() - cycleTime);
     cycleTime = Timer::GetFPGATimestamp();
     Robot::robotArm->Fulcrum();
     driveTrain->Crab(-oi->getDriveLeftY(), oi->getDriveLeftX(), -oi->getDriveRightX(), fieldCentric);
     robotArm->ArmGetPosition();
    //driveTrain->Crab(0.0, 0.0, 0.0, 0.0); //useGyro is undefined, placed placeholder value, not sure what true value is 
    // driveTrain->Crab(0.0, 0.0, 0.0, fieldCentric); This function was in here before, but old code does not include fieldcentric
    //driveTrain->Crab(-oi->getDriveLeftY(), oi->getDriveLeftX(), -oi->getDriveRightX(), fieldCentric);
//     Drive Control
//     joystickY is -up, so invert to match +Y -> forward
//     joystickX is +right, so do nothing to match +X -> right
//     joystickZ is +right, so invert to match -twist -> clockwise (decrement angle on unit circle)

//     if (gyroAssist) {
//        driveTrain->Crab(-oi->getDriveLeftY(), oi->getDriveLeftX(), gyroAssistPID->GetOutput(), true);
//     } else {
//         driveTrain->Crab(-oi->getDriveLeftY(), oi->getDriveLeftX(), -oi->getDriveRightX(), fieldCentric);
//    }
     //Robot::robotArm->Fulcrum();
     
//     // Drive Control
//     // joystickY is -up, so invert to match +Y -> forward
//     // joystickX is +right, so do nothing to match +X -> right
//     // joystickZ is +right, so invert to match -twist -> clockwise (decrement angle on unit circle)
//     if (gyroAssist) {
//         driveTrain->Crab(-oi->getDriveLeftY(), oi->getDriveLeftX(), gyroAssistPID->GetOutput(), true);
//     } else {
//         driveTrain->Crab(-oi->getDriveLeftY(), oi->getDriveLeftX(), -oi->getDriveRightX(), fieldCentric);
//     }

//     /*
//     // Elevator Control
//     if (elevatorPositionControl) {
//         elevator->PositionUpdate();
//     } else {
//         elevator->MoveElevator();
//     }
    

//     elevator->MoveElevator();

     Dashboard();

     Scheduler::GetInstance()->Run();
 }

 void Robot::TestPeriodic() {
//     driveTrain->DisablePIDs();
//     Dashboard();
 }

 void Robot::Dashboard() {
//     // Joystick Variables(
      SmartDashboard::PutNumber("Claw Motor", RobotMap::robotArmClaw->GetMotorOutputVoltage());
      // Joystick Variables
    SmartDashboard::PutNumber("DriveStickY", oi->getDriveLeftY());
    SmartDashboard::PutNumber("DriveStickX", oi->getDriveLeftX());
    SmartDashboard::PutNumber("DriveStickZ", oi->getDriveRightX());

    // Wheel Module Voltages
    SmartDashboard::PutNumber("FrontLeftVol", driveTrain->frontLeftPos->GetAverageVoltage());
    SmartDashboard::PutNumber("FrontRightVol", driveTrain->frontRightPos->GetAverageVoltage());
    SmartDashboard::PutNumber("RearLeftVol", driveTrain->rearLeftPos->GetAverageVoltage());
    SmartDashboard::PutNumber("RearRightVol", driveTrain->rearRightPos->GetAverageVoltage());
    // Wheel Module Errors
    /*SmartDashboard::PutNumber("FLError", driveTrain->frontLeft->GetError());
    SmartDashboard::PutNumber("FRError", driveTrain->frontRight->GetError());
    SmartDashboard::PutNumber("RLError", driveTrain->rearLeft->GetError());
    SmartDashboard::PutNumber("RRError", driveTrain->rearRight->GetError()); */
    // Wheel Module Setpoints
    SmartDashboard::PutNumber("FLSetPoint", driveTrain->frontLeft->GetSetpoint());
    SmartDashboard::PutNumber("FRSetPoint", driveTrain->frontRight->GetSetpoint());
    SmartDashboard::PutNumber("RLSetPoint", driveTrain->rearLeft->GetSetpoint());
    SmartDashboard::PutNumber("RRSetPoint", driveTrain->rearRight->GetSetpoint());

    // Driver Motor Voltages 
    SmartDashboard::PutNumber("FLDrive", RobotMap::driveTrainFrontLeftDrive->GetMotorOutputVoltage());
    SmartDashboard::PutNumber("FRDrive", RobotMap::driveTrainFrontRightDrive->GetMotorOutputVoltage());
    SmartDashboard::PutNumber("RRDrive", RobotMap::driveTrainRearRightDrive->GetMotorOutputVoltage());
    SmartDashboard::PutNumber("RLDrive", RobotMap::driveTrainRearLeftDrive->GetMotorOutputVoltage());

    //SmartDashboard::PutBoolean("LimitSwitch", RobotMap::elevatorUpperLimitSwitch->Get());


    SmartDashboard::PutNumber("Pigeon-Yaw", pigeon->GetYaw());
    SmartDashboard::PutNumber("Talon 4 Output", RobotMap::driveTrainFrontLeftSteer->GetMotorOutputVoltage());
    SmartDashboard::PutNumber("Talon 2 Output", RobotMap::driveTrainRearLeftSteer->GetMotorOutputVoltage());
     SmartDashboard::PutNumber("Talon 6 Output", RobotMap::driveTrainRearRightSteer->GetMotorOutputVoltage());
    
    //Arm Extenstion Sensors
    SmartDashboard::PutNumber("Arm Degrees", round(RobotMap::armangle->Get()));
    SmartDashboard::PutNumber("Extension Length", robotArm->ArmGetPosition());
    
    // SmartDashboard::PutBoolean("Pigeon-AmTilted", pigeon->AmTilted());
    // SmartDashboard::PutBoolean("Pigeon-COLLIDED", pigeon->WasCollision());

     //SmartDashboard::PutNumber("ControlStickY", oi->getControlLY());
//     SmartDashboard::PutNumber("DriveStickX", oi->getDriveJoystick()->GetX());
//     SmartDashboard::PutNumber("DriveStickZ", oi->getDriveJoystick()->GetZ());

//     // Wheel Module Voltages
//     SmartDashboard::PutNumber("FrontLeftVol", driveTrain->frontLeftPos->GetAverageVoltage());
//     SmartDashboard::PutNumber("FrontRightVol", driveTrain->frontRightPos->GetAverageVoltage());
//     SmartDashboard::PutNumber("RearLeftVol", driveTrain->rearLeftPos->GetAverageVoltage());
//     SmartDashboard::PutNumber("RearRightVol", driveTrain->rearRightPos->GetAverageVoltage());
//     // Wheel Module Errors
//     SmartDashboard::PutNumber("FLError", driveTrain->frontLeft->GetError());
//     SmartDashboard::PutNumber("FRError", driveTrain->frontRight->GetError());
//     SmartDashboard::PutNumber("RLError", driveTrain->rearLeft->GetError());
//     SmartDashboard::PutNumber("RRError", driveTrain->rearRight->GetError());
//     // Wheel Module Setpoints
//     SmartDashboard::PutNumber("FLSetPoint", driveTrain->frontLeft->GetSetpoint());
//     SmartDashboard::PutNumber("FRSetPoint", driveTrain->frontRight->GetSetpoint());
//     SmartDashboard::PutNumber("RLSetPoint", driveTrain->rearLeft->GetSetpoint());
//     SmartDashboard::PutNumber("RRSetPoint", driveTrain->rearRight->GetSetpoint());

//     SmartDashboard::PutBoolean("LimitSwitch", RobotMap::elevatorUpperLimitSwitch->Get());

//     SmartDashboard::PutNumber("Pigeon-Yaw", pigeon->GetYaw());
//     SmartDashboard::PutBoolean("Pigeon-AmTilted", pigeon->AmTilted());
//     SmartDashboard::PutBoolean("Pigeon-COLLIDED", pigeon->WasCollision());

//     SmartDashboard::PutBoolean("Gyro-Assist", gyroAssist);
//     SmartDashboard::PutNumber("GyroPID-Pos", gyroAssistPID->GetPosition());
//     SmartDashboard::PutBoolean("GyroPID-OnTarget", gyroAssistPID->OnTarget());
//     SmartDashboard::PutNumber("GyroPID-Twist", gyroAssistPID->GetOutput());
//     SmartDashboard::PutNumber("GyroPID-Error", gyroAssistPID->GetDegError());

//     SmartDashboard::PutNumber("Elevator-Distance", elevator->GetDistance());
//     SmartDashboard::PutNumber("Elevator-Error", elevator->GetPIDError());

//     SmartDashboard::PutNumber("LidarLite-Left", leftLidarLite->SmoothedDistanceFeet());
//     SmartDashboard::PutNumber("LidarLite-Right", rightLidarLite->SmoothedDistanceFeet());

//     SmartDashboard::PutNumber("Back-Distance", mb1013Sensor->SmoothedDistanceFeet());

//     SmartDashboard::PutBoolean("FieldCentric", fieldCentric);

   

// bool Robot::fieldCentric = true;
// std::string Robot::gameData = "";
// bool Robot::recievedGameData = false;
 }


// void Robot::DisabledPeriodic() {
// }

   
//     gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
//}




// void Robot::TestPeriodic() {
//     Dashboard();
// }


START_ROBOT_CLASS(Robot); 
