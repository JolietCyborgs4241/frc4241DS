/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/RobotArm.h"
#include "Robot.h"
#include "Robotmap.h"
#include "ctre/Phoenix.h"
#include "cyborg_talons.h"
#include "math.h"
#include "subsytem_definitions.h"

using namespace frc;


RobotArm::RobotArm() : Subsystem("ExampleSubsystem") {
  m_fulcrum = RobotMap::robotArmFulcrum;
  m_extension = RobotMap::robotArmExtension;
  m_claw = RobotMap::robotArmClaw;
  m_claw->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, TALON_CONFIG_TIMEOUT);
  m_claw->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, TALON_CONFIG_TIMEOUT);
  armangle = RobotMap::armangle;

  m_extension->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLOOPIdx, kTimeoutMs);

  m_extension->Config_kF(kPIDLOOPIdx, kF, kTimeoutMs);
  m_extension->Config_kP(kPIDLOOPIdx, kP, kTimeoutMs);
  m_extension->Config_kI(kPIDLOOPIdx, kI, kTimeoutMs);
  m_extension->Config_kD(kPIDLOOPIdx, kD, kTimeoutMs);

}

void RobotArm::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void RobotArm::openClaw() {
  m_claw->Set(1.0);
}

void RobotArm::closeClaw() {
  m_claw->Set(-1.0);
 }

void RobotArm::StopClaw() {
  m_claw->Set(0.0);
}

void RobotArm::extendClaw() {
  if (RobotArm::ArmRackPosition() > ARM_MAX_EXTEND) {
    RobotArm::StopExtension();
  } else {
      m_extension->Set(1.0);
  }
}

void RobotArm::retractClaw() {
  if (RobotArm::ArmRackPosition() < 0) {
    RobotArm::StopExtension();
  } else {
    m_extension->Set(-1.0);
  }
}

void RobotArm::StopExtension() {
  m_extension->Set(0.0);
}

void RobotArm::Fulcrum() {
   double motorValue =  -Robot::oi->getControlLY(); // INVERT the value!
   
   if (motorValue == 0) {
     if (fabs(RobotArm::ArmRackPosition() - RobotArm::ArmRackMaxAllowed()) > ARM_EXTENSION_TOLERANCE) {
       if (RobotArm::ArmRackPosition() > RobotArm::ArmRackMaxAllowed()) {
         RobotArm::retractClaw();
       } else {
         RobotArm::extendClaw();
       } 
       } else {
         RobotArm::StopExtension();
       }
    
    m_fulcrum->Set(0.0);

    return;
    }

    if (motorValue < 0) {
      if (RobotArm::ArmRackPosition() > RobotArm::ArmRackMaxAllowed()) {
        RobotArm::retractClaw();
      } else {
        RobotArm::StopExtension();
      }
    } else {
        if (RobotArm::ArmRackPosition() < RobotArm::ArmRackMaxAllowed()) {
          RobotArm::extendClaw();
        } else {
          RobotArm::StopExtension();
        }
    }
   m_fulcrum->Set(motorValue);
}

double  RobotArm::ArmRackPosition() {
  double racklength;
  racklength = m_extension->GetSelectedSensorPosition(0) / PULSES_PER_REVOLUTION * (RACK_MOTOR_GEAR_TEETH/RACK_TEETH_PER_INCH);
  return racklength;
}

double RobotArm::ArmRackMaxAllowed() {
  float    armExtension[] = {
    0.0,
    0.0,
    0.0,
    0.1,
    0.1,
    0.2,
    0.3,
    0.4,
    0.6,
    0.7,
    0.9,
    1.1,
    1.3,
    1.5,
    1.7,
    2.0,
    2.3,
    2.6,
    2.9,
    3.2,
    3.6,
    4.0,
    4.4,
    4.9,
    5.3,
    5.8,
    6.3,
    6.9,
    7.5,
    8.1,
    8.7,
    9.4,
    10.1,
    10.8,
    11.6,
    12.4,
    13.3,
    14.2,
    15.1,
    16.1,
    17.2,
    18.3,
    19.4,
    20.7,
    21.9,
    23.3,
    24.7,
    26.2,
    27.8,
    29.5,
    31.3,
    33.1,
    35.1,
    37.2,
    39.4,
    41.8,
    44.3,
    47.0,
    49.9,
    53.0,
    56.3,
    59.8,
    63.6,
    67.7,
    72.1,
    76.8,
    82.0,
    87.7,
    93.9,
    100.7,
    108.2,
    116.5,
    125.8,
    136.1,
    147.8,
    161.1,
    176.3,
    193.8,
    214.3,
    238.5,
    267.7,
    303.3,
    347.9,
    405.3,
    481.9,
    589.1,
    750.1,
    1018.5,
    1555.5,
    3166.8,
  };

  int angleIndex;

  angleIndex = (int)(round(RobotMap::armangle->Get()));

  if (angleIndex < 0 || angleIndex > (sizeof(armExtension) / sizeof(armExtension[0]))) {
    return 0;
  } else {
    return armExtension[angleIndex];
  }





}

double RobotArm::GetArmDegrees() {
  double angle;
  angle = round(RobotArm::armangle->Get());
  return angle;
}

void RobotArm::FulcrumMatchSet() {

  //Used before matches to set Arm extension and Angle for proper calibration of 
  //potentiometer, Only used in Test Periodic
   m_fulcrum->Set(-Robot::oi->getControlLY()); // INVERT the value!



}

void RobotArm::ArmExtensionSet() {

  //Used before matches to set Arm extension for proper calibration 
  //of Mag Encoder, Only used in Test Periodic
  m_extension->Set(-Robot::oi->getControlRY());
}

