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

using namespace frc;

RobotArm::RobotArm() : Subsystem("ExampleSubsystem") {
  m_fulcrum = RobotMap::robotArmFulcrum;
  m_extension = RobotMap::robotArmExtension;
  m_claw = RobotMap::robotArmClaw;
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
  m_extension->Set(1.0);
}

void RobotArm::retractClaw() {
  m_extension->Set(-1.0);
}

void RobotArm::StopExtension() {
  m_extension->Set(0.0);
}

void RobotArm::Fulcrum() {
   double motorValue = - (Robot::oi->getControlLY()); // INVERT the value!
   SmartDashboard::PutNumber("RobotArm::Fulcrum - motorValue", motorValue);
   m_fulcrum->Set(motorValue);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
