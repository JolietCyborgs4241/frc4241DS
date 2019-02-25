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
  //m_fulcrum = RobotMap::robotArmFulcrum;
  m_extension = RobotMap::robotArmExtension;
  m_claw = RobotMap::robotArmClaw;
}

void RobotArm::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void RobotArm::openClaw() {
  if (RobotMap::limitswitchopen->Get()) {
    m_claw->Set(0.0);
  }
  else {
    m_claw->Set(1.0);
<<<<<<< HEAD
  /*if (RobotMap::limitswitchopen->Get()) {
    m_claw->Set(0.0);
  }
  else {*/
    m_claw->Set(0.45);
  //}
=======
  }
>>>>>>> parent of 46c576a... Undo this
}

void RobotArm::closeClaw() {
  if (RobotMap::limitswitchclose->Get()) {
    m_claw->Set(0.0)
  }
  else {
    m_claw->Set(-1.0);
<<<<<<< HEAD
  //}
 /* if (RobotMap::limitswitchclose->Get()) {
    m_claw->Set(0.0);
  }
  else {*/
    m_claw->Set(-0.45);
//  }
=======
  }
>>>>>>> parent of 46c576a... Undo this
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
<<<<<<< HEAD
   double motorValue =  -Robot::oi->getControlLY(); // INVERT the value!
/*void RobotArm::Fulcrum() {
=======
>>>>>>> parent of 46c576a... Undo this
   double motorValue = - (Robot::oi->getControlLY()); // INVERT the value!
   m_fulcrum->Set(motorValue);
} */

// Put methods for controlling this subsystem
// here. Call these from Commands.
