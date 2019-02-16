/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Ramp.h"
#include "RobotMap.h"
#include "Robot.h"
Ramp::Ramp() : Subsystem("ExampleSubsystem") {
  RampMolo = RobotMap::ramp;
  initString = "INIT!";
}
void Ramp::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void Ramp::RampDeploy() {
  RampMolo->Set(ControlMode::PercentOutput, 1.0);
}


void Ramp::RampUnDeploy() {
  RampMolo->Set(ControlMode::PercentOutput, 0.0);
}

char *Ramp::getInitString() {
  return Ramp::initString;
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
