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
}
void Ramp::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void Ramp::RampDeploy() {
  RampMolo->Set(1.0);
}


// Put methods for controlling this subsystem
// here. Call these from Commands.
