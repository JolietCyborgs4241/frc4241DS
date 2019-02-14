/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Ramp.h"
Ramp::Ramp() : Subsystem("ExampleSubsystem") {
  RampRelay = new Relay(1);
}
void Ramp::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void Ramp::RampDeploy() {
  RampRelay->Set(Relay::Value::kOn);
}

void Ramp::RampUp() {
  RampRelay->Set(Relay::Value::kOff);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
