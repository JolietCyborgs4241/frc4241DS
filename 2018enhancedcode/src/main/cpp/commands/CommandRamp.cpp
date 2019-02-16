/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "Robot.h"
#include "commands/CommandRamp.h"
#include <iostream>

CommandRamp::CommandRamp() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  std::cout << "CommandRamp:CommandRamp";

  CommandRamp::RampTimer = new Timer;
}

// Called just before this Command runs the first time
void CommandRamp::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CommandRamp::Execute() {
  Robot::ramp->RampDeploy();
  SmartDashboard::PutString("CR:timer", "rst/str");
  CommandRamp::RampTimer->Reset();
  CommandRamp::RampTimer->Start();
  SmartDashboard::PutString("CR:timer", "started");
}

// Make this return true when this Command no longer needs to run execute()
bool CommandRamp::IsFinished() {
  SmartDashboard::PutNumber("CR:IsFinished", CommandRamp::RampTimer->Get());
  if (CommandRamp::RampTimer->Get() < RAMP_ACTIVATION_LIMIT) {
    SmartDashboard::PutString("CR:IsFinishedStr", "Keep going");
    return false;
  } 

  CommandRamp::RampTimer->Stop();

  SmartDashboard::PutString("CR:IsFinishedStr", "stop!");

  return true;
}

// Called once after isFinished returns true
void CommandRamp::End() {
    Robot::ramp->RampUnDeploy();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CommandRamp::Interrupted() {}
