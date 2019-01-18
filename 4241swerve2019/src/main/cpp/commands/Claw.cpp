/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Claw.h"
#include "Robot.h"
#include "Intake.h"

Claw::Claw() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::open());
}

// Called just before this Command runs the first time
void Claw::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Claw::Execute() {
Robot::Intake->open();
}

// Make this return true when this Command no longer needs to run execute()
bool Claw::IsFinished() { return false; }

// Called once after isFinished returns true
void Claw::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void Claw::Interrupted() {
  
}
