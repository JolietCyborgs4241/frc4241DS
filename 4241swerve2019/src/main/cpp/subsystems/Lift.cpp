/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Lift.h"
#include "RobotMap.h"

Lift::Lift() : Subsystem("ExampleSubsystem") {
  LiftWinch = RobotMap::lift;
}

void Lift::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void Lift::LiftUp() {
  LiftWinch->Set(.5);
}

void Lift::LiftDown() {
<<<<<<< HEAD
  LiftWinch->Set(8);
  LiftWinch->Set(1.0);
}

void Lift::LiftDown() {
  LiftWinch->Set(-1.0);
=======
  LiftWinch->Set(-.5);
>>>>>>> parent of 46c576a... Undo this
}

void Lift::LiftStop() {
  LiftWinch->Set(0.0);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
