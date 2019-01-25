/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Pneumatics.h"
#include "Robot.h"
using namespace frc;

Pneumatics::Pneumatics() : Subsystem("Pneumatics") {
  compressor = new Compressor(1);
ClawPiston1 = new Solenoid(1);
ClawPiston2 = new Solenoid(2);
}


void Pneumatics::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void Pneumatics::Start() {
  #ifdef REAL
    compressor->Start();
  #endif
    ClawClose();
}

void Pneumatics::ClawOpen() {
  ClawPiston1->Set(true);
  ClawPiston2->Set(true);
}

void Pneumatics::ClawClose() {
  ClawPiston1->Set(false);
  ClawPiston2->Set(false);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
