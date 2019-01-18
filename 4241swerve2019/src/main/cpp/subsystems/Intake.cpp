/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Intake.h"
#include "WPILib.h"
#include "ctre/phoenix.h"

Intake::Intake() : Subsystem("Intake") {
  intakemotor1 = TalonSRX(5);
  intakemotor2 = TalonSRx(7);
}

void Intake::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());

}
 void Intake::open(){
   intakemotor1 -> set(0.2);
   intakemotor2 -> set(0.2);
 }
// Put methods for controlling this subsystem
// here. Call these from Commands.
