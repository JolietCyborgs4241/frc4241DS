/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Intake.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

Intake::Intake() : Subsystem("Intake") {
  intakeMotor1 = new WPI_TalonSRX(5);
  intakeMotor2 = new WPI_TalonSRX(7);
}

void Intake::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());

}
 void Intake::Open(){
   intakeMotor1->Set(0.2);
   intakeMotor2->Set(0.2);
 }
// Put methods for controlling this subsystem
// here. Call these from Commands.
