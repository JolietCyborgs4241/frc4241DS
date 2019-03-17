/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ArmPot.h"
#include "RobotMap.h"
#include "Robot.h"
#include "math.h"
#include "WPILib.h"
using namespace frc;

ArmPot::ArmPot() : Subsystem("ExampleSubsystem") {
armPotent = RobotMap::armangle;
}



void ArmPot::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

int ArmPot::ReturnAngleIndex() {
    int indexAngle;

    indexAngle = round(armPotent->Get());
    return indexAngle;

}

// Put methods for controlling this subsystem
// here. Call these from Commands.
