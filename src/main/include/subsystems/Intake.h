/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include "ctre/Phoenix.h"
using namespace frc;
class Intake : public frc::Subsystem {
 private:
 WPI_TalonSRX* intakeMotor1;
 WPI_TalonSRX* intakeMotor2;
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
  Intake();
  void InitDefaultCommand() override;
  void Open();
};
