/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "WPILib.h"
#include <frc/commands/Subsystem.h>
using namespace frc;

class ArmPot : public frc::Subsystem {
 private:
 Potentiometer* armPotent;
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
  ArmPot();
  void InitDefaultCommand() override;
  int ReturnAngleIndex();

};
