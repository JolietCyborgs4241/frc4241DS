/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include "ctre/Phoenix.h"
#include "frc/WPILib.h"
using namespace frc;
class RobotArm : public frc::Subsystem {
 private:
 WPI_TalonSRX* m_fulcrum;
 WPI_TalonSRX* m_extension;
 WPI_TalonSRX* m_claw;
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
  RobotArm();
  void InitDefaultCommand() override;
  void openClaw();
  void closeClaw();
  void StopClaw();
  void StopExtension();
  void extendClaw();
  void retractClaw();
  void Fulcrum();

};
