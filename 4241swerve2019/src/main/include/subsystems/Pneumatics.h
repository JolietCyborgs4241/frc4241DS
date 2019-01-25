/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
using namespace frc;

class Pneumatics : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  static constexpr double MAX_PRESSURE = 2.55;

  Compressor* compressor;
  Solenoid* ClawPiston1;
  Solenoid* ClawPiston2;

 public:
  Pneumatics();
  void InitDefaultCommand() override;
  void Start();
  void ClawOpen();
  void ClawClose();
};
