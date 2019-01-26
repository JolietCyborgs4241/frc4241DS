/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"
#include "commands/ClawPneumatics.h"
#include <frc/WPILib.h>

OI::OI() {
  // Process operator interface input here.
  mechanismjoystick = new Joystick(0);
  a = new JoystickButton(mechanismjoystick, 1);

a->ToggleWhenPressed(new ClawPneumatics);
}


