#pragma once

#include <frc/commands/Command.h>

class ClawRetract : public frc::Command {
 public:
  ClawRetract();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};