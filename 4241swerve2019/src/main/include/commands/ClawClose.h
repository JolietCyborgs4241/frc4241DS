#pragma once

#include <frc/commands/Command.h>

class ClawClose : public frc::Command {
 public:
  ClawClose();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};