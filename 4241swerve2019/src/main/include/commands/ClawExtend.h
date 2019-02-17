#pragma once

#include <frc/commands/Command.h>

class ClawExtend : public frc::Command {
 public:
  ClawExtend();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};