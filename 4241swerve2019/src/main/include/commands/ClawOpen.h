#pragma once

#include <frc/commands/Command.h>

class ClawOpen : public frc::Command {
 public:
  ClawOpen();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};