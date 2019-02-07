// #pragma once

// #include "frc/WPILib.h"
// #include "ctre/Phoenix.h"
// #include <frc/Commands/PIDSubsystem.h>
// #include <math.h>
// #include "ctre/Phoenix.h"
// using namespace frc;
// class PigeonPID : public PIDSubsystem {
//  private:
//     const double kP = 0.04;
//     const double kI = 0.0;
//     const double kD = 0.0;
//     const double kF = 0.0;

//     const double kTolerance = 3;  // degrees
//     const double kMaxSpeed = 0.5; // motor speed

//     double output;

//     std::shared_ptr<PIDController> pid;

//   public:
//     PigeonPID();
//     ~PigeonPID();
//     void InitDefaultCommand();
//     double ReturnPIDInput();
//     void UsePIDOutput(double output);

//     bool IsEnabled();
//     double GetOutput();
//     double GetDegError();
// };
//#endif  // PigeonPID_H
