#include "frc/WPILib.h"
#include "OI.h"
#include "frc/liveWindow/LiveWindow.h"
#include "RobotMap.h"
#include "subsystems/RobotArm.h"
#include "subsystems/Lift.h"
#include "subsystems/Ramp.h"
#include <string>
#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SampleRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "ctre/Phoenix.h"
#include "frc/WPILib.h"
using namespace frc;

class Robot : public IterativeRobot {
  public:
    // Command *autonomousCommand;
    static OI* oi;
    static RobotArm* robotArm;
    static Lift* lift;
    static Ramp* ramp;
  
    /*static Pigeon* pigeon;
    static Elevator* elevator;
    static Pneumatics* pneumatics;
    static Elevator2* elevator2;


#include "OI.h"



class Robot : public frc::TimedRobot {
 public:
 
  static OI m_oi;

  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void VisionThread();
  void VisionInit();
 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc::Command* m_autonomousCommand = nullptr;

  frc::SendableChooser<frc::Command*> m_chooser;
};
    static bool gyroAssist;
    static PigeonPID* gyroAssistPID;

    static bool fieldCentric;
    static bool elevatorPositionControl;
    static bool useUpperLimitSwitch;

    static MB1013Sensor* mb1013Sensor;

    static LIDARLite* leftLidarLite;
    static LIDARLite* rightLidarLite;

    static std::string gameData;
    static bool recievedGameData;

    static Timer* autoTimer;

   */LiveWindow* lw; 
    virtual void RobotInit();
    virtual void TeleopInit();
    virtual void TeleopPeriodic();
    virtual void TestPeriodic();
    virtual void DisabledInit();
    virtual void DisabledPeriodic();
    void Autonomous();
    void OperatorControl();
  private:
    void Dashboard();

    double FLOffset;
    double FROffset;
    double RLOffset;
    double RROffset;

    float cycleTime;
  WPI_TalonSRX frontLeft{5};
  WPI_TalonSRX frontRight{9};
  WPI_TalonSRX rearLeft{3};
  WPI_TalonSRX rearRight{7};
  frc::RobotDrive m_robotDrive{frontLeft, rearLeft, frontRight, rearRight};
  frc::Joystick m_stick{1};
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
   /* std::unique_ptr<frc::Command> autonomousCommand;
    frc::SendableChooser<int> chooser;

    
      void LEDSet(int led);
      bool prevTrigger;
      Preferences* Prefs;
      int turnDegree;
      int turnDirection;
      float driveForwardAngle;
      
};
      */
};
