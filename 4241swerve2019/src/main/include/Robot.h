#include "frc/WPILib.h"
#include "OI.h"
#include "frc/liveWindow/LiveWindow.h"
#include "RobotMap.h"
#include "subsystems/RobotArm.h"
#include "commands/CommandFulcrum.h"

using namespace frc;

class Robot : public IterativeRobot {
  public:
    // Command *autonomousCommand;
    static OI* oi;
    static RobotArm* robotArm;
  
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

  private:
    void Dashboard();

    double FLOffset;
    double FROffset;
    double RLOffset;
    double RROffset;

    float cycleTime;

    std::unique_ptr<frc::Command> autonomousCommand;
    frc::SendableChooser<int> chooser;

    
      void LEDSet(int led);
      bool prevTrigger;
      Preferences* Prefs;
      int turnDegree;
      int turnDirection;
      float driveForwardAngle;
      
};
