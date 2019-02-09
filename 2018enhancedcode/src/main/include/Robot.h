#include "frc/WPILib.h"
#include "OI.h"
#include "subsystems/DriveTrain.h"
#include "frc/liveWindow/LiveWindow.h"
#include "RobotMap.h"
#include "subsystems/Pigeon.h"
#include "subsystems/PigeonPID.h"
using namespace frc;

class Robot : public IterativeRobot {
  public:
    // Command *autonomousCommand;
    static OI* oi;
    static DriveTrain* driveTrain;
    static Pigeon* pigeon;
    //static double twistPID_Value;
	  //static bool twistPID_Enabled;
    /*static Elevator* elevator;
    static Pneumatics* pneumatics;
    static Elevator2* elevator2;
    */
    static bool gyroAssist; 
    //static PigeonPID* gyroAssistPID;

    static bool fieldCentric;
    /*static bool elevatorPositionControl;
    static bool useUpperLimitSwitch;

    static MB1013Sensor* mb1013Sensor;

    static LIDARLite* leftLidarLite;
    static LIDARLite* rightLidarLite;

    static std::string gameData;
    static bool recievedGameData;

    static Timer* autoTimer;
*/
    LiveWindow* lw;
    virtual void RobotInit();
    virtual void AutonomousInit();
    virtual void AutonomousPeriodic();
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