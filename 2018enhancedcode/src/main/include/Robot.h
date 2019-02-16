#include "frc/WPILib.h"
#include "OI.h"
#include "frc/liveWindow/LiveWindow.h"
#include "RobotMap.h"
#include "subsystems/Lift.h"
#include "subsystems/Ramp.h"
using namespace frc;

class Robot : public IterativeRobot {
  public:
    // Command *autonomousCommand;
    static OI* oi;
    static Lift* lift;
    static Ramp* ramp;
   

    //static bool fieldCentric;
    //static std::string gameData;
    //static bool recievedGameData;

    

    LiveWindow* lw; 
    virtual void RobotInit();
    virtual void TeleopInit();
    virtual void TeleopPeriodic();
    //virtual void TestPeriodic();
    virtual void DisabledInit();
    //virtual void DisabledPeriodic();

  private:
    void Dashboard();

    double FLOffset;
    double FROffset;
    double RLOffset;
    double RROffset;

    float cycleTime;

   /* std::unique_ptr<frc::Command> autonomousCommand;
    frc::SendableChooser<int> chooser;

    
      void LEDSet(int led);
      bool prevTrigger;
      Preferences* Prefs;
      */
};