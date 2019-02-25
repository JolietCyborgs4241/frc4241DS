#include "RobotMap.h"

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

#include "frc/liveWindow/LiveWindow.h"

using namespace frc;


WPI_TalonSRX* RobotMap::robotArmFulcrum = NULL;
WPI_TalonSRX* RobotMap::robotArmExtension = NULL;
WPI_TalonSRX* RobotMap::robotArmClaw = NULL;

WPI_TalonSRX* RobotMap::lift = NULL;
WPI_TalonSRX* RobotMap::ramp = NULL;
DigitalInput* RobotMap::limitswitchopen = NULL;
DigitalInput* RobotMap::limitswitchclose = NULL;

/*WPI_TalonSRX* RobotMap::elevatorMotor = NULL;
DigitalInput* RobotMap::elevatorUpperLimitSwitch = NULL;
DigitalInput* RobotMap::elevatorBottomLimitSwitch = NULL; */

void RobotMap::init() {
    LiveWindow* lw = LiveWindow::GetInstance();

    robotArmFulcrum = new WPI_TalonSRX(4);
    robotArmExtension = new WPI_TalonSRX(11);
    robotArmClaw = new WPI_TalonSRX(12);
    
    lift = new WPI_TalonSRX(10);
    ramp = new WPI_TalonSRX(3);
    
}
