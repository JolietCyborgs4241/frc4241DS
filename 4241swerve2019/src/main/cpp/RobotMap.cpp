#include "RobotMap.h"

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

#include "frc/liveWindow/LiveWindow.h"

using namespace frc;


WPI_TalonSRX* RobotMap::lift = NULL;
WPI_TalonSRX* RobotMap::ramp = NULL;

/*WPI_TalonSRX* RobotMap::elevatorMotor = NULL;
DigitalInput* RobotMap::elevatorUpperLimitSwitch = NULL;
DigitalInput* RobotMap::elevatorBottomLimitSwitch = NULL; */

void RobotMap::init() {
    LiveWindow* lw = LiveWindow::GetInstance();

    
    lift = new WPI_TalonSRX(4);
    ramp = new WPI_TalonSRX(3);
    
}
