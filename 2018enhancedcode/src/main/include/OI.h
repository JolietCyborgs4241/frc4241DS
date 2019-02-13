#pragma once

#include "frc/WPILib.h"
#include "Math.h"
using namespace frc;
class OI {
  private:
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    Joystick* xBoxControl;
    Joystick* xBoxDrive;
    JoystickButton* ControlA;
    JoystickButton* ControlB;
    JoystickButton* Select;
    
    
    /*JoystickButton* DriveA;
    JoystickButton* DriveB;
    JoystickButton* DriveX;
    JoystickButton* DriveY;
    JoystickButton* DriveLB;
    JoystickButton* DriveRB;
    JoystickButton* DriveBack;
    JoystickButton* DriveStart;
    JoystickButton* ljoy;
    JoystickButton* rjoy;
    JoystickButton* DriveLeftStick;
    JoystickButton* DriveRightStick;
    // Xbox Drive ^^^
    JoystickButton* ControlA;
    JoystickButton* ControlB;
    JoystickButton* ControlX;
    JoystickButton* ControlY;
    JoystickButton* ControlLB;
    JoystickButton* ControlRB;
    JoystickButton* ControlBack;
    JoystickButton* ControlStart;

    JoystickButton* ControlLeftStick;
    JoystickButton* ControlRightStick; */
    // Xbox Control ^^^

    double adjustJoystick(double);

  public:
    OI();

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PROTOTYPES
    Joystick* getControlJoystick();
    Joystick* getDriveJoystick();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PROTOTYPES
    double getJoystickMagnitude();
    double getDriveRightX();
    double getDriveLeftX();
    double getDriveLeftY();
    double getControlJoy();
    bool getLB();
};