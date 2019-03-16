#include "OI.h"
#include "commands/ClawClose.h"
#include "commands/ClawOpen.h"
#include "commands/ClawExtend.h"
#include "commands/ClawRetract.h"
#include "commands/LiftUp.h"
#include "commands/LiftDown.h"
#include "commands/CommandRamp.h"
#include "Robot.h"
using namespace frc;
OI::OI() {
    // Process operator interface input here.
    xBoxControl = new Joystick(0);
    xBoxDrive = new Joystick(1);
    // Xbox
    //ControlA = new JoystickButton(xBoxControl, 1);
    //ControlB = new JoystickButton(xBoxControl, 2);
    ControlA = new JoystickButton(xBoxControl, 1);
    ControlB = new JoystickButton(xBoxControl, 2);
    ControlX = new JoystickButton(xBoxControl, 3);
    ControlY = new JoystickButton(xBoxControl, 4);
    ControlLB = new JoystickButton(xBoxControl, 5);
    ControlRB = new JoystickButton(xBoxControl, 6);
    Select = new JoystickButton(xBoxControl, 7);

    // Xbox controller set up
    ControlA->WhileHeld(new ClawOpen());
    ControlB->WhileHeld(new ClawClose());
    ControlLB->WhileHeld(new ClawExtend());
    ControlRB->WhileHeld(new ClawRetract());
    // ControlB->WhileHeld(new LiftUp());
    // ControlA->WhileHeld(new LiftDown());
    Select->WhileHeld(new CommandRamp());
}


Joystick* OI::getControlJoystick() {
    return xBoxControl;
}

double OI::getControlLY()  {
    return adjustJoystick(xBoxControl->GetRawAxis(1));
}

Joystick* OI::getDriveJoystick() {
    return xBoxDrive;
}

bool OI::getLB() {
    return xBoxDrive->GetRawButton(5);
}

double OI::getJoystickMagnitude() {
    if (xBoxDrive->GetMagnitude() < .1) {
        return 0;
    } else {
        if (xBoxDrive->GetY() < 0) {
            return -xBoxDrive->GetMagnitude();
        } else {
            return xBoxDrive->GetMagnitude();
        }
    }
}

double OI::getDriveRightX() {
    return adjustJoystick(xBoxDrive->GetRawAxis(4));
}

double OI::getDriveLeftX() {
    return adjustJoystick(xBoxDrive->GetX());
}

double OI::getDriveLeftY() {
    return adjustJoystick(xBoxDrive->GetY());
}

double OI::getControlJoy() {
    return adjustJoystick(xBoxControl->GetY());
}

double OI::adjustJoystick(double value) {
    // cube output
    double adjV = pow(value, 3);

    // add deadzone
    if (fabs(adjV) < 0.20) {
        return 0;
    } else {
        return adjV;
    }
}


       


