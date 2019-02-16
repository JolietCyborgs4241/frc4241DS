#include "OI.h"
#include "commands/LiftUp.h"
#include "commands/LiftDown.h"
#include "commands/CommandRamp.h"
#include "commands/RampUndeploy.h"
#include "commands/LiftStop.h"
#include "Robot.h"
using namespace frc;
OI::OI() {
    // Process operator interface input here.
    xBoxControl = new Joystick(0);

    // Xbox
    ControlA = new JoystickButton(xBoxControl, 1);
    ControlB = new JoystickButton(xBoxControl, 2);
    Select = new JoystickButton(xBoxControl, 7);

    ControlA->WhileHeld(new LiftUp());
    ControlB->WhileHeld(new LiftDown());
    Select->WhileHeld(new CommandRamp());
    
}    

Joystick* OI::getControlJoystick() {
    return xBoxControl;
}

