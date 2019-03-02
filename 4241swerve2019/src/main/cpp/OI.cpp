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
    ControlX->WhileHeld(new ClawOpen());
    ControlY->WhileHeld(new ClawClose());
    ControlLB->WhileHeld(new ClawExtend());
    ControlRB->WhileHeld(new ClawRetract());
    ControlB->WhileHeld(new LiftUp());
    ControlA->WhileHeld(new LiftDown());
    Select->WhileHeld(new CommandRamp());
    
using namespace frc;
OI::OI() {
    // Process operator interface input here.
    xBoxControl = new Joystick(0);
    xBoxDrive = new Joystick(1);

    // Xbox
    //ControlA = new JoystickButton(xBoxControl, 1);
    //ControlB = new JoystickButton(xBoxControl, 2);
    
    
    
    // Xbox controller set up
    

    




    /*DriveA = new JoystickButton(xBoxDrive, 1);
    DriveB = new JoystickButton(xBoxDrive, 2);
    DriveX = new JoystickButton(xBoxDrive, 3);
    DriveY = new JoystickButton(xBoxDrive, 4);
    DriveLB = new JoystickButton(xBoxDrive, 5);
    DriveRB = new JoystickButton(xBoxDrive, 6);
    DriveBack = new JoystickButton(xBoxDrive, 7);
    DriveStart = new JoystickButton(xBoxDrive, 8);
    DriveLeftStick = new JoystickButton(xBoxControl, 9);
    DriveRightStick = new JoystickButton(xBoxControl, 10);*/
    //^Xbox^

    /*ControlA = new JoystickButton(xBoxControl, 1);
    ControlB = new JoystickButton(xBoxControl, 2);
    ControlX = new JoystickButton(xBoxControl, 3);
    ControlY = new JoystickButton(xBoxControl, 4);
    ControlLB = new JoystickButton(xBoxControl, 5);
    ControlRB = new JoystickButton(xBoxControl, 6);
    ControlBack = new JoystickButton(xBoxControl, 7);
    ControlStart = new JoystickButton(xBoxControl, 8);
   
    // Used For controlling main subsystems excluding drive

    // XboxDrive
    /*DriveA->WhenPressed(new ResetPigeonYaw(90));
    DriveB->WhenPressed(new ToggleFieldCentric);
    DriveX->WhenPressed(new TogglePrecisionDrive); */

    /* Eh, no gyro assist today
    DriveY->WhenPressed(new ToggleTwistPID);
    if (xBoxDrive->GetPOV() != -1) {
        Robot::gyroAssist = true;
        Robot::gyroAssistPID->SetSetpoint(xBoxDrive->GetPOV());
    }
    */

    /* No elevator pos control
    ControlA->WhenPressed(new ElevatorPosControl(0));
    ControlLB->WhenPressed(new ElevatorPosControl(3));
    ControlRB->WhenPressed(new ElevatorPosControl(4));

    ControlY->WhenPressed(new ToggleElevatorPosControl);

    ControlBack->WhenPressed(new SetElevatorPosition);
    */

   /* ControlStart->WhenPressed(new ToggleLimitSwitch);

    ControlX->ToggleWhenPressed(new ClawControl);
    ControlB->ToggleWhenPressed(new PusherPiston);
    ControlA->WhenPressed(new EjectCube); */
}

Joystick* OI::getControlJoystick() {
    return xBoxControl;
}

double OI::getControlLY()  {
    return adjustJoystick(xBoxControl->GetRawAxis(1));
    
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

 double OI::adjustJoystick(double value) {
    // cube output
    double adjV = pow(value, 3);

    // add deadzone
    if (fabs(adjV) < 0.05) {
        return 0;
    } else {
        return adjV;
    }
 }
       


