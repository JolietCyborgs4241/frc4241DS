#include "subsystems/DriveTrain.h"
#include "Math.h"
#include "Robot.h"
#include "Robotmap.h"
#include "ctre/Phoenix.h"
#include "WPILib.h"
#include "SpeedControllerGroup.h"
using namespace frc;

DriveTrain::DriveTrain() : Subsystem("DriveTrain") {

    frontLeftDrive = RobotMap::driveTrainFrontLeftDrive;
    frontRightDrive = RobotMap::driveTrainFrontRightDrive;
    rearLeftDrive = RobotMap::driveTrainRearLeftDrive;
    rearRightDrive = RobotMap::driveTrainRearRightDrive;

    frontLeftPos = RobotMap::driveTrainFrontLeftPos;
    frontLeftSteer = RobotMap::driveTrainFrontLeftSteer;
    frontLeft = RobotMap::driveTrainFrontLeft;

    frontRightPos = RobotMap::driveTrainFrontRightPos;
    frontRightSteer = RobotMap::driveTrainFrontRightSteer;
    frontRight = RobotMap::driveTrainFrontRight;

    rearLeftPos = RobotMap::driveTrainRearLeftPos;
    rearLeftSteer = RobotMap::driveTrainRearLeftSteer;
    rearLeft = RobotMap::driveTrainRearLeft;

    rearRightPos = RobotMap::driveTrainRearRightPos;
    rearRightSteer = RobotMap::driveTrainRearRightSteer;
    rearRight = RobotMap::driveTrainRearRight;

    FLInv = 1; 
    FRInv = 1;
    RRInv = 1;
    RLInv = 1;
}

void DriveTrain::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
}

void DriveTrain::SetWheelbase(float w, float x, float y) {
    TrackRear  = w;
    Wheelbase  = x;
    TrackFront = y;
}

void DriveTrain::SetOffsets(double FLOff, double FROff, double RLOff, double RROff) {
    FLOffset = FLOff;
    FROffset = FROff;
    RLOffset = RLOff;
    RROffset = RROff;
}

void DriveTrain::ToggleFrontBack() {
    driveFront = !driveFront;
}

void DriveTrain::Crab(float y, float x, float twist, bool useGyro) {
    // float forward = y * driveAdjust;
    // float strafe = x * driveAdjust; //doesn't match github
    float forward = y; 
    float strafe = x;
 
    char buffer[1000];
    sprintf(buffer, "Crab(y=%f, x=%f, z=%f, useGyro=%d)", y, x, twist, useGyro);
    SmartDashboard::PutString("Crab", buffer);

    if (useGyro) {
        double robotangle = Robot::pigeon->GetYaw() * M_PI / 180;
        forward = +y * sin(robotangle) + x * cos(robotangle);
        strafe = -y * cos(robotangle) + x * sin(robotangle);
    } 

    radius = sqrt(pow(TrackFront, 2) + pow(Wheelbase, 2));

    AP = strafe + twist * Wheelbase / radius;
    BP = strafe - twist * Wheelbase / radius;
    CP = forward + twist * TrackFront / radius;
    DP = forward - twist * TrackFront / radius;

    sprintf(buffer, "radius=%f, AP=%f, BP=%f, CP=%f, DP=%f", radius, AP, BP, CP, DP);
    SmartDashboard::PutString("Crab2", buffer);

    float FLSetPoint = 0;
    float FRSetPoint = 0;
    float RLSetPoint = 0;
    float RRSetPoint = 0;

    // ENCODER PROCESSING BELOW!!
    //
    // REMOVE THIS COMMENT WHEN IT'S WORKED OUT
    if (DP != 0 || BP != 0)
        FLSetPoint = (2.5 - 2.5 / pi * atan2(BP, DP));
    if (BP != 0 || CP != 0)
        FRSetPoint = (2.5 - 2.5 / pi * atan2(BP, CP));
    if (AP != 0 || DP != 0)
        RLSetPoint = (2.5 - 2.5 / pi * atan2(AP, DP));
    if (AP != 0 || CP != 0)
        RRSetPoint = (2.5 - 2.5 / pi * atan2(AP, CP));

    sprintf(buffer, "prior to SetSteerSetPoint FLSP=%f FRSP=%f RLSP=%f RRSP=%f", FLSetPoint, FRSetPoint, RLSetPoint, RRSetPoint);
    SmartDashboard::PutString("Crab2", buffer);

    SetSteerSetpoint(FLSetPoint, FRSetPoint, RLSetPoint, RRSetPoint);
    FLDistToCOR = sqrt(pow(BP, 2) + pow(DP, 2));
    FRDistToCOR = sqrt(pow(BP, 2) + pow(CP, 2));
    RLDistToCOR = sqrt(pow(AP, 2) + pow(DP, 2));
    RRDistToCOR = sqrt(pow(AP, 2) + pow(CP, 2));

    sprintf(buffer, "b4 speedarry FL=%f FR=%f RL=%f RR=%f", FL, FR, RL, RR);
    SmartDashboard::PutString("Crab3", buffer);

    // Solve for fastest wheel speed
    double speedarray[] = {fabs(FL), fabs(FR), fabs(RL), fabs(RR)};

    int length = 4;
    double maxspeed = speedarray[0];
    for (int i = 1; i < length; i++) {
        if (speedarray[i] > maxspeed) {
            maxspeed = speedarray[i];
        }
    }

    // Set ratios based on maximum wheel speed
    if (maxspeed > 1 || maxspeed < -1) {
        FLRatio = FLDistToCOR / maxspeed;
        FRRatio = FRDistToCOR / maxspeed;
        RLRatio = RLDistToCOR / maxspeed;
        RRRatio = RRDistToCOR / maxspeed;
    } else {
        FLRatio = FLDistToCOR;
        FRRatio = FRDistToCOR;
        RLRatio = RLDistToCOR;
        RRRatio = RRDistToCOR;
    }

    // Set drive speeds
    SetDriveSpeed(FLRatio, FRRatio, RLRatio, RRRatio);
}

//void DriveTrain::SwerveArcade(float y, float x, float twist, bool useGyro)
void DriveTrain::SwerveArcade(float y, float x, float twist) {
    float forward = y * driveAdjust;
    float strafe = x * driveAdjust;

    /*if (useGyro) {
        double robotangle = Robot::pigeon->GetYaw() * M_PI / 180;
        forward = +y * sin(robotangle) + x * cos(robotangle);
        strafe = -y * cos(robotangle) + x * sin(robotangle);
    } */

    AP = strafe;
    BP = strafe;
    CP = forward;
    DP = forward;

    float FLSetPoint = 0;
    float FRSetPoint = 0;
    float RLSetPoint = 0;
    float RRSetPoint = 0;

    if (DP != 0 || BP != 0) {
        FLSetPoint = (2.5 - 2.5 / pi * atan2(BP, DP));
    }

    if (BP != 0 || CP != 0) {
        FRSetPoint = (2.5 - 2.5 / pi * atan2(BP, CP));
    }

    if (AP != 0 || DP != 0) {
        RLSetPoint = (2.5 - 2.5 / pi * atan2(AP, DP));
    }

    if (AP != 0 || CP != 0) {
        RRSetPoint = (2.5 - 2.5 / pi * atan2(AP, CP));
    }

    SetSteerSetpoint(FLSetPoint, FRSetPoint, RLSetPoint, RRSetPoint);
    FLDistToCOR = sqrt(pow(BP, 2) + pow(DP, 2));
    FRDistToCOR = sqrt(pow(BP, 2) + pow(CP, 2));
    RLDistToCOR = sqrt(pow(AP, 2) + pow(DP, 2));
    RRDistToCOR = sqrt(pow(AP, 2) + pow(CP, 2));

    // add in twist like arcade drive
    FLDistToCOR -= twist * 0.5;
    RLDistToCOR -= twist * 0.5;

    FRDistToCOR += twist * 0.5;
    RRDistToCOR += twist * 0.5;

    // Solve for fastest wheel speed
    double speedarray[] = {fabs(FLDistToCOR), fabs(FRDistToCOR),
                           fabs(RLDistToCOR), fabs(RRDistToCOR)};

#pragma message ("warning: why is length 4?")
    //   ???? where does this 4 come from?
    //   ???? Number of elements in speedarray? If so, use sizeof()

    int length = 4;
    double maxspeed = speedarray[0];

    for (int i = 1; i < length; i++) {
        if (speedarray[i] > maxspeed) {
            maxspeed = speedarray[i];
        }
    }

    // Set ratios based on maximum wheel speed
    if (maxspeed > 1 || maxspeed < -1) {
        FLRatio = FLDistToCOR / maxspeed;
        FRRatio = FRDistToCOR / maxspeed;
        RLRatio = RLDistToCOR / maxspeed;
        RRRatio = RRDistToCOR / maxspeed;
    } else {
        FLRatio = FLDistToCOR;
        FRRatio = FRDistToCOR;
        RLRatio = RLDistToCOR;
        RRRatio = RRDistToCOR;
    }

}

#pragma message ("warning: possible encoder values referenced below")
// ENCODER VALUES BELOW!!
double DriveTrain::CorrectSteerSetpoint(double setpoint) {
    // Used to correct steering setpoints to within the 0 to 5 V scale
    if (setpoint < 0) {
        return setpoint + 5;
    } else if (setpoint > 5) {
        return setpoint - 5;
    } else if (setpoint == 5) {
        return 0;
    } else {
        return setpoint;
    }
}

#pragma message ("warning: possible encoder values referenced below")
// ENCODER VALUES BELOW!!
void DriveTrain::SetSteerSetpoint(float FLSetPoint, float FRSetPoint, float RLSetPoint, float RRSetPoint) {
    frontLeft->SetSetpoint(CorrectSteerSetpoint(FLSetPoint + FLOffset));
    frontRight->SetSetpoint(CorrectSteerSetpoint(FRSetPoint + FROffset));
    rearLeft->SetSetpoint(CorrectSteerSetpoint(RLSetPoint + RLOffset));
    rearRight->SetSetpoint(CorrectSteerSetpoint(RRSetPoint + RROffset));

    if (fabs(FLSetPoint - frontLeftPos->GetAverageVoltage()) < 1.25 ||
        fabs(FLSetPoint - frontLeftPos->GetAverageVoltage()) > 3.75) {
        frontLeft->SetSetpoint(CorrectSteerSetpoint(FLSetPoint));
        FLInv = 1;
    } else {
        frontLeft->SetSetpoint(CorrectSteerSetpoint(FLSetPoint - 2.5));
        FLInv = -1;
    }

    if (fabs(FRSetPoint - frontRightPos->GetAverageVoltage()) < 1.25 ||
        fabs(FRSetPoint - frontRightPos->GetAverageVoltage()) > 3.75) {
        frontRight->SetSetpoint(CorrectSteerSetpoint(FRSetPoint));
        FRInv = 1;
    } else {
        frontRight->SetSetpoint(CorrectSteerSetpoint(FRSetPoint - 2.5));
        FRInv = -1;
    }

    if (fabs(RLSetPoint - rearLeftPos->GetAverageVoltage()) < 1.25 ||
        fabs(RLSetPoint - rearLeftPos->GetAverageVoltage()) > 3.75) {
        rearLeft->SetSetpoint(CorrectSteerSetpoint(RLSetPoint));
        RLInv = 1;
    } else {

        rearLeft->SetSetpoint(CorrectSteerSetpoint(RLSetPoint - 2.5));
        RLInv = -1;
    }

    if (fabs(RRSetPoint - rearRightPos->GetAverageVoltage()) < 1.25 ||
        fabs(RRSetPoint - rearRightPos->GetAverageVoltage()) > 3.75) {
        rearRight->SetSetpoint(CorrectSteerSetpoint(RRSetPoint));
        RRInv = 1;
    } else {
        rearRight->SetSetpoint(CorrectSteerSetpoint(RRSetPoint - 2.5));
        RRInv = -1;
    }
}

void DriveTrain::SetDriveSpeed(float FLSpeed, float FRSpeed, float RLSpeed, float RRSpeed) {
    // applies inversion variables defined in SetSteerSetPoint function
  /*  frontLeftDrive->Set(ControlMode::PercentOutput, FLSpeed * FLInv);
    frontRightDrive->Set(ControlMode::PercentOutput, FRSpeed * FRInv);
    rearLeftDrive->Set(ControlMode::PercentOutput, RLSpeed * RLInv);
    rearRightDrive->Set(ControlMode::PercentOutput, RRSpeed * RRInv); */
}


// Lock
//
// Point the wheels in different directions to help lock the robot in place
//
// Between the wheels being pointed in differnt directions and the resistance
// the motors being locked, the robot will resist moving in any specific
// direction

void DriveTrain::Lock() {
    // splay the wheels out at different angles so no two wheels can roll
    // in the same direction - this will give the robot the most resistance
    // to moving in any direction
    SetSteerSetpoint(angleToAnalogEncoder(145, ANGLE_DEGREES),
                     angleToAnalogEncoder(55,  ANGLE_DEGREES),
                     angleToAnalogEncoder(235, ANGLE_DEGREES),
                     angleToAnalogEncoder(325, ANGLE_DEGREES));

    // set speed to 0 to lock in place
    SetDriveSpeed(0, 0, 0, 0);
}


#pragma message ("warning: possible encoder values referenced below")
void DriveTrain::DriveForward(double speed, double twist) {
    // ENCODER VALUES BELOW!!
    SetSteerSetpoint(2.5, 2.5, 2.5, 2.5);

    double leftSpeed = speed - twist;
    double rightSpeed = speed + twist;

    double maxSpeed = fmax(fabs(leftSpeed), fabs(rightSpeed));

    if (maxSpeed > 1) {
        leftSpeed /= maxSpeed;
        rightSpeed /= maxSpeed;
    }

    SetDriveSpeed(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
}

void DriveTrain::DriveReverse(double speed, double twist) {
    DriveForward(-speed, twist);
}

#pragma message ("warning: possible encoder values referenced below")
void DriveTrain::DriveLeft(double speed, double twist) {
    // // ENCODER VALUES BELOW!!
    // 3.75 is pointing left
    SetSteerSetpoint(3.75, 3.75, 3.75, 3.75);

    double leftSpeed = speed - twist;
    double rightSpeed = speed + twist;

    // Get the max speed and adjust if it's too large
    double maxSpeed = fmax(fabs(leftSpeed), fabs(rightSpeed));

    if (maxSpeed > 1) {
        leftSpeed /= maxSpeed;
        rightSpeed /= maxSpeed;
    }

    // The "right" side are the front two wheels
    SetDriveSpeed(rightSpeed, rightSpeed, leftSpeed, leftSpeed);
}

void DriveTrain::DriveRight(double speed, double twist) {
    DriveLeft(-speed, twist);
}

void DriveTrain::DriveAngle(double speed, double angle) {
#pragma message ("warning: possible encoder values referenced below")
// ENCODER VALUES BELOW!!
    double steer = ((angle + 90) / 360) * 5.0;
    SetSteerSetpoint(steer, steer, steer, steer);
    SetDriveSpeed(speed, speed, speed, speed);
}


// Stop
//
// Set all motors to 0

void DriveTrain::Stop() {
    SetDriveSpeed(0, 0, 0, 0);
}

void DriveTrain::TogglePrecisionDrive() {
    if (!precisionDrive) {
        // turn it on
        driveAdjust = PRECISION_ADJUST;
    } else {
        // turn it off
        driveAdjust = 1.0;
    }

    precisionDrive = !precisionDrive;
}

void DriveTrain::DisablePIDs() {
    RobotMap::driveTrainFrontLeft->Disable();
    RobotMap::driveTrainFrontRight->Disable();
    RobotMap::driveTrainRearLeft->Disable();
    RobotMap::driveTrainRearRight->Disable();
}

void DriveTrain::EnablePIDs() {
    RobotMap::driveTrainFrontLeft->Enable();
    RobotMap::driveTrainFrontRight->Enable();
    RobotMap::driveTrainRearLeft->Enable();
    RobotMap::driveTrainRearRight->Enable();
}


// analogEncoderToAngle
//
// Map an analog sensor value to a specific angle in degrees or radians
// based on the angleType parameter
//
// This way the code always deals with angles and never with raw encoder
// values (in the event the encoder changes)
//
// This function uses absolute encoder values and will need to be adapted to
// otehr encoders if used!

float DriveTrain::analogEncoderToAngle(float analogValue, int angleType) {

    switch (angleType) {

    case ANGLE_DEGREES:    // 360 degrees * analog encoder value
        return (360.0 * analogValue / ENCODER_MAX);
        break;

    case ANGLE_RADIANS:    // 2pi * analog encoder value
        return (2 * pi * analogValue / ENCODER_MAX);
        break;

    }
}


// angleToAnalogEncoder
//
// Map an angle in degrees or radians to a specific analog sensor value
// based on the angleType parameter
//
// This way the code always deals with angles and never with raw encoder
// values (in the event the encoder changes)
//
// This function uses absolute encoder values and will need to be adapted to
// otehr encoders if used!

float DriveTrain::angleToAnalogEncoder(float angle, int angleType) {

    if (angle == 0.0) {
        return (0);
    }

    switch (angleType) {

    case ANGLE_DEGREES:
        return (angle / 360.0 * ENCODER_MAX);
        break;

    case ANGLE_RADIANS:
        return (angle / (2 * pi) * ENCODER_MAX);
        break;
    }
}
