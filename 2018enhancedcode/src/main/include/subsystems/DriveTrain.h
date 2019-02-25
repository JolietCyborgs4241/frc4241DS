pragma once
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "Math.h"
using namespace frc;

// Code copied and modified from FRC 16's Macy's 2014 DriveTrain
// https://github.com/FRCTeam16/Macys2014/blob/master/Macys2014/Subsystems/DriveTrain.cpp




// constants related to encoder to angle mapping functions

#define ANGLE_DEGREES    0
#define ANGLE_RADIANS    1

    // constants for the 4 directional oridinals
#define ANGLE_FWD_RADS   0
#define ANGLE_RIGHT_RADS (pi / 2.0)
#define ANGLE_LEFT_RADS  (pi * 1.5)
#define ANGLE_REV_RADS   (pi) 

#define ANGLE_FWD_DEGS    0
#define ANGLE_RIGHT_DEGS  90
#define ANGLE_LEFT_DEGS   270
#define ANGLE_REV_DEGS    180




class DriveTrain : public Subsystem {
  private:
    static constexpr double pi = 3.14159;

#pragma message ( "warning: SetSteerSetPoint takes an encoder value - map angles before calling!" )
    //
    // ????? - should it take an angle for each value and an identifier
    // specifying the angle measurement in use instead?  Probably...
    void SetSteerSetpoint(float FLSet, float FRSet, float RLSet, float RRSet);

    void SetDriveSpeed(float FLSpeed, float FRSpeed, float RLSpeed, float RRSpeed);


    // DriveDirection
    bool driveFront = true;


    // Precision Control
    double PRECISION_ADJUST = 0.35;


    // Steering Variables
    double radian = 0.0;     // input steering angle in radians

    double A = 0.0;          // A is the ratio of X to turn harder
                             // ????? what?

    double TrackRear = 0.0;  // distance from the mid-point of one back wheel
                             // to the mid-point of the other back wheel

    double Wheelbase = 0.0;  // distance from the mid-point of the back wheels
                             // to the mid-point of the front wheels
                             //
                             // ?????? what is the front and rear track are
                             // different?  If this a measurement directly
                             // from the front to rear or perpendicular to
                             // a line through the front and rear wheels?

    double TrackFront = 0.0; // distance from the mid-point of one front wheel
                             // to the mid-point of the other front wheel


    // distance to center of rotation (COR) for each wheel
 
    double FLDistToCOR = 0.0; // Front Left Wheel to the center of rotation
    double FRDistToCOR = 0.0; // Front Right Wheel to the center of rotation
    double RLDistToCOR = 0.0; // Rear Left Wheel to the center of rotation
    double RRDistToCOR = 0.0; // Rear Right Wheel to the center of rotation


    // ????? Ratio compared to what?  The fastest wheel?  Will that be at "1.0"?
 
    double FRRatio = 0.0; // Ratio of Speed of Front Right wheel
    double FLRatio = 0.0; // Ratio of Speed of Front Left wheel
    double RRRatio = 0.0; // Ratio of Speed of Rear Right wheel
    double RLRatio = 0.0; // Ratio of Speed of Rear Left wheel


    // ????? What are these?
 
    double FLOffset = 0.0;
    double FROffset = 0.0;
    double RLOffset = 0.0;
    double RROffset = 0.0;


    // Crab Variables
    double AP = 0.0;
    double BP = 0.0;
    double CP = 0.0;
    double DP = 0.0;


#pragma message ( "warning: figure out what radius is" )
    // ????? Radius of the wheel itself?
    // ????? Distance from the contact point to the pivot point?
 
    double radius = 0.0; // distance from center to each wheel


    // ????? What are these - directions for each wheel to turn in?

    int FLInv = 0;
    int FRInv = 0;
    int RLInv = 0;
    int RRInv = 0;


    double CorrectSteerSetpoint(double setpoint);



    // map analog encoder values to radians or degrees

    float analogEncoderToAngle(float analogValue, int angleType);

    // map radians or degrees to analog encoder values

    float angleToAnalogEncoder(float angle, int angleType);


  public:
    WPI_TalonSRX* frontLeftDrive;
    WPI_TalonSRX* frontRightDrive;
    WPI_TalonSRX* rearLeftDrive;
    WPI_TalonSRX* rearRightDrive;

    AnalogInput* frontLeftPos;
    WPI_TalonSRX* frontLeftSteer;
    PIDController* frontLeft;

    AnalogInput* frontRightPos;
    WPI_TalonSRX* frontRightSteer;
    PIDController* frontRight;

    AnalogInput* rearLeftPos;
    WPI_TalonSRX* rearLeftSteer;
    PIDController* rearLeft;

    AnalogInput* rearRightPos;
    WPI_TalonSRX* rearRightSteer;
    PIDController* rearRight;

    DriveTrain();

    void InitDefaultCommand();

    void TwistRight();
    void TwistLeft();

    void SetWheelbase(float w, float x, float y);

    void SetOffsets(double FLOff, double FROff, double RLOff, double RROff);

    //void Crab(float y, float x, float twist, bool useGyro);
    void Crab(float y, float x, float twist);

    //void SwerveArcade(float y, float x, float twist, bool useGyro);
    void SwerveArcade(float y, float x, float twist);

    // point each wheel in a different direction
    void Lock();

    void Test();

    void ToggleFrontBack();

    void DriveForward(double speed, double twist);
    void DriveReverse(double speed, double twist);
    void DriveLeft(double speed, double twist);
    void DriveRight(double speed, double twist);
    void DriveAngle(double speed, double angle);

    void Stop();

    void TogglePrecisionDrive();

    void DisablePIDs();
    void EnablePIDs();

    double driveAdjust = 1.0;

    bool precisionDrive = false;
};
