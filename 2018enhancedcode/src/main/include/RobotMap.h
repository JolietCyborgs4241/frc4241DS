#pragma once
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
using namespace frc;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
class RobotMap {
  public:
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    static  WPI_TalonSRX* robotArmFulcrum;
    static WPI_TalonSRX* robotArmExtension;
    static WPI_TalonSRX* robotArmClaw;
   
    // static CameraServer* Cam;

    static void init();
};