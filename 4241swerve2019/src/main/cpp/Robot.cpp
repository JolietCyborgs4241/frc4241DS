#include "Robot.h"
#include "subsystems/Ramp.h"


using namespace frc;

OI* Robot::oi = NULL;
Lift* Robot::lift = NULL;
Ramp* Robot::ramp = NULL;
// bool Robot::fieldCentric = true;
// std::string Robot::gameData = "";
// bool Robot::recievedGameData = false;

void Robot::RobotInit() {
    RobotMap::init();

    oi = new OI();
    lift = new Lift();
    ramp = new Ramp();

    
//     lw = LiveWindow::GetInstance();
}

 void Robot::DisabledInit() {

 }

// void Robot::DisabledPeriodic() {
// }

   
//     gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
//}



 void Robot::TeleopInit() {
//     // This makes sure that the autonomous stops running when
//     // teleop starts running. If you want the autonomous to
//     // continue until interrupted by another command, remove
//     // this line or comment it out.

    Scheduler::GetInstance()->RemoveAll();

     cycleTime = Timer::GetFPGATimestamp();
 }

 void Robot::TeleopPeriodic() {
     SmartDashboard::PutNumber("CycleTime", 100+ Timer::GetFPGATimestamp() - cycleTime);
     cycleTime = Timer::GetFPGATimestamp();
     Dashboard();

     Scheduler::GetInstance()->Run();
 }

// void Robot::TestPeriodic() {
//     Dashboard();
// }

void Robot::Dashboard() {

}

START_ROBOT_CLASS(Robot); 
