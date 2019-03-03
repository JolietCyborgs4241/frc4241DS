#include "subsystems/Pigeon.h"
#include "RobotMap.h"

#include "ctre/Phoenix.h"
#include "ctre/phoenix/Sensors/PigeonIMU.h"

#include "commands/UpdatePigeon.h"

Pigeon::Pigeon() : Subsystem("Pigeon") {
    // REAL BOT TALON
    pigeon = new PigeonIMU(RobotMap::driveTrainFrontRightDrive);

    // PRACTICE BOT TALON
    // pigeon = new PigeonIMU(RobotMap::driveTrainRearLeftSteer);
}

void Pigeon::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    SetDefaultCommand(new UpdatePigeon());
}

void Pigeon::CalibratePigeon() {
    pigeon->EnterCalibrationMode(ctre::phoenix::sensors::PigeonIMU::Accelerometer, 0);
}

void Pigeon::Update() {
    double data[3] = {};
    pigeon->GetYawPitchRoll(data);

    yaw = data[0];
    pitch = data[1];
    roll = data[2];
}

double Pigeon::GetYaw() {
	int turnratio = yaw/360;
	double angle_error = (yaw - (360 * turnratio));
	return  angle_error;
}

bool Pigeon::WasCollision() {
    int16_t data[3] = {};
    pigeon->GetBiasedAccelerometer(data);

    // Convert from Q2.14 to floats
    double current_accel_X = data[0] * pow(2, -14);
    double current_accel_Y = data[1] * pow(2, -14);

    double jerk_X = current_accel_X - previous_accel_X;
    double jerk_Y = current_accel_Y - previous_accel_Y;

    // save the current accel values for the next update
    previous_accel_X = current_accel_X;
    previous_accel_Y = current_accel_Y;

	if (fabs(jerk_X > COLLISION_THRESHOLD_DELTA_G) ||
		fabs(jerk_Y > COLLISION_THRESHOLD_DELTA_G) ) {
		return true;
	}

	return false;
}

bool Pigeon::AmTilted() {
	if (fabs(pitch) > TILTED_THRESHOLD_DEGREES ||
		fabs(roll) > TILTED_THRESHOLD_DEGREES) {
		return true;
	}

	return false;
}

void Pigeon::ResetYaw(double angle) {
    pigeon->SetYaw((angle - 90) * 64.0, 0);
}

void Pigeon::SaveTilt() {
    originalPitch = pitch;
    originalRoll = roll;
}
