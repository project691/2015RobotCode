//Robot.cpp

//Includes
#include "WPILib.h"
#include <cmath>
#include <string>
#include "Mecanum.h"
#include "Values.h"

using namespace std;

//Main Class
class Robot : public SampleRobot {
private:
	Joystick rJoy;
	Joystick lJoy;
	Joystick liftJoy;

	Talon frMotor;
	Talon flMotor;
	Talon brMotor;
	Talon blMotor;
	Encoder frEnc;
	Encoder flEnc;
	Encoder brEnc;
	Encoder blEnc;
	PIDMotor fr;
	PIDMotor fl;
	PIDMotor br;
	PIDMotor bl;

	Mecanum drive;
	double forward;
	double right;
	double clockwise;
	double scalar;
	RobotDrive rawDrive;
	bool useEncoders;

	Talon lift;
	Solenoid claw;
	Talon scythe;

public:
	Robot(): rJoy(RIGHT_JOYSTICK),
			 lJoy(LEFT_JOYSTICK),
			 liftJoy(LIFT_JOYSTICK),
			 frMotor(FR_DRIVE_TALON),
			 flMotor(FL_DRIVE_TALON),
			 brMotor(BR_DRIVE_TALON),
			 blMotor(BL_DRIVE_TALON),
			 frEnc(FR_DRIVE_ENCODER_A, FR_DRIVE_ENCODER_B, FR_DRIVE_ENCODER_REVERSE),
			 flEnc(FL_DRIVE_ENCODER_A, FL_DRIVE_ENCODER_B, FL_DRIVE_ENCODER_REVERSE),
			 brEnc(BR_DRIVE_ENCODER_A, BR_DRIVE_ENCODER_B, BR_DRIVE_ENCODER_REVERSE),
			 blEnc(BL_DRIVE_ENCODER_A, BL_DRIVE_ENCODER_B, BL_DRIVE_ENCODER_REVERSE),
			 fr("FR", frMotor, frEnc, DRIVE_PID),
			 fl("FL", flMotor, flEnc, DRIVE_PID),
			 br("BR", brMotor, brEnc, DRIVE_PID),
			 bl("BL", blMotor, blEnc, DRIVE_PID),
			 drive(fr, fl, br, bl),
			 forward(0.0),
			 right(0.0),
			 clockwise(0.0),
			 scalar(1.0),
			 rawDrive(flMotor, blMotor, frMotor, brMotor),
			 useEncoders(true),
			 lift(LIFT_TALON),
			 claw(CLAW_SOLENOID),
			 scythe(SCYTHE_TALON)
	{
		rawDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		rawDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		rawDrive.SetSafetyEnabled(false);
		frEnc.SetDistancePerPulse(FR_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		flEnc.SetDistancePerPulse(FL_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		brEnc.SetDistancePerPulse(BR_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		blEnc.SetDistancePerPulse(BL_DRIVE_ENCODER_DISTANCE_PER_PULSE);
	}

	/**
	 * Print a message stating that the robot has been initialized.
	 */
	void RobotInit() {
		printf("Robot initialized!\n");
	}

	/**
	 * Print a message stating that the robot is disabled.
	 */
	void Disabled() {
		printf("Robot is disabled!\n");
	}

	/**
	 * Print a message stating that the robot is in autonomous mode.
	 */
	void Autonomous() {
		printf("Autonomous mode enabled!\n");
		while(IsEnabled() && IsAutonomous()) {

		}
	}

	/**
	 * Run PID Mecanum drive and shooter controls.
	 */
	void OperatorControl() {
		printf("Operator control enabled!\n");
		while(IsEnabled() && IsOperatorControl()) {
			if(fabs(rJoy.GetRawAxis(1)) < 0.2) {
				forward = 0.0;
			} else {
				forward = rJoy.GetRawAxis(1);
				forward *= fabs(forward);
				forward *= scalar;
			}
			if(fabs(rJoy.GetRawAxis(0)) < 0.2) {
				right = 0.0;
			} else {
				right = rJoy.GetRawAxis(0);
				right *= fabs(right);
				right *= scalar;
			}
			if(fabs(lJoy.GetRawAxis(0)) < 0.2) {
				clockwise = 0.0;
			} else {
				clockwise = lJoy.GetRawAxis(0);
				//if(fabs(clockwise) <= 0.5) {
				//	clockwise *= 0.5;
				//} else {
					//clockwise *= fabs(clockwise);
					//if(!rJoy.GetRawButton(4) && !lJoy.GetRawButton(4) && !rJoy.GetRawButton(5) && !lJoy.GetRawButton(5)) {
					//	clockwise *= 0.75;
					//}
				//}
				clockwise *= scalar;
			}
			if(rJoy.GetRawButton(1) || lJoy.GetRawButton(1)) {
				forward = 0.0;
				right = 0.0;
				clockwise = 0.0;
			}
			if(rJoy.GetRawButton(2) && lJoy.GetRawButton(2)) {
				useEncoders = true;
			}
			if(rJoy.GetRawButton(3) && lJoy.GetRawButton(3)) {
				useEncoders = false;
			}
			if(useEncoders) {
				drive.update(forward, right, clockwise);
							//Forward  Right  Clockwise
			} else {
				rawDrive.MecanumDrive_Cartesian(right, forward, clockwise);
			}

			if(fabs(liftJoy.GetRawAxis(1)) < 0.2) {
				lift.Set(0.0);
			} else {
				lift.Set(liftJoy.GetRawAxis(1));
			}
			if(liftJoy.GetRawButton(1)) {
				claw.Set(true);
			} else if(liftJoy.GetRawButton(3)) {
				claw.Set(false);
			}
			if(liftJoy.GetRawButton(4)) {
				scythe.Set(1.0);
			} else if(liftJoy.GetRawButton(5)) {
				scythe.Set(-1.0);
			} else {
				scythe.Set(0.0);
			}

			//printf("Forward: %f,\tRight: %f,\tClockwise: %f\n", forward, right, clockwise);
			SmartDashboard::PutString("DB/String 0", "Forward:");
			SmartDashboard::PutString("DB/String 5", to_string(forward));
			SmartDashboard::PutString("DB/String 1", "Right:");
			SmartDashboard::PutString("DB/String 6", to_string(right));
			SmartDashboard::PutString("DB/String 2", "Clockwise:");
			SmartDashboard::PutString("DB/String 7", to_string(clockwise));
			SmartDashboard::PutString("DB/String 3", "Time:");
			SmartDashboard::PutString("DB/String 8", to_string(GetTime()));

			Wait(0.005);
		}
	}

	/**
	 * Print a message stating that the robot is in test mode.
	 */
	void Test() {
		printf("Test mode enabled!\n");
		while(IsTest()) {

		}
	}
};

START_ROBOT_CLASS(Robot);
