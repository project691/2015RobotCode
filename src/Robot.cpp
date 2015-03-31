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
	bool useDriveEncoders;

	Talon liftMotor;
	Encoder liftEnc;
	PIDMotor lift;
	AnalogInput liftUpperLimit;
	AnalogInput liftLowerLimit;
	bool useLiftEncoder;

	Solenoid claw;
	bool clawLatch;
	Talon scythe;

	bool setup;
	bool move;
	double time;

	static SmartDashboard dashboard;

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
			 useDriveEncoders(false),
			 liftMotor(LIFT_TALON),
			 liftEnc(LIFT_ENCODER_A, LIFT_ENCODER_B, LIFT_ENCODER_REVERSE),
			 lift("LIFT", liftMotor, liftEnc, LIFT_PID, false),
			 liftUpperLimit(LIFT_UPPER_LIMIT),
			 liftLowerLimit(LIFT_LOWER_LIMIT),
			 useLiftEncoder(false),
			 claw(CLAW_SOLENOID),
			 clawLatch(false),
			 scythe(SCYTHE_TALON),
			 setup(true),
			 move(false),
			 time(GetTime())
	{
		rawDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		rawDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		rawDrive.SetSafetyEnabled(false);
		frEnc.SetDistancePerPulse(FR_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		flEnc.SetDistancePerPulse(FL_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		brEnc.SetDistancePerPulse(BR_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		blEnc.SetDistancePerPulse(BL_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		liftEnc.SetDistancePerPulse(LIFT_ENCODER_DISTANCE_PER_PULSE);
	}

	/**
	 * Print a message stating that the robot has been initialized.
	 */
	void RobotInit() {
		printf("Robot initialized!\n");
		dashboard.init();
		CameraServer::GetInstance()->SetQuality(50);
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");
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
		setup = true;
		move = false;
		time = GetTime();
		while(IsEnabled() && IsAutonomous()) {
			//Grab box
			if(setup) {
				if(useDriveEncoders) {
					drive.update(-0.5, 0.0, 0.0);
				} else {
					rawDrive.MecanumDrive_Cartesian(0.0, -0.5, 0.0);
				}
				if(GetTime() - time >= 0.75) {
					if(useDriveEncoders) {
						drive.update(0.0, 0.0, 0.0);
					} else {
						rawDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
					}
					Wait(0.5);
					claw.Set(true);
					Wait(2.0);
					setup = false;
					move = true;
					time = GetTime();
				} else {
					Wait(0.005);
				}
			}
			//Drive backward
			if(move) {
				if(useDriveEncoders) {
					drive.update(0.45, 0.0, 0.0);
				} else {
					rawDrive.MecanumDrive_Cartesian(0.0, 0.45, 0.0);
				}
				if(GetTime() - time >= 4.25) {	//Bump: 4.5, Flat: 2.5
					if(useDriveEncoders) {
						drive.update(0.0, 0.0, 0.0);
					} else {
						rawDrive.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
					}
					Wait(0.01);
					move = false;
					time = GetTime();
				} else {
					Wait(0.005);
				}
			}
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
				forward *= fabs(forward) * fabs(forward);
				forward *= scalar;
			}
			if(fabs(rJoy.GetRawAxis(0)) < 0.2) {
				right = 0.0;
			} else {
				right = rJoy.GetRawAxis(0);
				right *= fabs(right) * fabs(right);
				right *= scalar;
			}
			if(fabs(lJoy.GetRawAxis(0)) < 0.2) {
				clockwise = 0.0;
			} else {
				clockwise = lJoy.GetRawAxis(0);
				clockwise *= fabs(clockwise) * fabs(clockwise);
				clockwise *= scalar;
			}
			if(rJoy.GetRawButton(1) || lJoy.GetRawButton(1)) {
				forward = 0.0;
				right = 0.0;
				clockwise = 0.0;
			}
			if(rJoy.GetRawButton(2) && lJoy.GetRawButton(2)) {
				useDriveEncoders = true;
			}
			if(rJoy.GetRawButton(3) && lJoy.GetRawButton(3)) {
				useDriveEncoders = false;
			}
			if(useDriveEncoders) {
				drive.update(forward, right, clockwise);
							//Forward  Right  Clockwise
			} else {
				rawDrive.MecanumDrive_Cartesian(right, forward, clockwise);
			}

			if((!useLiftEncoder && fabs(liftJoy.GetRawAxis(1)) < 0.2) || (liftUpperLimit.GetVoltage() >= 4.5 && liftJoy.GetRawAxis(1) < 0.0) || (liftLowerLimit.GetVoltage() >= 4.5 && liftJoy.GetRawAxis(1) > 0.0)) {
				if(useLiftEncoder) {
					lift.run(0.0);
				} else {
					liftMotor.Set(0.0);
				}
			} else {
				if(useLiftEncoder) {
					if(liftJoy.GetRawButton(3)) {
						lift.run(liftEnc.GetDistance() + 500.0);
					} else if(liftJoy.GetRawButton(2)) {
						lift.run(liftEnc.GetDistance() - 500.0);
					}
					lift.run();
				} else {
					liftMotor.Set(-liftJoy.GetRawAxis(1) * fabs(liftJoy.GetRawAxis(1)) * fabs(liftJoy.GetRawAxis(1)));
				}
			}
			if(liftJoy.GetRawButton(1) && !clawLatch) {
				claw.Set(!claw.Get());
				clawLatch = true;
			} else if(!liftJoy.GetRawButton(1) && clawLatch) {
				clawLatch = false;
			}
			if(liftJoy.GetRawButton(4)) {
				scythe.Set(1.0);
			} else if(liftJoy.GetRawButton(5)) {
				scythe.Set(-1.0);
			} else {
				scythe.Set(0.0);
			}

			//printf("Forward: %f,\tRight: %f,\tClockwise: %f\n", forward, right, clockwise);
			dashboard.PutString("DB/String 0", "Forward:");
			dashboard.PutString("DB/String 5", to_string(forward));
			dashboard.PutString("DB/String 1", "Right:");
			dashboard.PutString("DB/String 6", to_string(right));
			dashboard.PutString("DB/String 2", "Clockwise:");
			dashboard.PutString("DB/String 7", to_string(clockwise));
			dashboard.PutString("DB/String 3", "Time:");
			dashboard.PutString("DB/String 8", to_string(GetTime()));

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
