//Values.h

#ifndef VALUES_H
#define VALUES_H

	const int RIGHT_JOYSTICK = 0;
	const int LEFT_JOYSTICK = 1;
	const int LIFT_JOYSTICK = 2;

	const int FR_DRIVE_TALON		= 0;
	const int FL_DRIVE_TALON		= 1;
	const int BR_DRIVE_TALON		= 2;
	const int BL_DRIVE_TALON		= 3;

	const int FR_DRIVE_ENCODER_A	= 0;
	const int FR_DRIVE_ENCODER_B	= 1;
	const double FR_DRIVE_ENCODER_DISTANCE_PER_PULSE = 1.0;
	const bool FR_DRIVE_ENCODER_REVERSE = false;

	const int FL_DRIVE_ENCODER_A	= 2;
	const int FL_DRIVE_ENCODER_B	= 3;
	const double FL_DRIVE_ENCODER_DISTANCE_PER_PULSE = 1.0;
	const bool FL_DRIVE_ENCODER_REVERSE = false;

	const int BR_DRIVE_ENCODER_A	= 4;
	const int BR_DRIVE_ENCODER_B	= 5;
	const double BR_DRIVE_ENCODER_DISTANCE_PER_PULSE = 1.0;
	const bool BR_DRIVE_ENCODER_REVERSE = false;

	const int BL_DRIVE_ENCODER_A	= 6;
	const int BL_DRIVE_ENCODER_B	= 7;
	const double BL_DRIVE_ENCODER_DISTANCE_PER_PULSE = 1.0;
	const bool BL_DRIVE_ENCODER_REVERSE = false;

	const double DRIVE_PID_SCALAR = 1325.0;
	const double DRIVE_PID_KP = 0.4;
	const double DRIVE_PID_KI = 0.0;
	const double DRIVE_PID_KD = 0.0005;
	const double DRIVE_PID_KF = 0.5;
	const double DRIVE_PID[5] = {DRIVE_PID_KP, DRIVE_PID_KI, DRIVE_PID_KD, DRIVE_PID_KF, DRIVE_PID_SCALAR};

	const int LIFT_TALON = 4;
	const int CLAW_SOLENOID = 0;
	const int SCYTHE_TALON = 5;
	const int SCYTHE_LIMIT = 8;

#endif  //VALUES_H
