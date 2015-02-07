//Mecanum.h

#ifndef MECANUM_H
#define MECANUM_H

//Includes
#include "WPILib.h"
#include "PIDMotor.h"

//Declaration
class Mecanum {
public:
	//Constructor
	Mecanum(PIDMotor &_fr,
			PIDMotor &_fl,
			PIDMotor &_br,
			PIDMotor &_bl);

	//Destructor
	~Mecanum();

	//Functions
	void moveDual(Joystick rjoy, Joystick ljoy);
	void move(Joystick joy);
	void update(double forward, double right, double clockwise);
	void stop();

private:
	//Init Data
	//PIDMotors
	PIDMotor &fr;
	PIDMotor &fl;
	PIDMotor &br;
	PIDMotor &bl;
	//Mecanum control
	double frVel;
	double flVel;
	double brVel;
	double blVel;
	double topVel;
};

#endif //MECCANUM_H
