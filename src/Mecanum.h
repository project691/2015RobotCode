//Mecanum.h

#ifndef MECANUM_H
#define MECANUM_H

//Includes
#include <PIDVelocityMotor.h>
#include "WPILib.h"

//Declaration
class Mecanum {
public:
	//Constructor
	Mecanum(PIDVelocityMotor &_fr, PIDVelocityMotor &_fl, PIDVelocityMotor &_br, PIDVelocityMotor &_bl);

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
	PIDVelocityMotor &fr;
	PIDVelocityMotor &fl;
	PIDVelocityMotor &br;
	PIDVelocityMotor &bl;
	//Mecanum control
	double frVel;
	double flVel;
	double brVel;
	double blVel;
	double topVel;
};

#endif //MECCANUM_H
