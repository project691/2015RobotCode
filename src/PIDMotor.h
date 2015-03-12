//PIDMotor.h

#ifndef PIDMOTOR_H
#define PIDMOTOR_H

//Includes
#include "WPILib.h"

//Declaration
class PIDMotor {
public:
	//Constructor
	PIDMotor(const char *_name,
			 Talon &_motor,
			 Encoder &_encoder,
			 const double *_pid,
			 bool _velocity = true);

	//Destructor
	~PIDMotor();

	//Functions
	void run();
	void run(double _target);
	bool atTarget();
	const char * getName();
	void setType(bool _velocity);
	void setFIR(double *_kFIR);

private:
	//Init data
	const char *name;
	Talon &motor;
	Encoder &encoder;
	bool velocity;
	//PIDMotor input
	double target;
	double error;
	double deltaTime;
	//FIR filter
	double lastInputs[100];
	double *kFIR;
	int firLen;
	//PIDMotor scale
	double kp;
	double ki;
	double kd;
	double kf;
	double scalar;
	//PIDMotor calculations
	double proportional;
	double integral;
	double derivative;
	double feedForward;
	//PIDMotor out
	double pout;
	double iout;
	double dout;
	double fout;
	double out;
	//PIDMotor loop
	double lastError;
	double lastTime;
};

#endif //PIDMOTOR_H
