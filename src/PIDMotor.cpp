//PIDMotor.cpp

//Includes
#include <cmath>
#include "PIDMotor.h"

//Constructor
PIDMotor::PIDMotor(const char *_name,
				   Talon &_motor,
				   Encoder &_encoder,
				   const double *_pid,
				   bool _velocity /* = true */) : name(_name),
												  motor(_motor),
												  encoder(_encoder),
												  velocity(_velocity),
												  target(0.0),
												  error(0.0),
												  deltaTime(0.0),
												  lastInputs{0.0},
												  kFIR(NULL),
												  firLen(0),
												  kp(_pid[0]),
												  ki(_pid[1]),
												  kd(_pid[2]),
												  kf(_pid[3]),
												  scalar(_pid[4]),
												  proportional(0.0),
												  integral(0.0),
												  derivative(0.0),
												  feedForward(0.0),
												  pout(0.0),
												  iout(0.0),
												  dout(0.0),
												  fout(0.0),
												  out(0.0),
												  lastError(0.0),
												  lastTime(0)
{
	printf("PIDMotor %s created!\n", name);
}

//Destructor
PIDMotor::~PIDMotor() {}

//Functions

//PIDMotor update
void PIDMotor::run() {
	if(GetTime() - 0.01 > lastTime) {
		if(velocity) {
			error = target - ((encoder.GetRate() / 360 * 60) / scalar);
			/*if(target == 0.0) {
				error = 0.0;
				integral = 0.0;
				lastError = 0.0;
			}*/
		} else {
			error = target - encoder.GetDistance();
			/*if(target == encoder.GetDistance()) {
				error = 0.0;
				integral = 0.0;
				lastError = 0.0;
			}*/
		}
		deltaTime = GetTime() - lastTime;

		proportional = error;
		integral += error * deltaTime;
		derivative = (error - lastError) / deltaTime;
		feedForward = target;

		pout = kp * proportional;
		iout = ki * integral;
		dout = kd * derivative;
		fout = kf * feedForward;
		if(pout > 1.0) {
			pout = 1.0;
		} else if(pout < -1.0) {
			pout = -1.0;
		}
		if(iout > 1.0) {
			iout = 1.0;
			integral = iout / ki;
		} else if(iout < -1.0) {
			iout = -1.0;
			integral = iout / ki;
		}
		if(dout > 1.0) {
			dout = 1.0;
		} else if(dout < -1.0) {
			dout = -1.0;
		}

		out = pout + iout + dout + fout;
		if(out > 1.0) {
			out = 1.0;
		} else if(out < -1.0) {
			out = -1.0;
		}
		motor.SetSpeed(out);

		if(velocity) {
			printf("Name: %s KP: %f Target: %f CurrentRPM: %f, ScaledRPM: %f Error: %f Get(): %d Out: %f\n", name, kp, target, (encoder.GetRate() / 360 * 60), ((encoder.GetRate() / 360 * 60) / scalar), error, encoder.Get(), out);
		} else {
			printf("Name: %s KP: %f Target: %f CurrentPos: %f Error: %f Get(): %d Out: %f\n", name, kp, target, encoder.GetDistance(), error, encoder.Get(), out);
		}

		lastError = error;
		lastTime += deltaTime;
	} else {
		//printf("PIDMotor %s waiting... Time: %f\n", name, GetTime());
	}
}

//PIDMotor control
void PIDMotor::run(double _target) {
	if(firLen == 0) {
		target = _target;
	} else {
		target = 0;

		for(int i = 99; i > 0; i--) {
			lastInputs[i] = lastInputs[i - 1];
		}
		lastInputs[0] = _target;

		for(int i = 0; i < firLen; i++) {
			target += lastInputs[i] * kFIR[i];
		}

		target /= firLen;
	}
	run();
}

bool PIDMotor::atTarget() {
	if(velocity) {
		return abs(error - target) <= 0.1;
	} else {
		return abs(error - target) <= 10;
	}
}

const char * PIDMotor::getName() {
	return name;
}

void PIDMotor::setType(bool _velocity) {
	velocity = _velocity;
}

void PIDMotor::setFIR(double *_kFIR) {
	firLen = sizeof(_kFIR) / sizeof(double);
	if(firLen > 99) {
		firLen = 99;
	}
	kFIR = _kFIR;
}
