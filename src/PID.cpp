#include "PID.hpp"

PID::PID(float p, float i, float d, float dt_) : P(p), I(i), D(d), dt(dt_){
	integration = 0;
	derivative = 0;
	error = 0;
}

~PID::PID(){}

float PID::update(float err){
	
	integration += err * dt;
	integration = (integration > abs(integration_limit)) ? abs(integration_limit) : integration;
	integration = (integration < -abs(integration_limit)) ? -abs(integration_limit) : integration;
	
	derivative = (error - err) / dt;
	error = err;
	
	return P * err + I * integration + D * derivative;
}

void PID::reset(){
	integration = 0;
	derivative = 0;
	error = 0;
}

float PID::getP(){
	return P;
}

float PID::getI(){
	return I;
}

float PID::getD(){
	return D;
}

float PID::get_dt(){
	return dt;
}	

void PID::setP(float p){
	P = p;
}
void PID::setI(float i){
	I = i;
}

void PID::setD(float d){
	D = d;
}

void PID::set_dt(float dt_){
	dt = abs(dt_);
}		

void PID::setIntLimit(float intLimit){
	integration_limit = intLimit;
}
