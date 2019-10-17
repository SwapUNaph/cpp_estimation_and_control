#pragma once

class PID{ 
private:
	float P;
	float I;
	float D;
	float dt;
	float error;
	float integration;
	float derivative;
	float integration_limit = 5.0;
public: 
	PID(float p, float i, float d, float dt_);
	float update(float err);
	void reset();
	float getP();
	float getI();
	float getD();
	float get_dt();
	void setP(float p);
	void setI(float i);
	void setD(float d);	
	void set_dt(float dt_);	
	void setIntLimit(float intLimit);	
};
