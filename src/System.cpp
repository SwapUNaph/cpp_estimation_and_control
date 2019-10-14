#pragma once
#include <Eigen/Core>
using namespace Eigen;

class System{
protected:
	VectorXf X;
	float dt;
public:
	System(VectorXf x0, float dt_): X(x0), dt(dt_){}
	~System(){}
	virtual VectorXf predict(VectorXf u);
	virtual VectorXf measure(void);
	
	void set_dt(float t){
		dt = t;
	};
	
	VectorXf getX(void){
		return X;
	};
	
	void setX(VectorXf x){
		if( X.size() != x.size() )
			throw "[System::setX] Invalid X dimensions!";
		X = x;
	}
};
