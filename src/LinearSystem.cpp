
#include <Eigen/Core>
#include "LinearSystem.hpp"


LinearSystem::LinearSystem(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf x0, float dt_) : F(f), B(b), H(h), X(x0), dt(dt_){
  //Check Matrix dimensions
  if( f.rows() != f.cols() || f.rows() != b.rows() || f.rows() != h.cols() || f.rows() != x0.size() )
    throw "[Linear System] Matrix dimension mismatch.";
}


LinearSystem::~LinearSystem(){}

VectorXf LinearSystem::predict(VectorXf U){
  if( U.size() != B.cols() )
    throw "[Linear System::update] Invalid U dimensions!";
    
  X = ( MatrixXf::Identity(F.rows(),F.cols()) + F * dt) * X + (B * U) * dt;
  return X;
}


VectorXf LinearSystem::measure(void){
    return (H * X);
}

void LinearSystem::setX(VectorXf x){
	if( X.size() != x.size() )
    	throw "[Linear System::setX] Invalid X dimensions!";
	X = x;
}
void LinearSystem::set_dt(float dt_){
	dt = abs(dt_);
}

VectorXf LinearSystem::getX(){
	return X;
}

float LinearSystem::get_dt(){
	return dt;
}

void LinearSystem::setF(MatrixXf f){
	if( F.rows() != f.rows() || F.cols() != f.cols() )
    	throw "[Linear System::setF] Invalid F dimensions!";
	F = f;
}

void LinearSystem::setB(MatrixXf b){
	if( B.rows() != b.rows() || B.cols() != b.cols() )
    	throw "[Linear System::setB] Invalid B dimensions!";
	B = b;
}

void LinearSystem::setH(MatrixXf h){
	if( H.rows() != h.rows() || H.cols() != h.cols() )
    	throw "[Linear System::setH] Invalid H dimensions!";
	H = h;
}

