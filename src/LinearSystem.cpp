
#include <Eigen/Core>
#include "LinearSystem.hpp"


LinearSystem::LinearSystem(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf X0, float dt_) : System(X0, dt_), F(f), B(b), H(h){
  //Check Matrix dimensions
  if( F.rows() != F.cols() || F.rows() != B.rows() || F.rows() != H.cols() || F.rows() != X.size() )
    throw "[Linear System] Matrix dimension mismatch.";
   
  D = MatrixXf::Zero(H.rows(), B.cols());
}

LinearSystem::LinearSystem(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf X0, float dt_) : System(X0, dt_), F(f), B(b), H(h), D(d){

  //Check Matrix dimensions
  if( F.rows() != F.cols() || F.rows() != B.rows() || F.rows() != H.cols() ||
  	  F.rows() != X.size() || D.rows() != H.rows() || D.cols() != B.cols() )
    throw "[Linear System] Matrix dimension mismatch.";
  
}

LinearSystem::~LinearSystem(){
}

VectorXf LinearSystem::predict(VectorXf U){
  if( U.size() != B.cols() )
    throw "[Linear System::update] Invalid U dimensions!";
    
  X = F * X + B * U;
  return X;
}

VectorXf LinearSystem::measure(VectorXf U){
	if( U.size() != D.cols() )
    	throw "[Linear System::measure] Invalid D dimensions!";
    	
    return (H * X + D * U);
}

VectorXf LinearSystem::measure(void){
    return (H * X);
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

void LinearSystem::setD(MatrixXf d){
	if( D.rows() != d.rows() || D.cols() != d.cols() )
    	throw "[Linear System::setD] Invalid D dimensions!";
	D = d;
}


