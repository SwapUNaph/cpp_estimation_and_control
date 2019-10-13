#include <Eigen/Core>
#include "LinearSystem.h"


LinearSystem::LinearSystem(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf X0, float v_, float w_){
  F = f;
  B = b;
  H = h;
  X = X0;
  v = v_;
  w = w_;
  
  //Check Matrix dimensions
  if( F.rows() != F.cols() || F.rows() != B.rows() || F.rows() != H.cols() || F.rows() != X.size() ){
    throw "[Linear System] Matrix dimension mismatch.";
    return;
  }
}

LinearSystem::~LinearSystem(){}

void LinearSystem::update(VectorXf U){
  
  if( U.size() != B.cols() )
    throw "[Linear System::update] Invalid U dimensions!";
    
  X = F * X + B * U + v * VectorXf::Random(F.rows());

}

VectorXf LinearSystem::measure(void){
    return (H * X + w * VectorXf::Random(H.rows()));
}

VectorXf LinearSystem::getState(void){
	return X;
}
