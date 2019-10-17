
#include <Eigen/Core>
#include "StochasticLinearSystem.hpp"


StochasticLinearSystem::StochasticLinearSystem(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf x0, float dt_, VectorXf v_, VectorXf w_)
		:	LinearSystem(f,b,h,x0,dt_), v(v_), w(w_)
{
	if (v_.size() != x0.size() || w_.size() != h.rows() )
		throw "[Stochastic Linear System] Invalid v or w dimensions.";
}		

StochasticLinearSystem::~StochasticLinearSystem(){}

VectorXf StochasticLinearSystem::predict(VectorXf U){
  if( U.size() != B.cols() )
    throw "[Linear System::update] Invalid U dimensions!";
    
  X = ( MatrixXf::Identity(F.rows(),F.cols()) + F * dt) * X + (B * U) * dt + v.cwiseProduct(VectorXf::Random(X.size()));
  return X;
}

VectorXf StochasticLinearSystem::measure(){
    return H * X + w.cwiseProduct(VectorXf::Random(H.rows()));
}

void StochasticLinearSystem::set_v(VectorXf v_){
	if ( v.size() != v_.size())
		throw "[Stochastic Linear System] Invalid v dimensions.";
	v = v_;
}

void StochasticLinearSystem::set_w(VectorXf w_){
	if ( w.size() != w_.size())
		throw "[Stochastic Linear System] Invalid w dimensions.";
	w = w_;
}
