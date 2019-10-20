/**
 * StochasticLinearSystem.cpp
 * 
 * Copyright 2019 Swapneel Naphade <naphadeswapneel@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */


#include <Eigen/Core>
#include "StochasticLinearSystem.hpp"


/**
 * @brief Stochastic Linear System class constructor
 * 			Based on stochastic linear system defined by:
 * 					X_dot = F * X + B * U + v
 * 					Z = H * X + w
 * @param f Dynamics Matrix
 * @param b Control Input Matrix 
 * @param h Measurement Matrix
 * @param x0 Initial Estimate
 * @param dt_ Time Step
 * @param v_ Porcess noise standard deviation
 * @param w_ Measurement noise standard deviation
 * @returns nothing
 * 
 * 
 */
StochasticLinearSystem::StochasticLinearSystem(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf x0, float dt_, VectorXf v_, VectorXf w_)
		:	LinearSystem(f,b,h,x0,dt_), v(v_), w(w_)
{
	if (v_.size() != x0.size() || w_.size() != h.rows() )
		throw "[Stochastic Linear System] Invalid v or w dimensions.";
}	

	

/**
 * @brief Stochastic Linear System class destructor
 * @returns nothing
 * 
 * 
 */
StochasticLinearSystem::~StochasticLinearSystem(){}



/**
 * @brief Predict and Update the next state after time step with added noise
 * @param U Input to the system
 * @returns X Updated state
 * 
 * 
 */
VectorXf StochasticLinearSystem::predict(VectorXf U){
  if( U.size() != B.cols() )
    throw "[Linear System::update] Invalid U dimensions!";
  VectorXf X_dot_current = F * X + B * U;
  X = X + (X_dot + X_dot_current) / 2 * dt + v.cwiseProduct(VectorXf::Random(X.size()));
  X_dot = X_dot_current; 
  return X;
}



/**
 * @brief Measure the current state based on current measurement configuration with added noise
 * @returns Measurement output of the system
 * 
 * 
 */
VectorXf StochasticLinearSystem::measure(){
    return H * X + w.cwiseProduct(VectorXf::Random(H.rows()));
}



/**
 * @brief Set the proces noise standard deviation vector (v)
 * @param v_ v
 * 
 * 
 */
void StochasticLinearSystem::set_v(VectorXf v_){
	if ( v.size() != v_.size())
		throw "[Stochastic Linear System] Invalid v dimensions.";
	v = v_;
}



/**
 * @brief Set the measurement noise standard deviation vector (w)
 * @param w_ w
 * 
 * 
 */
void StochasticLinearSystem::set_w(VectorXf w_){
	if ( w.size() != w_.size())
		throw "[Stochastic Linear System] Invalid w dimensions.";
	w = w_;
}
