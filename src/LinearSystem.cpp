/**
 * LinearSystem.cpp
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
#include "LinearSystem.hpp"


/**
 * @brief Linear System class constructor
 * 			Based on linear system defined by:
 * 					X_dot = F * X + B * U
 * 					Z = H * X
 * @param f Dynamics Matrix
 * @param b Control Input Matrix 
 * @param h Measurement Matrix
 * @param x0 Initial Estimate
 * @param dt_ Time Step
 * @returns nothing
 * 
 * 
 */
LinearSystem::LinearSystem(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf x0, float dt_) : F(f), B(b), H(h), X(x0), dt(std::abs(dt_)){
  //Check Matrix dimensions
  if( f.rows() != f.cols() || f.rows() != b.rows() || f.rows() != h.cols() || f.rows() != x0.size() )
    throw "[Linear System] Matrix dimension mismatch.";
  X_dot = VectorXf::Zero(X.size());
}



/**
 * @brief Linear System class destructor
 * @returns nothing
 * 
 * 
 */
LinearSystem::~LinearSystem(){}



/**
 * @brief Predict and Update the next state after time step
 * @param U Input to the system
 * @returns X Updated state
 * 
 * 
 */
VectorXf LinearSystem::predict(VectorXf U){
  if( U.size() != B.cols() )
    throw "[Linear System::update] Invalid U dimensions!";
  VectorXf X_dot_current = F * X + B * U;
  X = X + (X_dot + X_dot_current) / 2 * dt;
  X_dot = X_dot_current;
  return X;
}


/**
 * @brief Measure the current state based on current measurement configuration
 * @returns Measurement output of the system
 * 
 * 
 */
VectorXf LinearSystem::measure(void){
    return (H * X);
}



/**
 * @brief Set Current State (X)
 * @param x X
 * 
 * 
 */
void LinearSystem::setX(VectorXf x){
	if( X.size() != x.size() )
    	throw "[Linear System::setX] Invalid X dimensions!";
	X = x;
}


/**
 * @brief Set time step (dt)
 * @param dt_ dt
 * 
 * 
 */
void LinearSystem::set_dt(float dt_){
	dt = abs(dt_);
}



/**
 * @brief Get current state (X)
 * @returns X
 * 
 * 
 */
VectorXf LinearSystem::getX(){
	return X;
}



/**
 * @brief Get time step
 * @returns dt
 * 
 * 
 */
float LinearSystem::get_dt(){
	return dt;
}

/**
 * @brief Set System Dynamics matrix (F)
 * @param f F
 * 
 * 
 */
void LinearSystem::setF(MatrixXf f){
	if( F.rows() != f.rows() || F.cols() != f.cols() )
    	throw "[Linear System::setF] Invalid F dimensions!";
	F = f;
}


/**
 * @brief Set Input matrix (B)
 * @param b B
 * 
 * 
 */
void LinearSystem::setB(MatrixXf b){
	if( B.rows() != b.rows() || B.cols() != b.cols() )
    	throw "[Linear System::setB] Invalid B dimensions!";
	B = b;
}



/**
 * @brief Set measurement matrix (H)
 * @param h H
 * 
 * 
 */
void LinearSystem::setH(MatrixXf h){
	if( H.rows() != h.rows() || H.cols() != h.cols() )
    	throw "[Linear System::setH] Invalid H dimensions!";
	H = h;
}

