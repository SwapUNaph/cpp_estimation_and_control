/**
 * KalmanFilter.cpp
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


#include <Eigen/Dense>
#include "KalmanFilter.hpp"

using namespace Eigen;


/**
 * @brief Kalman Filter class constructor
 * 			Based on linear system defined by:
 * 					X_dot = F * X + B * U
 * 					Z = H * X
 * @param f Dynamics Matrix
 * @param b Control Input Matrix 
 * @param h Measurement Matrix
 * @param x0 Initial Estimate
 * @param dt_ Time Step
 * @param q Process noise covariance matrix
 * @param r Measurement noise covariance matrix
 * @returns nothing
 * 
 * 
 */
KalmanFilter::KalmanFilter(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf x0, float dt_, MatrixXf q, MatrixXf r) : LinearSystem(f,b,h,x0,dt_), Q(q), R(r), P(q) {
  K = MatrixXf::Zero(f.rows(), h.rows());
}



/**
 * @brief Kalman Filter class destructor
 * @returns nothing
 * 
 * 
 */
KalmanFilter::~KalmanFilter(){}



/**
 * @brief Estimate the state based on control input
 * @param Z Measurement
 * @param U Control Input
 * @returns Estimated state vector
 * 
 * 
 */
VectorXf KalmanFilter::estimate(VectorXf Z, VectorXf U){
	//Predict
	predict(U);
	
	//Propagate covariance
	MatrixXf A = MatrixXf::Identity(F.rows(),F.cols()) + F * dt ;
	P = A * P * A.transpose() + Q;
	
	//Kalman Gain
	MatrixXf PHt = P * H.transpose();
	MatrixXf S = H * PHt + R;
	K = PHt * S.inverse();
	
	//Update estimate
	X = X + K * ( Z - measure() );
	
	//Update covariance
	P = P - K * H * P;
	
	return X;
}



/**
 * @brief Get Error Covariance Matrix (P)
 * @returns P 
 * 
 * 
 */
MatrixXf KalmanFilter::getP(void){
  return P;
}



/**
 * @brief Get Kalman Gain (K)
 * @returns K
 * 
 * 
 */
MatrixXf KalmanFilter::getK(void){
  return K;
}



/**
 * @brief Set Process Covariance matrix (Q)
 * @param q Q
 * 
 * 
 */
void KalmanFilter::setQ(MatrixXf q){
	if( Q.rows() != q.rows() || Q.cols() != q.cols() )
    	throw "[Kalman Filter::setQ] Invalid Q dimensions!";
 	Q = q;
}



/**
 * @brief Set Measurement noise caovariance matrix (R)
 * @param r R
 * 
 * 
 */
void KalmanFilter::setR(MatrixXf r){
	if( R.rows() != r.rows() || R.cols() != r.cols() )
    	throw "[Kalman Filter::setR] Invalid R dimensions!";
  	R = r;
}



