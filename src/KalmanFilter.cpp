/***
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the Swapneel Naphade nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */


#include <Eigen/Dense>
#include "KalmanFilter.hpp"

using namespace Eigen;

KalmanFilter::KalmanFilter(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf x0, float dt_, MatrixXf q, MatrixXf r) : LinearSystem(f,b,h,x0,dt_), Q(q), R(r), P(q) {
  K = MatrixXf::Zero(f.rows(), h.rows());
}

KalmanFilter::~KalmanFilter(){}

VectorXf KalmanFilter::filter(VectorXf Z, VectorXf U){
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
	X = X + K * (Z - measure() );
	
	//Update covariance
	P = P - K * H * P;
	
	return X;
}


MatrixXf KalmanFilter::getP(void){
  return P;
}

MatrixXf KalmanFilter::getK(void){
  return K;
}

void KalmanFilter::setQ(MatrixXf q){
	if( Q.rows() != q.rows() || Q.cols() != q.cols() )
    	throw "[Kalman Filter::setQ] Invalid Q dimensions!";
 	Q = q;
}

void KalmanFilter::setR(MatrixXf r){
	if( R.rows() != r.rows() || R.cols() != r.cols() )
    	throw "[Kalman Filter::setR] Invalid R dimensions!";
  	R = r;
}



