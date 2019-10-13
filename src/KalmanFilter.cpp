#include <Eigen/Dense>
#include "KalmanFilter.h"

using namespace Eigen;

KalmanFilter::KalmanFilter(MatrixXf f, MatrixXf b, MatrixXf h, MatrixXf q, MatrixXf r){
  F = f;
  B = b;
  H = h;
  Q = q;
  R = r;
  
  //Check Matrix dimensions are correct
  if( F.rows() != F.cols() || F.rows() != B.rows() || F.rows() != H.cols()){
    throw "[Kalman Filter] Matrix dimension mismatch.";
    return;
  }
  
  X = VectorXf::Zero(F.rows());
  P = Q;
  K = MatrixXf::Zero(F.rows(), H.rows());

}

KalmanFilter::~KalmanFilter(){}

void KalmanFilter::predict(VectorXf U){
  
  if( U.size() != B.cols() )
    throw "[Kalman Filter::predict] Invalid U dimensions!";
    
  X = F * X + B * U;
  P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(VectorXf Z){
  if( Z.size() != H.rows() )
    throw "[Kalman Filter::update] Invalid Z dimensions!";
  
  MatrixXf PHt = P * H.transpose();
  MatrixXf S = H * PHt + R;
  K = PHt * S.inverse();
  X = X + K * (Z - H * X);
  P = P - K * H * P;
}

void KalmanFilter::filter(VectorXf Z, VectorXf U){
  predict(U);
  update(Z);
}

VectorXf KalmanFilter::getState(void){
  return X;
}

MatrixXf KalmanFilter::getCovariance(void){
  return P;
}

MatrixXf KalmanFilter::getKalmanGain(void){
  return K;
}

void KalmanFilter::setF(MatrixXf f){
  F = f;
}

void KalmanFilter::setB(MatrixXf b){
  B = b;
}

void KalmanFilter::setH(MatrixXf h){
  H = h;
}

void KalmanFilter::setQ(MatrixXf q){
  Q = q;
}

void KalmanFilter::setR(MatrixXf r){
  R = r;
}



