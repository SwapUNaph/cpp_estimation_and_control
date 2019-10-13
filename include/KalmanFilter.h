#pragma once
#include <Eigen/Dense>
using namespace Eigen;

class KalmanFilter{
private:
    MatrixXf F;
    MatrixXf B;
    MatrixXf H;
    MatrixXf P;
    MatrixXf R;
    MatrixXf Q;
    MatrixXf K;
    VectorXf X;
public:
    KalmanFilter(MatrixXf F, MatrixXf B, MatrixXf H, MatrixXf Q, MatrixXf R);
    ~KalmanFilter();
    void predict(VectorXf U);
    void update(VectorXf Z);
    void filter(VectorXf Z, VectorXf U);
    VectorXf getState(void);
    MatrixXf getCovariance(void);
    MatrixXf getKalmanGain(void);
    void setF(MatrixXf f);
    void setB(MatrixXf f);
    void setH(MatrixXf f);
    void setQ(MatrixXf f);
    void setR(MatrixXf f);
};
