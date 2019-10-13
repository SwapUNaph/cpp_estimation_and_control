#pragma once
#include <Eigen/Core>

using namespace Eigen;

class LinearSystem{
private:
    MatrixXf F;
    MatrixXf B;
    MatrixXf H;
    VectorXf X;
    float v;
    float w;
public:
    LinearSystem(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf X0, float v_, float w_);
    ~LinearSystem();
    void update(VectorXf U);
    VectorXf measure(void);
    VectorXf getState(void);
};
