/**
 * KalmanFilter.hpp
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

 
#pragma once
#include <Eigen/Dense>
#include "LinearSystem.hpp"
using namespace Eigen;

class KalmanFilter : public LinearSystem {
protected:
	MatrixXf Q;
	MatrixXf R; 
	MatrixXf P; 
    MatrixXf K;
public:
    KalmanFilter(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf x0, float dt_, MatrixXf q, MatrixXf r);
    ~KalmanFilter();
    VectorXf estimate(VectorXf Z, VectorXf U);
    MatrixXf getP(void);
    MatrixXf getK(void);
    void setQ(MatrixXf q);
    void setR(MatrixXf r);
};
