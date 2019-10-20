/**
 * LinearSystem.hpp
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
#include <cmath>
#include <Eigen/Core>

using namespace Eigen;

class LinearSystem {
protected:
    MatrixXf F;
    MatrixXf B;
    MatrixXf H;
    VectorXf X;
    VectorXf X_dot;
	float dt;
public:
    LinearSystem(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf X0,float dt);
    ~LinearSystem();
    VectorXf predict(VectorXf U); 
    VectorXf measure(VectorXf U);
    VectorXf measure(void);
    void setX(VectorXf x);
    void set_dt(float dt);
    void setF(MatrixXf f);
    void setB(MatrixXf b);
    void setH(MatrixXf h);
    VectorXf getX();
    float get_dt();
};
