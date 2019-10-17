/**
 * StochasticLinearSystem.hpp
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
#include <Eigen/Core>
#include "LinearSystem.hpp"

using namespace Eigen;

class StochasticLinearSystem : public LinearSystem {
protected:
    VectorXf v;
    VectorXf w;
public:
    StochasticLinearSystem(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf x0, float dt_, VectorXf v_, VectorXf w_);
    ~StochasticLinearSystem();
    VectorXf predict(VectorXf u);
    VectorXf measure();
    void set_v(VectorXf v_);
    void set_w(VectorXf w_);
};
