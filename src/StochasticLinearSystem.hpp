/****************************************************************************
 *
 *   Copyright (c) 2019 Swapneel Naphade. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name cpp_estimation_and_control nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file StochasticLinearSystem.hpp
 * @version 1.0
 *
 * Definition of Stochastic Linear System.
 *
 * @author Swapneel Naphade <naphadeswapneel@gmail.com>
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
