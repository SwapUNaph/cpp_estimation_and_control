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
 * @file LinearSystem.h
 * @version 1.0
 *
 * Definition of Linear System.
 *
 * @author Swapneel Naphade <naphadeswapneel@gmail.com>
 */

#pragma once
#include <Eigen/Core>
#include "System.cpp"

using namespace Eigen;

class LinearSystem : virtual public System{
protected:
    MatrixXf F;
    MatrixXf B;
    MatrixXf H;
    MatrixXf D;
public:
    LinearSystem(MatrixXf f, MatrixXf b, MatrixXf h, VectorXf X0,float dt);
    LinearSystem(MatrixXf f, MatrixXf b, MatrixXf h, MatrixXf d, VectorXf X0, float dt);
    ~LinearSystem();
    virtual VectorXf predict(VectorXf U); 
    virtual VectorXf measure(VectorXf U);
    virtual VectorXf measure(void);
    void setF(MatrixXf f);
    void setB(MatrixXf b);
    void setH(MatrixXf h);
    void setD(MatrixXf d);
};
