/**
 * main.cpp
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

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "KalmanFilter.hpp"
#include "StochasticLinearSystem.hpp"

using namespace Eigen;
using namespace std;

int main(){
	float dt = 0.01;
	float T = 10;
	
    MatrixXf F(2,2);
    F << 0,1,0,0;
    
    MatrixXf B(2,1);
    B << 0.0,1.0;
    
    MatrixXf H(1,2);
    H << 1,0;
    
    MatrixXf Q(2,2);
    Q << 0.01, 0, 0, 0.01;
    
    MatrixXf R(1,1);
    R << 0.5;
    
    VectorXf X0(2);
    X0 << 0.0, 0.0;
    
    VectorXf X0e(2);
    X0e << 2.0, 1.0;
    
    VectorXf v_(2);
    v_ << 0.001, 0.001;
    
    VectorXf w_(1);
    w_ << 0.5;
    
    StochasticLinearSystem SLS(F,B,H,X0,dt, v_, w_);
    ArrayXf t = ArrayXf::LinSpaced((int)(T/dt),0,T);
    //  | x |  =  | 1  dt | | x |  +  | 0 | a   +  noise                             
    //  | v |     | 0  1  | | v |     | dt| 
	                     
    KalmanFilter KF(F,B,H,X0e,dt,Q,R);
    ArrayXf U = 5 * (5 * t).sin(); // a = 5sin(5t)

    
    ofstream outputFile;
    outputFile.open("../files/measurements.csv");
	outputFile << "Time, Inputs, Actual State, Measurements, Filtered\n";
    
    for(int i=0; i < U.size(); i++){
        VectorXf u(1);
        u << U(i); 
        SLS.predict(u);
        VectorXf meas = SLS.measure();
        outputFile << t(i) << ", " << U(i) << ", " << SLS.getX()(0) << ", " << meas << ", " << KF.estimate(meas,u)(0)<< "\n";
    }
    
    outputFile.close();
    cout << "File saved as [PROJECT_DIR]/files/measurements.csv" << endl;
    
    return 0;
}
