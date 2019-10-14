#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "KalmanFilter.hpp"

using namespace Eigen;
using namespace std;

int main(){
	float dt = 0.1;
	float T = 10;
	
    MatrixXf F(2,2);
    F << 1,dt,0,1;
    
    MatrixXf B(2,1);
    B << 0,dt;
    
    MatrixXf H(1,2);
    H << 1,0;
    
    MatrixXf Q(2,2);
    Q << 0.01, 0, 0, 0.01;
    
    MatrixXf R(1,1);
    R << 0.5;
    
    VectorXf X0(2);
    X0 << 5.0, 0.0;
    
    LinearSystem LS(F,B,H,X0,dt);
    ArrayXf t = ArrayXf::LinSpaced((int)(T/dt),0,T);

    KalmanFilter KF(F,B,H,Q,R,X0,dt);
    ArrayXf U = t.sin();

    
    ofstream outputFile;
    outputFile.open("../files/measurements.csv");
	outputFile << "Time, Inputs, Actual State, Measurements, Filtered\n";
    
    for(int i=0; i < U.size(); i++){
        VectorXf u(1);
        u << U(i); 
        LS.predict(u);
        VectorXf meas = LS.measure();
        //VectorXf actual = LS.getX();
        //KF.filter(meas,u);
        //VectorXf filtered = KF.getX();
        outputFile << t(i) << ", " << U(i) << ", " << LS.getX()(0) << ", " << meas << ", " << KF.filter(meas,u)(0)<< "\n";
    }
    
    outputFile.close();
    cout << "File saved as [PROJECT_DIR]/files/measurements.csv" << endl;
    
    return 0;
}
