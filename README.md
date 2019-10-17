# C++ Estimation and Control
C++ Estimation and Control is a C++ library for estimation and control algorithms.

## Dependencies
* make
* Eigen

## Build instructions _[For Linux and Mac]_:
Follow the steps to download and build the repository
1. `git clone https://github.com/SwapUNaph/cpp_estimation_and_control`
2. `cd cpp_estimation_and_control` 
3. `make all`

## Example run instructions:
1. `./build/apps/program`
2. `cd files && python plot_data.py`

This will plot a graph of an example system which estimates the distance based on acceleration input and noisy distance measurements
![alt text](https://github.com/SwapUNaph/cpp_estimation_and_control/blob/master/files/kalman_filter_plot.png)

