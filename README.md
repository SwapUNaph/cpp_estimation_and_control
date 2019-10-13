# C++ Estimation and Control
C++ Estimation and Control is a C++ library for estimation and control algorithms.

## Dependencies
* make
* Eigen

## Build instructions _[For Linux and Mac]_:
* Download Eigen library and put the "Eigen" folder in "/usr/include/" directory.
* clone cpp_estimation_and_control repository on your local machine and cd into it.
* In terminal, type `make all`

## Example run instructions:
* `./build/apps/program`
* `python /files/plot_data.py`
* This will plot a graph of an example system which estimates the distance based on acceleration input and noisy distance measurements
![alt text](https://github.com/SwapUNaph/cpp_estimation_and_control/blob/master/files/kalman_filter_plot.png)
