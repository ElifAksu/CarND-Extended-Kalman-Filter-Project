## Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project, kalman filter is used to estimate the state of a moving object of interest with noisy lidar and radar measurements. 
Two main codes provide the sensor fusion output. These are:
* `kalman_filter.cpp` : defines generic kalman filter prediction, update and initialization steps. 
* `FusionEKF.cpp` : takes sensor data and updates the states according to the sensor type.  

The outputs are evaluated with using Simulator. Estimation output's error values are calculated from ground truth coming from simulator. Two datasets are used for tests. The estimation results are given in the following figures. The RMSE values can be seen also onn the right side of the figures.  

Dataset1 results: 
<p align="center">
  <img width="800" height="400" src="./images/dataset1_Carnd.png ">
</p>

Dataset2 results:
<p align="center">
  <img width="800" height="400" src="./images/dataset2_carnd.png ">
</p>


The Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 



## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `





