# MPC Control for Tracking
Udacity Self-Driving Car Nanodegree Term 2, Project 5

## Project Basics
Using Model Predictive Control (MPC), this project involves writing a C++ program that can drive a simulated car around a virtual track using specific waypoints from the track itself. The simulated car's actuators have a 100ms latency (delay) that must be accounted for as well as part of the MPC calculation.

### Results shows
![MPC_Tracking result](./MPC.gif)

## Discussion/Reflection
### The Model
My MPC model starts out by taking in certain information from the simulator: 
* ptsx (x-position of waypoints ahead on the track in global coordinates)
* ptsy (y-position of waypoints ahead on the track in global coordinates)
* px (current x-position of the vehicle's position in global coordinates)
* py (current y-position of the vehicle's position in global coordinates)
* psi (current orientation angle of the vehicle, converted from the simulator's format to that expected in mathematical formulas)
* v (current velocity of the vehicle)
* delta (current steering angle of the car, i.e. where the wheels are turned, as opposed to the actual orientation of the car in the simulator at that point [psi])
* a (current throttle)

#### Polynomial Fitting & Preprocessing
Now, in order to simplify the calculations, I transform the points from the simulator's global coordinates into the vehicle's coordinates. This is done in lines 102-107 of `main.cpp`. First, each of the waypoints are adjusted by subtracting out px and py accordingly such that they are based on the vehicle's position. Next, the waypoint coordinates are changed using standard 2d vector transformation equations to be in vehicle coordinates:
* ptsx_car[i] = x * cos(-psi) - y * sin(-psi)
* ptsy_car[i] = x * sin(-psi) + y * cos(-psi)


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](http://m.makaidong.com/fuhang/581_18472767.html)
  * install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipop://weibo.com/ttarticle/p/show?id=2309404271776447247868)
  * Linux
    * You will need a version of Ipopt 3.12.7 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Simulink tool](https://github.com/udacity/self-driving-car-sim.git)
  * Please download term2 and use project 5
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
