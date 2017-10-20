# Project 5: MPC Controller

In this project we implement in C++ a controller based on model predictive control (MPC) framework to drive the vehicle around the track. At each iteration, the simulator provides reference points (waypoints) and the position, heading, and the speed of the car. The MPC controller takes these values as input and outputs the steering angle and throttle value.


## Files Submitted
All the files are included in two directory: \src and \build

In the \src directory, the following key files are included:

* main.cpp: the main file that implements the communication with the simulator, reading waypoint and car position data, calculating the steering angle and throttle value via MPC controllers, and sending the steering and throttle data to simulator to drive the car. 
* MPC.cpp and MPC.h: the file and the associated header file that implement MPC controller.
* json.hpp: hpp file provided for using JSON data format for the communication between the main.cpp and the simulator.  This file is included in the stater code. No modification was made
* P5-MPC.md: this report.

In the \build directory, the following key files are included:

* mpc.exe: executable file that implements MPC to drive the car and to maintain a speed at around 40 mph, respectively.
* CMakeList.txt - build instruction for generating executable.

## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Implementation
### State
The state includes x coordinate of the car ```px_car```, y coordinate of the car ```py_car```, heading of the car ```psi_car```, speed ```v```, cross-track error ```cte```, and orientation error ```epsi```. Note that in my implemenation, ``px_car```, ```py_car```, and ```psi_car``` are all in vehicle-centric coordinate. The state is defined as following in main.cpp:

```
state << px_car, py_car, psi_car, v, cte, epsi; 
```
### Actuators
The actuators are steering angle ```delta0``` and acceleration ```a0```.

### Model
The system model that relates state at t+1, [x\_[t+1], y\_[t+1], psi\_[t+1], v\_[t+1], cte[t+1], epsi[t+1] to state at t, [x\_[t], y\_[t], psi\_[t], v\_[t], cte[t], epsi[t], is as following:

* x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
* y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
* psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
* v_[t+1] = v[t] + a[t] * dt
* cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
* epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

These update equations are converted to optimization constraints in MPC.cpp

```
fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
fg[2 + cte_start + i] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[2 + epsi_start + i] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```

###Timestep Length and Frequency

The prediction horizon is the duration over which future predictions are made. We’ll refer to this as T, which is the product of two other variables, N and dt. 

* For a reference speed of 45mph, I find that T in the range of 0.4 to 0.8 second works well.
* N is the number of timesteps in the horizon. N determines ts the number of variables the optimized by MPC. A large N not only drives the computational cost but also reduce quality optimization solution.  In the implementation, I choose N to be between 8 to 10.
* dt is how much time elapses between actuations.Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. In the implementation, I find that dt in the range of 0.03 to 0.06 works well for a reference speed of 45mph.

In the submitted file, I use T=0.5 second, N=10, and dt=0.05 second.

##Polynomial Fitting and MPC Preprocessing

The readings of waypoints, car positions, and car headings are provided in global (map) coordinates. In the implementation, I convert these readings to vehicle-centric coordinates:

```
// Setup car coordinates
Eigen::VectorXd ptsx_car = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
Eigen::VectorXd ptsy_car = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());
          
// Convert ptsx_car and ptsy_car to car coordinates
for (int i = 0; i < ptsx_car.size(); i++) {
     double x_tmp = ptsx_car[i] - px;
     double y_tmp = ptsy_car[i] - py;
     ptsx_car[i] = x_tmp * cos(psi) + y_tmp * sin(psi);
     ptsy_car[i] = - x_tmp * sin(psi) + y_tmp * cos(psi);
  }
```          

Then the third order polynomial fit is performed on ```ptsx_car``` and ```ptsy_car```:

```// 3rd order polynomial fit in car coordinates
          auto coeffs = polyfit(ptsx_car, ptsy_car, 3);
```
          
### Latency

To account for the 100 millisecond latency, I feed the solver the state 100 millisecond (0.1 sec) into the future. For example,   ```px_car``` is zero in vehicle-centric coordinate.  If we consider the latency, we have  ```px_car = v * latency_adjustment ```. 

```
// time adjustment for latency
double latency_adjustment = 0.1; //100 ms latency
double px_car = v*latency_adjustment;
double py_car = 0;
double psi_car = 0;
         
// The cross track error is calculated by evaluating 
//at polynomial at x, f(x), and subtracting y.
double cte = polyeval(coeffs, 0) - py_car;
  
// calculate the orientation error
double epsi = psi_car-atan(coeffs[1] + 2 * px_car * coeffs[2] + 3 * px_car * px_car * coeffs[3]);
state << px_car, py_car, psi_car, v, cte, epsi;
```

## Simulation

The simulation result is shown in [https://youtu.be/Qpo5SFfWp5k](https://youtu.be/Qpo5SFfWp5k). The car closely tracks the waypoints with a reference speed of 45mph.