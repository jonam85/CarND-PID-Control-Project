# **CarND-PID-Control-Project**

Author : Manoj Kumar Subramanian

---

## Overview

This repository is as part of my Submission to the Project 4: PID Controller Project for the Udacity Self Driving Car Nano Degree Program Term 2.

In this project,  a PID controller is realized in C++ to provide the control signals for a vehicle in a simulated environment. This project involves the Term 2 Simulator. The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

Udacity has provided the following as a starter for this project.

1. A [GitHub repo with starter code](https://github.com/udacity/CarND-Controls-PID) that was forked for this project.
2. A simulator, downloaded from the [releases](https://github.com/udacity/self-driving-car-sim/releases) page of the project repo.

------

## Project Goals

The goals of this project are the following:

- The code must compile without any errors with cmake and make
- PID class has to be implemented in C++ and the same has to be used to fine tune the control parameters of maneuvering the vehicle around the track in the simulator
- Reflections to be provided on the PID components
- Simulation of the vehicle driving itself should meet the criteria of not leaving out of track

------

## Rubric Points

### Compiling without any errors

I have used Docker for Windows using the DockerToolbox setup and pulled the Udacity's Carnd control kit docker container which constituted the necessary tools required (including cmake, make, gcc & git) for the project.

**<u>Basic Build Instructions</u>**

1. Clone this repo.

2. Make a build directory: `mkdir build && cd build`

3. Compile: `cmake .. && make` 

   - On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`

4. Run it: `./PID` 

   The program should wait listening to port 4567.

**<u>Running the simulator</u>**

Before running the simulator, configure the port forwarding to the port 4567 since the simulator and the c++ program talk using the uWebSocketIO in port 4567.

Switch to the Project 4: PID Controller in the simulator and press SELECT. The simulator will send the current state of the vehicle in port 4567 which is used by the C++ program. In return, the program provides the control data for Steering and Acceleration which is used by the simulator to navigate the vehicle through the track.

INPUT: values provided by the simulator to the c++ program

- ["cte"] => current cross track error w.r.t track center
- ["speed"] => current speed of the vehicle in mph
- ["steering_angle"] => current steering angle
- ["throttle"] => current throttle value

OUTPUT: values provided by the c++ program to the simulator

- ["steering_angle"] <= target steering angle for which the car should drive
- ["throttle"] <= target throttle position to maintain speed

------

### **PID Class Implementation**

The following sections are implemented in the program.

1. PID Init - that assigns the Kp, Ki and Kd parameters primarily to the respective PID object created.
2. Update Error - based on the error difference between the set value and the current value, the output is returned as a combination of PID control update. "***(Kp x p_error) + (Ki x i_error) + (Kd x d_error)***"
3. The above two methods are sufficient for realizing the PID controller class, however, the following sub-functions were implemented for tuning purposes.
   1. Twiddle
   2. Total Error
   3. Average Error
   4. Re-Init

------

### **Reflections on PID Parameters**

As a general feedback on the PID parameters, the following are the theoretically expected and also experimentally observed in this project.

**Kd Parameter**

Kd Parameter is the Proportional Gain Parameter that is multiplied with the error value calculated as the difference between the set value and the current value.

Kd Parameter influences how fast the current values meets the set values. In other words, this reduces the Rise time drastically, and in the process, this overshoots a little and oscillates over the set value in both the positive and negative direction. Hence, it is appropriate to tell that this reduces the Steady state error but does not eliminate.

**Ki Parameter**

Ki Parameter is the Integral Gain Parameter that is multiplied with the sum of all errors over a period of time that is previously observed by the controller. In other words, this accumulates the error over time and provides the necessary control reaction for the stored error value. Ki also improves in the rise time, but compared to Kp, Ki produces much overshoots because of the accumulated errors and produces much oscillations before settling down. However, this plays a critical role in eliminating the Steady state error.

**Kd Parameter**

Kd Parameter is the Derivative Gain Parameter that is multiplied with the difference between the current error to the previous error. Hence this directly co-relates with the rate of change of error and dampens the actions that produces much variations in the errors, thus helping in moving towards the Set point in as dampened smooth curve.	This decreases the overshoot but also decreases the rise time and hence delays the process.

------

### **Parameters Values Tuning and Settling in this Project**

Most of the tuning for this project is done manually with the help of Ziegler Nichols tuning method. 

**Manual Tuning Sequence**

All the Parameters were set to zero.

Only the Kp parameter were increased step by step until the vehicle is seen oscillating. If the vehicle oscillates more and goes off track, then the values are decreased.

The Kd parameter is then introduced in steps for counter attacking the reactions of Kp parameter to provide smooth transitions.

The above two iterations themselves have produced significant result of maintaining the vehicle in the track at around 30mph.

The vehicle speed PID logic was implemented and added a condition to reduce speed if there are much deviations in the steering cross track error.

**Twiddle Tuning** 

The twiddle function is implemented as per the pseudo code described in the lecture videos. Implementing Twiddle first had some reservations since the pseudo code doesn't actually converge to the optimal values but to look for local solutions around the initial point we set. Also, once a local curve is set for one parameter, then expanding the curve to look for an update based on other parameter update was not proper in twiddle code. And finally, a run means, running the control for at least a minimum number of frames that covers significant events for properly evaluating the different parameter settings. 

Hence, the Twiddle was only used for fine tuning the localized parameters rather than start from 0s.

The parameter Ki was always tending towards 0 in the twiddle outputs. Hence kept a very low value in my final parameter selections.

Final Parameters for Steering PID control:

`Kp = 0.09 `

`Ki = 0.000001 `

`Kd = 2.5`

Final Parameters for Throttle PID control:

`Kp = 0.5`

`Ki = 0.002`

`Kd = 1`

I didn't spend time on fine tuning this since the results were nominal to maintain around 78mph with the set value of 80mph.

## Simulation Video Link

Here is the link for my video running at 78mph with the PID values set as mentioned.

https://youtu.be/avIJihTjzvs



------

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
