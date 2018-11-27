# **Path Planning Project**
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Goals
---
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Car's localization and sensor fusion data is provided along with a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

Basic Build Instructions
---
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Reflection
---
Path planner is designed with four states, `KEEP_LANE`, `SHIFT_LEFT`, `SHIFT_RIGHT` and `TOO_CLOSE`.

If there is no car in front in 30m distance in `s` coordinates the car will be in `KEEP_LANE` state. In this state car's speed is increased at the rate of 0.224mph till 49.5mph.

If there's a car is in front in 30m distance in `s` coordinates the car will look at the other lanes to switch.
To switch:
- If the car is in `MIDDLE` lane, then there is a possibility that it can switch to either `LEFT` or `RIGHT` lane. So each lane is checked for the safety to see that no car is ahead in 30m range and there is no car behind 20m.
    - If both lanes are safe, then the car will switch to a lane where the car in that lane is far ahead.
    - If one of the lane is safe it is switched to that lane.
- If the car is in `LEFT` lane, then it will switch right to the `MIDDLE` lane if it is safe.
- If the car is in `RIGHT` lane, then it will switch left to the `MIDDLE` lane if it is safe.

If the lane switch is not feasible and a car is in front then the ego car will enter to `TOO_CLOSE` state. In this state the speed of the front vehicle is calculated and the speed is reduced until that velocity and is maintained in the same speed.

After the speed and lane is calculated, next is calculation of waypoints. Waypoints are calculated using spline library. A spline is built with one previous point, one present point and two future predicted points. Path is calculated for a total 50 points and a distance of 50m. 

When calculating the next trajectory, to avoid jerk the end `s` coordinate of the previous path is given for the trajectory calculaton using spline.

Video of the path planner in action can be found here[https://drive.google.com/file/d/1pc27BymcaEVGrGogHLlMAZNK8bwAj801/view?usp=sharing]