# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Overview
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Implementation

My implementation consists of ``Vehicle`` data type, which can reresent two types of vehicles - either ego or any other. They are distinquished by calling constructors with different overrides. The constructor for creating Ego vehicle contains such parameters as x,y,s,d positions, yaw and previous paths for x and y coordinates. Other vehicles constructor takes id and x,y,s,d position parameters, speed. To solve this task I implemented two methods in ``Vehicle`` class: 
```cpp
void setVehicleParams(const vector<Vehicle>& vehicles);

vector<vector<double>> processTrajectory(const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y)
```

First one generates valid velocity and lane number for ego vehicle and saves them as attributes of Vehicle class. Second method uses generated parameters to create smooth driving trajectory based on splines. You can find them in Vehicle.cpp file.
`` setVehicleParams`` checks other vehicles position to infer which type of actions are needed to avoid collisions and safely change lines. The positions of all the other vehicles are analyzed relative to the ego vehicle. If the ego vehicle is within 30 meters of the vehicle in front, the boolean too_close is flagged true. 
If vehicles are within that margin on the left or right, car_left or car_right are flagged true, respectively. If a car is ahead within the gap, the lanes to the left and right are checked. If one of them is empty, the car will change lanes. Otherwise it will slow down.

If the way in front of the car is clear, the car will speed up.

``processTrajectory`` creates drivable trajectory using splines as it is explaine in the QA session for this project.

In [main.cpp](./src/main.cpp) everything is pretty clear - creation of ego vehicle and the list of other vehicles followed by calls to the methods I have described above.

### Result

Here you can check the [video](https://youtu.be/iDKkFrOWz5A)


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

