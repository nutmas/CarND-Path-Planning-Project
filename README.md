# CarND-Path-Planning-Project
## Safely navigate around a virtual highway with other traffic, while maintaining, but not exceeding a speed close to the speed limit.

---

[//]: # (Image References)

[image1]: ./support/SimulatorStartup.png "Simulator Startup Window"
[image2]: ./support/Path_Planning_simulator.png " Path Planning Simulator"
[image3]: ./support/Path_Planning_running.png " Path Planning Running"
[image4]: ./support/Path_Planning_result.png " Path Planning Result"
[image5]: ./support/Path_Planning.gif " Path Planning Video Clip"


## Overview
Implementation of a Path Planning controller in C++, which navigates a vehicle around the lake race track in the Udacity simulator. The simulator provides the ego vehicle with localisation information. The Path Planner utilises this information to drive within the lane boundaries and perform manoeuvres based on behaviour and position of the other traffic to try to maintain a speed as close to, but to never exceed the set speed limit of 50MPH. 

The following information passed to the ego vehicle from the simulator: 

    Ego Vehicle Localisation Data:
    + ["x"] The car's x position in map coordinates
    + ["y"] The car's y position in map coordinates
    + ["s"] The car's s position in frenet coordinates
    + ["d"] The car's d position in frenet coordinates
    + ["yaw"] The car's yaw angle in the map
    + ["speed"] The car's speed in MPH 
    
    Sensor data showing information about other vehicles on the course
    + ["sensor_fusion"] - array of data about each detected vehicle
    + Array element [0] Car ID 
    + Array element [1] Car X in map coordinates
    + Array element [2] Car Y in map coordinates
    + Array element [3] Car X velocity in m/s
    + Array element [4] Car Y velocity in m/s
    + Array element [5] Car S in frenet coordinates
    + Array element [6] Car D in frenet coordinates
    
    Previous Path Data passed to simulator with processed points removed:
    + ["previous_path_x"] list of previous x points
    + ["previous_path_y"] list of previous y points
    + ["end_path_s"] list of previous frenet s points values
    + ["end_path_d"] list of previous frenet d points values


The Path Planning receives this information and processes it to produce an understanding of the ego vehicle in relation to detected vehicles and lane positions. The planner then decides whether the vehicle should alter its speed or change lanes to avoid colliding with other traffic.

Once the manoeuvre decision is complete the planner builds a target trajectory of the required manoeuvre from existing car position, previous path points and projected points based on target lane position. These points are then fitted to a spline. Finally the spacing of the points are determined based on the current target speed to control the ego vehicle speed. 

These vectors are then passed back to the simulator perfect controller to show the vehicle resulting motion in the simulator.

---

## Installation steps

To run this code the following downloads are required:

1. Make a project directory `mkdir project_udacity && cd project_udacity`
2. Clone this repository into the project_udacity directory. `git clone https://github.com/nutmas/CarND-Path Planning-Project.git`
3. Setup environment. `cd CarND-Path Planning-Project\` and launch the script to install uWebSocketIO `./install-mac.sh`. Alternatively for Ubuntu installs launch `./install-ubuntu.sh`. The environment will be installed with these scripts.
4. Download Term 3 Simulator: 
      * Download from here: [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).
      * Place the extracted folder into the project_Path Planning directory. 
      * Within the folder run simulator `term3_sim`; if successful the following window should appear:
      ![alt text][image1]

---

## Other Important Dependencies

* cmake >= 3.5
* make >= 4.1 
* gcc/g++ >= 5.4

---

## Build the code

1. From the project_udacity folder change to folder created from cloning `cd CarND-Path Planning-Project`
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 

---

## Usage

After running the script, confirming the operation of the simulator and building the code the Path Planning program is ready to be run.

1. From terminal window; change to build folder of project `cd ~/project_udacity/CarND-Path Planning-Project/build/`
2. Run Path Planning: `./path_planning `
3. Message appears `Listening to port 4567` the Path Planning is now running and waiting for connection from the simulator
4. Run the simulator `term3_sim`
5. In the simulator window, press 'Select' button to start the Path Planning Simulator.

![alt text][image2]

6. In the Path Planning terminal window `Connected!!!` will now appear.
7. The Path Planning is now fully operational, the vehicle will proceed to navigate around the track, controlling its own speed and steering autonomously. A series of green spheres connected by a green line protrudes from the front of the ego vehicle to indicate its planned manoeuvre.

The Animation will start and the vehicle can be seen moving around the screen.

![alt text][image3]

---

## Model Documentation

The information received from the simulator required preparation before being passed to the Path Planning.

#### Path Planning Pipeline:

**Localisation** 

The ego vehicle position is passed from the simulator. This information is converted into a lane reference to understand the lane position on the road. `Line 305 in main.cpp` calls a function getLane() `Lines 132-153 in main.cpp` to interpret the ego vehicle frenet d value and return the lane number.  `Lines 309-318 in main.cpp` take the ego lane and compared it to the latest target lane. This sets a flag to determine current stability of ego vehicle. This flag is utilised in other areas of the code to restrict or allow a lane change manoeuvre.


**Prediction** 

Prediction of other vehicle positions is performed by cycling through the received sensor detection data and determining: 
  - If there is a vehicle ahead of ego vehicle `Line 321 in main.cpp`, which calls a function carAhead() `Lines 156-191 in main.cpp`. This function checks the presence of a car ahead by 30m and in same lane as ego vehicle. If a vehicle is detected the unique tracking ID is returned.
  - If there is a vehicle in the left adjacent lane a flag is set in `Line 367 in main.cpp`. The function laneChangeLeft()`Lines 20-73 in main.cpp` is called and the value return is assigned to the flag. This function cycles through the sensor data and checks for the presence of vehicles to the left of the vehicle. A zone is set 35m ahead of Ego vehicle and 15m rearwards. If this area has no vehicle detections in the sensor data a flag is returned which indicates the lane is clear.
  - Similarly if a there is a vehicle in the right adjacent lane a flag is set in `Line 369 in main.cpp`. The function laneChangeRight()`Lines 76-125 in main.cpp` is called and the value return is assigned to the flag. This function is very similar to leftChangeRight function, but modified for right lane planning.


**Behaviour Planner**

The planned behaviour of the vehicle is decided in `Lines 321-400 in main.cpp`
If there is no vehicle in front of the vehicle - such as at the start, then the code will jump to `Line 396-400 in main.cpp`, which accelerates the vehicle speed at 5m/s if the ego velocity is below the max speed of 49.7mph.
If there is nothing in front of vehicle and max speed is reached then the target speed `ref_vel` is maintained.
If the function `carAhead()` at `Line 321 in main.cpp` returns a tracked vehicle ID the behaviour planner sets a flag to allow a lane change plan to be considered, it then calculates the tracked vehicle's speed and distance. `Lines 335-337 of main.cpp`. A very simple speed control is implemented to decrease the speed of the ego vehicle as it approaches tracked vehicle. `Lines 341-361 of main.cpp`.
In order to manoeuvre around the slower traffic and try to maintain the maximum allowable speed, the planner checks if car_left & car_right flags are set to true indicating the lane is clear, `Lines 367-369 in main.cpp`. If the lane change flag is true and the lane is clear the target lane will be set to either the adjacent left or right lanes to instruct the ego car to perform the lane change. `Lines 372-391 in main.cpp`. Within this code `lane_change` and `lane_steady` flags are set to false to prohibit any lane change until stability and further checks are carried out.

**Motion Trajectory**

`Lines 404-512 in main.cpp` are the implementation of the trajectory which will be passed back to the simulator's perfect controller to move the ego vehicle.
In order to build the trajectory the function will check to see if there is any previous path. `Lines 412-441 in main.cpp`
 - if the ego vehicle is past the end of the previous path, or there was no previous path then a vector of previous xy points and current point to initialise the path vector creation of a path tangent to the ego vehicle heading.
 - If the previous path has points then they are used to create the initial planned path points.
 `Lines 444-455 in main.cpp` utilise the function `getXY()` to generate waypoints at 30m, 60m and 90m ahead of the ego vehicle in the current target lane decided in behaviour planner.
 These coordinates are then converted from map coordinates to vehicle local coordinates. This vector of points are used to generate a spline `Lines 468-470 in main.cpp`.
 Finally the planner creates a vector with a resolution of 50 points along the X axis. The spacing of these points is dependent on the current speed of the ego vehicle. These spline is then interpolated with these 50 points in order to generate the Y axis points of the path for the ego vehicle to follow `Lines 490-497 in main.cpp`.
 The points are transformed back to global map coordinates in preparation to be fed back the the simulator controller. `Lines 505-506 in main.cpp`
 The resulting vector of xy points which represent the trajectory path the ego vehicle must follow to navigate safely is passed back to the simulator. `Lines 511-512 in main.cpp`


---

## Reflection

The path planner that is implemented achieved the required results. 
Two areas that given the time could create a more robust and reliable planner are:  
- Shortly after a vehicle detection a very simple speed control is implemented, which is not perfectly smooth. This was an ideal opportunity to implement a simple PD control to match the speed of the vehicle in front.  
- I did experiment with cost functions and tried to manage the the transition with this, however my planning scenarios were becoming very complicated and taking too much time to build and implement, so I opted for a simpler state machine implementation.



---

## Results

The vehicle successfully navigates around the track without incident.

![alt text][image4]

Criteria for success:

 `The car is able to drive at least 4.32 miles without incident. Incident includes exceeding acceleration/jerk/speed limits, collisions, and driving outside of the lanes` 

 The following video shows a short duration of the vehicle driving around the track being controlled by the Path Planner.  
 The green line shows the planned trajectory the vehicle will follow.  
 The right area of the screen shows the progress of the vehicle, without incident. Other information shown is current acceleration and jerk.

![alt text][image5]

A video of the full distance can be viewed by downloading: [Path Planning_Video](https://youtu.be/65KFKfTl0VQ)



---

## License

For License information please see the [LICENSE](./LICENSE) file for details

---

