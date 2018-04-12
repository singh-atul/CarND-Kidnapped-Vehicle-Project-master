# CarND-Kidnapped-Vehicle-Project

This repository contains C++ implementation of a 2 dimensional Particle Filter to localize a vehicle kidnapped in a closed environment . This task was implemented to complete the goal of Term 2 in Udacity self driving Car .

The original repository for the project can be found [here](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).

 
## Background

Localization can defined as predicting the location of the vehicle with high accuracy. This location is in respect to  the global map of the locality in which the vehicle is present.

We can localize the vehicle using data from multiple devices. One common method is by  using the global positioning the system , which make use of traingulation to predict the position of the object using multiple satellite. But the problem with GPS is that its error in accuracy may range from 3-4 m to 10-50 m. Hence the use of only GPS to localaize the vehicle is not reliable.

To achieve the accuracy of 1-3 cm  Laser sensor (LIDAR) and/or Radial distance and angle sensor (RADAR) is used and fused together using a Particle Filter.

## Project Goal

In this project robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

We have to implement a 2 dimensional particle filter in C++. Particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step this filter will also get observation and control data. 


## Running the Code

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


# Implementing the Particle Filter

Each major step involved in implementation is shown below 

###### 


The code implentation of these steps are present in particles.cpp file.


The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Inputs to the Particle Filter
Inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Output
Particle filter implemented was run on Udacity's simulator and its error and performance was noted. Below are the results with 20 and 100 particles respectively:

Run 1 : 20 particle

####

Run 2 : 100 particle

####


## Steps for building the project

### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
 * Linux and Mac OS, you can also skip to installation of uWebSockets as it installs it as a dependency.
 
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
  * Linux and Mac OS, you can also skip to installation of uWebSockets as it installs it as a dependency.
  
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
  * Linux and Mac OS, you can also skip to installation of uWebSockets as it installs it as a dependency.
  
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`. This will install cmake, make gcc/g++ too.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x.

* Simulator. You can download these from the [Udacity simulator releases tab](https://github.com/udacity/self-driving-car-sim/releases).

### Running the project in Ubuntu

  1. Execute every step from ./install-ubuntu.sh. This will install gcc, g++, cmake, make and uWebsocketIO API.
  
  2. Build and run project by running ./build.sh.
  
  3. In case of error, manually build the project and run using:
    a. mkdir build && cd build
    b. cmake ..
    c. make
    d. ./particle_filter
    
  4. Run the Udacity simulator and check the results


