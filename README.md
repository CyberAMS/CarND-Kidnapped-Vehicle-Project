# Project: Kidnapped Vehicle

This project has been prepared by Andre Strobel.

The goal of this project is to program a [particle filter](https://en.wikipedia.org/wiki/Particle_filter) in *C/C++* to demonstrate how to localize a moving vehicle in a map. The vehicle measures distances to objects which can be associated to known landmarks in the map. This self-localization is the first step in [robot navigation](https://en.wikipedia.org/wiki/Robot_navigation).

The following table shows an overview of the most important files:

| File                         | Description                                                    |
|------------------------------|----------------------------------------------------------------|
| README.md                    | This file                                                      |
| build.sh                     | Script to build the particle filter executable                 |
| run.sh                       | Script to run the particle filter executable                   |
| data/map_data.txt            | Provided map data with landmark definitions                    |
| src/helper_functions.h       | Source code of helper functions for the particle filter        |
| src/main.cpp                 | Source code of main function of particle filter filter project |
| src/map.h                    | Source code of map object                                      |
| src/particle_filter.{h, cpp} | Source code of particle filter object                          |

---

## Content

1. Tool chain setup
    1. Gcc, Cmake, Make and uWebSocketIO
    1. Udacity Simulator
1. Data objects and structures
    1. Maps and landmarks
    1. Particle filter and particles
1. Particle filter implementation
    1. Primary equations
    1. Implementation in C/C++
    1. Debugging environment
1. Execution with given input data
    1. Commands to start the simulation
    1. Simulation results
1. Discussion

[//]: # (Image References)

[image1]: ./docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_5.gif
[image2]: ./docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_20.gif
[image3]: ./docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_100.gif

---

## 1. Tool chain setup

### 1. Gcc, Cmake, Make and uWebSocketIO

This project requires the following programs:

* gcc/g++ >= 5.4
  - Linux: gcc / g++ is installed by default on most Linux distros
  - Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  - Windows: recommend using [MinGW](http://www.mingw.org/)
  
* cmake >= 3.5
  - All OSes: [click here for installation instructions](https://cmake.org/install/)
  
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  - Linux: make is installed by default on most Linux distros
  - Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  - Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
  
* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
  - Works with Linux and Mac systems
  - Windows: Use Docker, VMware or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) (although I wasn't able to get it working with the latest Ubuntu app in Windows 10)

### 2. Udacity Simulator

The extended Kalman filter program connects to the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases) via [uWebSocketIO](https://github.com/uWebSockets/uWebSockets). The simulator is available for Linux, Mac and Windows.

## 2. Data objects and structures

### 1. Maps and landmarks

The map information is loaded from the file `data/map_data.txt` which has the following structure:

| Column | Description |
|--------|-------------|
| 1      | x position  |
| 2      | y position  |
| 3      | landmark id |

The map object class `Map` is defined in `map.h`. The only content is a vector list `landmark_list` of landmarks `single_landmark_s` with the following data structure:

| Variable | Value or Description   |
|----------|------------------------|
| `id_i`   | landmark id as integer |
| `x_f`    | x position as float    |
| `y_f`    | y position as float    |

### 2. Particle filter and particles

The particle filter is defined as object class `ParticleFilter` in `particle_filter.{h, cpp}`. A particle filter instance contains the number of particles in `num_particles`, whether it has been initialized in `is_initialized`, a vector list `weights` containing the weights of all particles as well as a vector list `particles` for all the particles. Each particle is defined as data structure `Particle`:

| Variable       | Value or Description                                                                   |
|----------------|----------------------------------------------------------------------------------------|
| `id`           | particle id as integer                                                                 |
| `x`            | x position as double                                                                   |
| `y`            | y position as double                                                                   |
| `theta`        | heading angle of particle as double                                                    |
| `weight`       | weight showing how good it associates landmarks to the observations as double          |
| `associations` | vector list of landmark ids (integers) associated to the observations                  |
| `sense_y`      | vector list of x positions (double) of the observations added to the particle position |
| `sense_x`      | vector list of y positions (double) of the observations added to the particle position |

## 3. Particle filter implementation

### 1. Primary equations

In this project the prediction step of the particle filter assumes a linear bicycle motion model.

```C
// predict state with bicycle motion model - handle zero yaw rate separately
if (fabs(yaw_rate) < ZERO_DETECTION) {
	
	// precalculations
	theta_0 = particles[current_particle].theta;
	velocity_dot = velocity * delta_t;
	
	// motion step
	particles[current_particle].x += velocity_dot * cos(theta_0);
	particles[current_particle].y += velocity_dot * sin(theta_0);
	
}
else {
	
	// precalculations
	theta_0 = particles[current_particle].theta;
	velocity_over_yaw_rate = velocity / yaw_rate;
	theta_dot = yaw_rate * delta_t;
	
	// motion step
	particles[current_particle].x += velocity_over_yaw_rate * (sin(theta_0 + theta_dot) - sin(theta_0));
	particles[current_particle].y += velocity_over_yaw_rate * (cos(theta_0) - cos(theta_0 + theta_dot));
	particles[current_particle].theta += theta_dot;
	
}
```

The distance between two points is calculated with the following function:

```C
inline double dist(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
```

The transformation between vehicle coordinates (observations with change/distance relative to the vehicle) and global map coordinates (particles with position as offset and heading angle alpha) is calculated with the following equations:

```C
	// transformations
	x_map = x_offset_map + (cos(alpha) * x_change_relative) - (sin(alpha) * y_change_relative);
	y_map = y_offset_map + (sin(alpha) * x_change_relative) + (cos(alpha) * y_change_relative);
```

The multi-variate Gaussian distribution is calculated with the following function:

```C
inline double mvg(double x_value, double y_value, double mu_x, double mu_y, double s_x, double s_y) {
	return (1 / (2 * M_PI * s_x * s_y)) * exp(-((pow(x_value - mu_x, 2) / (2 * pow(s_x, 2))) + (pow(y_value - mu_y, 2) / (2 * pow(s_y, 2)))));
}
```

### 2. Implementation in C/C++

The the beginning the map data is loaded in the `main.cpp` function. The `main.cpp` function connects to the simulator via [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) and receives the current observations in each step. In each step it determines the best particle by picking the one with the highest weight. It then sends the best particle's position with attached observations and associated landmark ids back to the simulator for visualization and error calculation.

The flow of the particle filter in each step is defined by the following sequence of methods:

```C
ParticleFilter pf;
...
*in each steps do* {
	if (!pf.initialized()) {
		pf.init(...);
	}
	else {
		pf.prediction(...);
	}
	pf.UpdateWeights(...);
	pf.resample(...);
}
```

In the first step the `pf.init()` method initializes all particles with a random distribution around a first estimate for the location. The random noise is added to each particle's location with the method `addNoise()`.

The `pf.prediction()` method uses the above described linear bicycle motion model to calculate the location of all particles after the duration `delta_t`. It also adds random noise to each particle's location with the method `addNoise()`. This ensures that we consider new and potentially better particle locations in each step.

The `pf.UpdateWeights()` method executes a sequence of steps for all particles to update their weights based on the new observations (i.e. measurements). First it determines the map landmarks within the sensor range of the particle's location. Then it uses the above mentioned equations to transform the vehicle observations into global map coordinates for the particle. After this it uses the nearest neighbor method to associate the observations in global map coordinates to the map landmarks within the sensor range. And finally it calculates the new weight for the particle as multiplication of the probabilities that each observation is close to the associated landmark. The multi-variate Gaussian distribution function from above is used to determine the probability that the observation in global map coordinates is equal to the associated landmark.

The `pf.resample()` method is then used to draw a new list of particles from the existing list of particples. The weight of each particle is used as probability during this drawing process. This ensures that more likely particle locations get drawn more often into the new list of particles. The new list of particles contains the same number of particles as before.

### 3. Debugging environment

In order to debug the particle filter efficiently, several methods have been added to display the content of all the input and output variables of each relevant function.

The debug options of the `main()` function are selected with the below parameters. If `bFILEOUTPUT` is `true`, the standard output is redirected into the file `out.txt`. If `bDISPLAY` is `true`, more information about input and output variables is displayed to the standard output. If `bTest` is `true`, a predefined set of inputs is used to run and check the particle filter instead of connecting it to the simulator environment.

```C
const bool bFILEOUTPUT = false;
const bool bDISPLAY = false;
const bool bTEST = false;
```

Inside the particle filter object, all methods can display their input and output variables to the standard output. The parameter `bDISPLAY` must be `true` to enable any of these features. The debugging feature of each particle filter method can be turned on by setting the parameter `bDISPLAY_<name of method>` to `true`. The available method names are listed below.

```C
const bool bDISPLAY = false;
const bool bDISPLAY_init = true;
const bool bDISPLAY_prediction = true;
const bool bDISPLAY_dataAssociation = false;
const bool bDISPLAY_updateWeights = true;
const bool bDISPLAY_resample = true;
const bool bDISPLAY_SetAssociations = false;
const bool bDISPLAY_addNoise = false;
const bool bDISPLAY_getMapLandmark = false;
const bool bDISPLAY_transformVehicle2Map = false;
```

The following methods are used to convert the variable contents into a single string that can be displayed:

```C
std::string createDoubleVectorString(std::vector<double> double_vector);
std::string createIntegerVectorString(std::vector<int> int_vector);
std::string createArrayString(double array[], unsigned int num_elements);
std::string createParticleString(Particle particle);
std::string createParticlesString(std::vector<Particle> particles);
std::string createLandmarkString(LandmarkObs landmark);
std::string createLandmarksString(std::vector<LandmarkObs> landmarks);
std::string createMapString(Map map);
```

## 4. Execution with given input data

### 1. Commands to start the simulation

The program is compiled using the `.\build.sh` command. After this it can be started using the `.\run.sh' command. Once the program is running and listening on port 4567 the simulator can be started.

### 2. Simulation results

In the simulator the blue dots are LIDAR measurements and the red dots are RADAR measurements. The sensor fusion results are shown as green dots and are clearly closer to the ground truth values than the individual measurements for LIDAR or RADAR (see image below).

![alt text][image1]
![alt text][image2]
![alt text][image3]

<img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_20 0.png" width="30%"> <img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_20 168.png" width="30%"> <img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_20 207.png" width="30%">

<img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_20 264.png" width="30%"> <img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_20 289.png" width="30%"> <img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_20 341.png" width="30%">

<img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_20 362.png" width="30%"> <img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_20 434.png" width="30%"> <img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_20 639.png" width="30%">

The debugging output of this example can be found in [./build/out.txt](./build/out.txt).

The final RMSE values are `[0.0973, 0.0855, 0.4513, 0.4399]` and therefore well below the maximum allowed error values `[0.11, 0.11, 0.52, 0.52]`.

## 5. Discussion

The main challenge of this project is the setup of a working tool chain when you use a Windows computer.

<img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_5 9.png" width="30%"> <img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_5 10.png" width="30%"> <img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_5 11.png" width="30%">

<img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_5 58.png" width="30%"> <img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_5 59.png" width="30%"> <img src="docu_images/181221_StAn_Udacity_SDC_Kidnapped_Vehicle_Project_5 60.png" width="30%">

Also, the provided templates used a mixture of different programming styles and a lot of inline variable definitions. For consistency and easier debugging I did some minor clean up.

As `C/C++` requires to compile the code prior to execution, I added a lot of debugging messages that are routed via the standard output `cout` to the file [./build/out.txt](./build/out.txt). In lack of an [Integrated Development Environment (IDE)](https://en.wikipedia.org/wiki/Integrated_development_environment) this helped a lot to quickly get a working program. It will also help if somebody wants to make additional changes to this code in the future. The debugging can be turned off by setting all `bDISPLAY` variables to `false`.





























# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

#### Submission
All you will submit is your completed version of `particle_filter.cpp`, which is located in the `src` directory. You should probably do a `git pull` before submitting to verify that your project passes the most up-to-date version of the grading code (there are some parameters in `src/main.cpp` which govern the requirements on accuracy and run time.)

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

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

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

# Implementing the Particle Filter
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

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Success Criteria
If your particle filter passes the current grading code in the simulator (you can make sure you have the current version at any time by doing a `git pull`), then you should pass!

The things the grading code is looking for are:


1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
