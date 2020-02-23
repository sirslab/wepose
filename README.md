# WePosE - a Wearable Pose Estimation system

This repository contains the paper and the source code under BY-NC-SA Creative Commons Licence.

## Description
Estimating the limbs pose in a wearable way
may benefit multiple areas such as rehabilitation, teleoperation, human-robot interaction, gaming, and many more. Several solutions are commercially available, but they are usually expensive or not wearable/portable. We present a wearable pose estimation system (WePosE), based on inertial measurements units (IMUs), for motion analysis and body tracking. Differently from camerabased approaches, the proposed system does not suffer from occlusion problems and lighting conditions, it is cost effective and it can be used in indoor and outdoor environments. Moreover, since only accelerometers and gyroscopes are used to estimate the orientation, the system can be used also in the presence of iron and magnetic disturbances. An experimental validation using a high precision optical tracker has been performed 

## Citation
```latex
@article{lisiniFarina2019,
   Author = {Lisini Baldi, T. and Farina, F. and Garulli, A. and Giannitrapani, A, and Prattichizzo, D.},
   Title = {{Upper Body Pose Estimation Using Wearable Inertial Sensors and Multiplicative Kalman Filter}},
   Journal = {IEEE Sensors Journal},
   Year = {2019},
   doi = {https://doi.org/10.1109/JSEN.2019.2940612},
}
```



## Installation and Usage

## MATLAB
### Requirements
* MATLAB 2014 or later

### Usage
1. Run mekf.m from .\matlab\

## CPP library
### Windows
#### Requirements
* Windows
* Visual Studio 2017
#### Usage
1. Open mekf.sln from .\cpp\
2. Build the project in release mode

### Unix
#### Usage
1. Compile cpp/mekf/example.cpp

## Demo
### Requirements
* XSens device (one or more)
* Windows 7, 8, 8.1, 10

### Binary execution

1. Install the Xsens Driver
2. Check the COM port and the baudrate
3. Run demo/bin/Win32/wepose.exe

### From source

#### Xsens Device
1. Install Xsens Driver
2. Check the COM port and the 
3. Open the Visual Studio Solution and check the included and linked folders
4. Build the project in release mode


