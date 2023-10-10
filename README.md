# Lesson 1: Introduction to Lidar and Point Clouds
## LIDAR (Light Detecting and Ranging) 

Lidar sensing involves the emission of thousands of infrared laser signals at various angles, covering a 360-degree range. These signals bounce off objects, returning to the sensor. By measuring the time it takes for the signals to return, Lidar calculates the distance to these objects with high precision. Additionally, the intensity of the reflected signals can be measured to gain insights into the material properties of the objects. While Lidar provides accurate 3D models of the surrounding environment, it is currently quite expensive, often exceeding $60,000 for a standard unit.

## HDL 64 Lidar Specifications

<div style="text-align:center;">
    <img src="media/lidar-velodyne.png" width="700" height="400" />
</div>

Velodyne lidar sensors, with HDL 64, HDL 32, VLP 16 from left to right. The larger the sensor, the higher the resolution.

- **Number of Layers:** 64 layers, each sent out at a different angle from the z-axis, providing varying inclines for comprehensive coverage.

- **Angular Resolution:** Each layer has an angular resolution of 0.08 degrees, ensuring fine detail in the scanned data.

- **Scanning Rate:** On average, the Lidar scans ten times a second, rapidly capturing data points for real-time analysis.

- **Maximum Range for Cars and Foliage:** It can detect objects such as cars and foliage at distances of up to 120 meters, making it suitable for long-range applications.

- **Pavement Detection Range:** The Lidar is capable of sensing pavement surfaces up to a range of 50 meters, which is valuable for navigation and road analysis.

These specifications highlight the Lidar's ability to provide high-resolution, multi-layered data with impressive range and precision, making it suitable for various applications in areas like autonomous vehicles, mapping, and environmental monitoring.

<div style="text-align:center;">
    <img src="media/hdl-64e.png" width="700" height="400" />
</div>

<div style="text-align:center;">
    <img src="media/vlp-sensor-specs.png" width="700" height="400" />
</div>

## Point Cloud

Lidar data is typically stored in a format called Point Cloud Data (PCD). A .pcd file represents a single snapshot of the environment, containing (x, y, z) Cartesian coordinates along with intensity values (i). For example, with a VLP 64 Lidar, a single .pcd file would contain approximately 256,000 (x, y, z, i) data points, capturing the spatial information and intensity data from a single scan.
The coordinate system used for point cloud data aligns with the car's local coordinate system. In this system:

- The x-axis points towards the front of the car.
- The y-axis points to the left of the car.
- The z-axis points up above the car.

This coordinate system follows the right-handed convention, where you can use your right hand to visualize the positive direction of each axis.

<div style="text-align:center;">
    <img src="media/pcd-coordinates.png" width="800" height="400" />
</div>

## Point Cloud Library

<div style="text-align:center;">
    <img src="media/pcl-logo.png" width="400" height="200" />
</div>

In this module, the focus is on processing point cloud data within a C++ environment. Prior familiarity with C++ can be advantageous for participants. The primary tool used for this purpose is the Point Cloud Library (PCL), an open-source C++ library specifically designed for manipulating point cloud data. PCL will be utilized for tasks such as data visualization, rendering shapes, and leveraging its built-in processing functions.
The robotics community widely adopts PCL for handling point cloud data, and there are ample online tutorials available for guidance. The module will utilize various PCL functions, including Segmentation, Extraction, and Clustering, to identify obstacles within point cloud data.

## Excersise 1: Lidar object in simulation


# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resolution imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.

## Classroom Workspace

The workspace provided in the SFND classroom comes preinstallated with everything that you need to finish the exercises and projects. Versions used by Udacity for this ND are as follows:

* Ubuntu 16.04
* PCL - v1.7.2
* C++ v11
* gcc v5.5

**Note** The [[CMakeLists.txt](https://github.com/udacity/SFND_Lidar_Obstacle_Detection/blob/master/CMakeLists.txt)] file provided in this repo can be used locally if you have the same package versions as mentioned above. If you want to run this project locally (outside the Udacity workspace), please follow the steps under the **Local Installation** section.


## Local Installation

### Ubuntu 

1. Clone this github repo:

   ```sh
   cd ~
   git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
   ```

2.  Edit [CMakeLists.txt](https://github.com/udacity/SFND_Lidar_Obstacle_Detection/blob/master/CMakeLists.txt) as follows:

   ```cmake
   cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
   
   add_definitions(-std=c++14)
   
   set(CXX_FLAGS "-Wall")
   set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
   
   project(playback)
   
   find_package(PCL 1.11 REQUIRED)
   
   include_directories(${PCL_INCLUDE_DIRS})
   link_directories(${PCL_LIBRARY_DIRS})
   add_definitions(${PCL_DEFINITIONS})
   list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
   
   
   add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
   target_link_libraries (environment ${PCL_LIBRARIES})
   ```

3. Execute the following commands in a terminal

   ```shell
   sudo apt install libpcl-dev
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```

   This should install the latest version of PCL. You should be able to do all the classroom exercises and project with this setup.
   
**Note** The library version of PCL being distributed by the apt repository for 18.04 and 20.04 are both older than v1.11. The following links have the information regarding the versions-

[Bionic 18.04](https://www.ubuntuupdates.org/package/core/bionic/universe/updates/libpcl-dev)
[Focal 20.04](https://www.ubuntuupdates.org/package/core/focal/universe/base/libpcl-dev)

You can either build PCL from source (for v1.11) or use the older version.

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

6. Clone this github repo

   ```shell
   cd ~
   git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
   ```

7. Edit the CMakeLists.txt file as shown in Step 2 of Ubuntu installation instructions above.

8. Execute the following commands in a terminal

   ```shell
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```
If you get build errors related to Qt5, make sure that the path for Qt5 is correctly set in .bash_profile or .zsh_profile (Refer [#45](https://github.com/udacity/SFND_Lidar_Obstacle_Detection/issues/45))

### WINDOWS

#### Install via cvpkg

1. Follow the steps [here](https://pointclouds.org/downloads/) to install PCL.

2. Clone this github repo

   ```shell
   cd ~
   git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
   ```

3. Edit the CMakeLists.txt file as shown in Step 2 of Ubuntu installation instructions above.

4. Execute the following commands in Powershell or Terminal

   ```shell
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_macosx.html#compiling-pcl-macosx)
