<span style="color:red">**! THE CODE WILL BE UPLOADED SOON !**</span>



# DMSA_LiDAR_SLAM
DMSA LiDAR SLAM is a robust and accurate package for LiDAR and IMU based mapping. First point clouds within a sliding time window are optimized conjointly with so called static points from the map and IMU measurements. When a new keyframe is added to the map, related keyframes are optimized. Keyframes are stored in a ring buffer, therefore old keyframes are deleted from a certain map size.
For point cloud alignment, **Dense Multi Scan Adjustment** (DMSA) is used.

<p align='left'>
    <img src="./doc/stairs.gif" alt="drawing" width="600"/>
</p>

## Preface
The package is primarily designed as an offline mapping module and the default parameters are optimized towards robustness and accuracy. The processing speed is highly dependent on the hardware used and the data acquisition environment. Typical processing times are 2-3 times the recording time.

**Input:** Rosbag with LiDAR (PointCloud2-Messages) and IMU data

**Output:** Trajectory as Pose.txt file in (TUM-format) and resulting keyframe point cloud

The package can work with LiDAR data from the following manufacturers:
- Ouster
- Hesai
- Velodyne
- Robosense

**! The ROS drivers must be configured in such a way that time stamp and ring id are provided for each point !**

## Prerequisites
The package was developed with the following packages:

- Ubuntu 20.04
- ROS Noetic
- Eigen 3.4.0

Compatibility with other Ubuntu/ROS/Eigen versions should be possible, but has not been tested.

## Installation

### 1. Create workspace:
`mkdir catkin_ws`
### 2. Create source folder:
`cd catkin_ws && mkdir src && cd src`
### 3. Clone repository:
`git clone https://github.com/davidskdds/DMSA_LiDAR_SLAM.git`
### 4. Compile
`cd .. && catkin_make`

## Setup for Hilti dataset

## Setup for Newer College dataset 

## Setup for own recordings:
### 1. Setup rosbag directory in config
### 2. Setup topic names
### 3. Setup sensor type
### 4. Setup IMU frame to LiDAR frame transform
### 5. Setup output files directory
