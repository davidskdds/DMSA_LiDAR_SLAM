<span style="color:red">**! THE CODE WILL BE UPLOADED SOON !**</span>



# DMSA SLAM
DMSA LiDAR SLAM is a robust and accurate package for LiDAR and IMU based mapping. First point clouds within a sliding time window are optimized conjointly with so called static points from the map and IMU measurements. When a new keyframe is added to the map, related keyframes are optimized. Keyframes are stored in a ring buffer, therefore old keyframes are deleted from a certain map size.
For point cloud alignment, **Dense Multi Scan Adjustment** (DMSA) is used.

<figure>
<img src="./doc/basement_bv.png" alt="drawing" width="600"/>
<figcaption>Fig.1 - Resulting keyframe point cloud of sequence exp14 Basement of the Hilti-Oxford Dataset. The estimated trajectory is marked with a red line.</figcaption>
</figure>

<figure>
<p align='left'>
    <img src="./doc/stairs.gif" alt="drawing" width="600"/>
</p>
<figcaption>Fig.2 - Running DMSA SLAM on the stairs sequence of the Newer College Dataset.</figcaption>
</figure>

## Preface
The package is primarily designed as an offline mapping module and the default parameters are optimized towards robustness and accuracy. The processing speed is highly dependent on the hardware used and the data acquisition environment. Typical processing times are 2-3 times the recording time.

**Note:** In order to publish the software under the MIT license, a different type of interpolation was used to calculate the dense trajectory poses, in contrast to the published paper. While [Cubic Hermitian spline interpolation](https://github.com/ttk592/spline) was used for the results published in the paper, the software published here uses [Barycentric Rational interpolation](https://live.boost.org/doc/libs/1_72_0/libs/math/doc/html/math_toolkit/barycentric.html). This change and the continuous development of the software may lead to minor deviations in accuracy compared to the published results.

**Input:** Rosbag with LiDAR (PointCloud2-Messages) and IMU data

**Output:** Trajectory as Pose.txt file (TUM-format) and resulting keyframe point cloud as .pcd file

The package can work with LiDAR data from the following manufacturers (the ROS drivers must be configured in such a way that time stamp and ring id are provided for each point):
- Ouster
- Hesai
- Velodyne
- Robosense

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
