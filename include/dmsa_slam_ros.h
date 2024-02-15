/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include "DMSA/DmsaSlam.h"
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>

// Function to split the input string into a vector of strings
inline std::vector<std::string> splitIntoWords(const std::string &str)
{
  std::istringstream iss(str);
  std::vector<std::string> words;
  std::string word;

  while (iss >> word)
  {
    words.push_back(word);
  }

  return words;
}

class dmsa_slam_ros
{

public:
  dmsa_slam_ros();
  ~dmsa_slam_ros();
  void spin();

private:
  ros::NodeHandle nh_;

  DmsaSlam DmsaSLAMObj;

  bool saveMap = true;

  std::vector<std::string> bagnames;

  std::string lidarSubTopicName, imuSubTopicName;

  // config
  Config config;
  std::string result_dir;

  void callbackImuData(const sensor_msgs::Imu::ConstPtr &msg);
  void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);

  void publishPointCloudsAndPose();

  ros::Publisher pubMap;
  ros::Publisher pubSubmap;
  ros::Publisher pubPose;
  ros::Publisher pubTraj;

  Eigen::Matrix4f lidar2imu;
  Eigen::Matrix4f imu2lidar;

  Eigen::Matrix3d cov_gyr;
  Eigen::Matrix3d cov_acc;

  Eigen::Vector3d lastKeyPos;
  Eigen::Vector3d lastKeyAxang;

  Eigen::Matrix3Xd ActivePoints;
  Eigen::Vector3d lastPosUpdateActivePoints;


  // config
  double lastPcMsgStamp = -1.0;
  double timeshift_to_imu0;
};
