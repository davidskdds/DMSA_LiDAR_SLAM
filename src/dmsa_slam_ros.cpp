/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include "dmsa_slam_ros.h"
#include <ros/console.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace Eigen;
using namespace pcl;


dmsa_slam_ros::dmsa_slam_ros()
{
    ros::NodeHandle nh("~");

    // load parameters
    std::cout << "Current parameter config:\n";

    nh.getParam("lidar_topic", lidarSubTopicName);
    nh.getParam("imu_topic", imuSubTopicName);
    std::cout << "lidar_topic: " << lidarSubTopicName << std::endl;
    std::cout << "imu_topic: " << imuSubTopicName << std::endl;

    nh.getParam("result_dir", result_dir);
    std::cout << "result_dir: " << result_dir << std::endl;

    nh.getParam("max_num_points_per_scan", config.max_num_points_per_scan);
    std::cout << "max_num_points_per_scan: " << config.max_num_points_per_scan << std::endl;

    nh.getParam("decay_rate_sw", config.decay_rate_sw);
    std::cout << "decay_rate_sw: " << config.decay_rate_sw << std::endl;

    nh.getParam("decay_rate_key", config.decay_rate_key);
    std::cout << "decay_rate_key: " << config.decay_rate_key << std::endl;

    nh.getParam("min_distance_ds", config.minDistDS);
    std::cout << "min_distance_ds: " << config.minDistDS << std::endl;

    nh.getParam("min_dist", config.min_dist);
    std::cout << "min_dist: " << config.min_dist << std::endl;

    nh.getParam("select_best_set_key", config.select_best_set_key);
    std::cout << "select_best_set_key: " << config.select_best_set_key << std::endl;

    nh.getParam("min_num_points_gauss_key", config.min_num_points_gauss_key);
    std::cout << "min_num_points_gauss_key: " << config.min_num_points_gauss_key << std::endl;

    nh.getParam("min_num_points_gauss", config.min_num_points_gauss);
    std::cout << "min_num_points_gauss: " << config.min_num_points_gauss << std::endl;

    std::string bag_dirs;
    nh.getParam("bag_dirs", bag_dirs);
    std::cout << "bag_dirs: " << bag_dirs << std::endl;

    bagnames = splitIntoWords(bag_dirs);

    // init transform in imu frame
    imu2lidar = Matrix4f::Identity();

    Eigen::Quaternionf q;
    nh.getParam("q_x", q.x());
    nh.getParam("q_y", q.y());
    nh.getParam("q_z", q.z());
    nh.getParam("q_w", q.w());
    std::cout << "Quaternion from imu frame: " << q << std::endl;

    imu2lidar.block(0, 0, 3, 3) = q.normalized().toRotationMatrix();

    nh.getParam("t_x", imu2lidar(0, 3));
    nh.getParam("t_y", imu2lidar(1, 3));
    nh.getParam("t_z", imu2lidar(2, 3));

    std::cout << "Translation from imu frame: " << imu2lidar.block(0, 3, 3, 1) << std::endl;
    lidar2imu = imu2lidar.inverse();

    nh.getParam("use_gravity_term_in_keyframe_opt", config.use_gravity_term_in_keyframe_opt);
    std::cout << "use_gravity_term_in_keyframe_opt: " << config.use_gravity_term_in_keyframe_opt << std::endl;

    nh.getParam("use_odometry_term_in_keyframe_opt", config.use_odometry_term_in_keyframe_opt);
    std::cout << "use_odometry_term_in_keyframe_opt: " << config.use_odometry_term_in_keyframe_opt << std::endl;

    nh.getParam("balancing_factor_gravity", config.balancing_factor_gravity);
    std::cout << "balancing_factor_gravity: " << config.balancing_factor_gravity << std::endl;

    nh.getParam("balancing_factor_odometry", config.balancing_factor_odometry);
    std::cout << "balancing_factor_odometry: " << config.balancing_factor_odometry << std::endl;

    nh.getParam("timeshift_lidar2imu", timeshift_to_imu0);
    std::cout << "timeshift_lidar2imu: " << timeshift_to_imu0 << std::endl;

    nh.getParam("optimize_sliding_window_keyframes", config.optimize_sliding_window_keyframes);
    std::cout << "optimize_sliding_window_keyframes: " << config.optimize_sliding_window_keyframes << std::endl;

    nh.getParam("num_clouds_submap", config.n_clouds);
    std::cout << "num_clouds_submap: " << config.n_clouds << std::endl;

    nh.getParam("num_control_poses", config.num_control_poses);
    std::cout << "num_control_poses: " << config.num_control_poses << std::endl;

    nh.getParam("last_n_keyframes_for_optim", config.last_n_keyframes_for_optim);
    std::cout << "last_n_keyframes_for_optim: " << config.last_n_keyframes_for_optim << std::endl;

    nh.getParam("oldest_k_keyframes_as_static_points", config.oldest_k_keyframes_as_static_points);
    std::cout << "oldest_k_keyframes_as_static_points: " << config.oldest_k_keyframes_as_static_points << std::endl;

    nh.getParam("alpha_keyframe_optim", config.alpha_keyframe_optim);
    std::cout << "alpha_keyframe_optim: " << config.alpha_keyframe_optim << std::endl;

    nh.getParam("num_iter_keyframe_optim", config.num_iter_keyframe_optim);
    std::cout << "num_iter_keyframe_optim: " << config.num_iter_keyframe_optim << std::endl;

    nh.getParam("imu_factor_weight_submap", config.imu_factor_weight_submap);
    std::cout << "imu_factor_weight_submap: " << config.imu_factor_weight_submap << std::endl;

    nh.getParam("epsilon_keyframe_opt", config.epsilon_keyframe_opt);
    std::cout << "epsilon_keyframe_opt: " << config.epsilon_keyframe_opt << std::endl;

    nh.getParam("dist_new_keyframe", config.dist_new_keyframe);
    std::cout << "dist_new_keyframe: " << config.dist_new_keyframe << std::endl;

    nh.getParam("min_overlap_new_keyframe", config.min_overlap_new_keyframe);
    std::cout << "min_overlap_new_keyframe: " << config.min_overlap_new_keyframe << std::endl;

    nh.getParam("use_imu", config.use_imu);
    std::cout << "use_imu: " << config.use_imu << std::endl;

    nh.getParam("num_iter_sliding_window_optim", config.num_iter_sliding_window_optim);
    std::cout << "num_iter_sliding_window_optim: " << config.num_iter_sliding_window_optim << std::endl;

    nh.getParam("alpha_sliding_window_imu", config.alpha_sliding_window_imu);
    std::cout << "alpha_sliding_window_imu: " << config.alpha_sliding_window_imu << std::endl;

    nh.getParam("alpha_sliding_window_no_imu", config.alpha_sliding_window_no_imu);
    std::cout << "alpha_sliding_window_no_imu: " << config.alpha_sliding_window_no_imu << std::endl;

    nh.getParam("dist_static_points_keyframe", config.dist_static_points_keyframe);
    std::cout << "dist_static_points_keyframe: " << config.dist_static_points_keyframe << std::endl;

    nh.getParam("gravity_outlier_thresh", config.gravity_outlier_thresh);
    std::cout << "gravity_outlier_thresh: " << config.gravity_outlier_thresh << std::endl;

    bool acceleration_in_g = false;
    nh.getParam("acceleration_in_g", acceleration_in_g);
    std::cout << "acceleration_in_g: " << acceleration_in_g << std::endl;

    double sigma_gyr;

    nh.getParam("sigma_gyr", sigma_gyr);
    std::cout << "sigma_gyr: " << sigma_gyr << std::endl;

    config.cov_gyr = sigma_gyr * sigma_gyr * Matrix3d::Identity();

    double sigma_acc;
    nh.getParam("sigma_acc", sigma_acc);
    std::cout << "sigma_acc: " << sigma_acc << std::endl;

    config.cov_acc = sigma_acc * sigma_acc * Matrix3d::Identity();

    nh.getParam("sensor", config.sensor);
    std::cout << "Sensor: " << config.sensor << std::endl;

    // DEBUG
    if (false)
    {
        std::string bag_dirs2 = "/home/david/Rosbags/Hilti/Benchmark_seq/exp03_construction_stairs.bag";

        saveMap = false;

        bagnames = splitIntoWords(bag_dirs2);

        Eigen::Quaternionf q;

        q.x() = 0.7094397486399825;
        q.y() = -0.7047651311547696;
        q.z() = 0.001135774698382635;
        q.w() = -0.0002509459564800096;

        imu2lidar.block(0, 0, 3, 3) = q.normalized().toRotationMatrix();
        imu2lidar(0, 3) = 0.0;
        imu2lidar(1, 3) = 0.0;
        imu2lidar(2, 3) = 0.055;
        lidar2imu = imu2lidar.inverse();

        timeshift_to_imu0 = 0.0;

        imuSubTopicName = "/alphasense/imu";
        lidarSubTopicName = "/hesai/pandar";

        config.cov_gyr = 1e-4 * Matrix3d::Identity();
        config.cov_acc = 1e-4 * Matrix3d::Identity();
        config.n_clouds = 10;
        config.sensor = "hesai";
        config.last_n_keyframes_for_optim = 5;
        config.num_iter_keyframe_optim = 1;
        config.num_iter_sliding_window_optim = 5;
        config.num_control_poses = 10;
        config.use_imu = true;
        config.max_num_points_per_scan = 1000;
        config.minDistDS = 10.0;
        config.dist_new_keyframe = 0.3;
        config.min_overlap_new_keyframe = 0.95;
        config.alpha_keyframe_optim = 0.0000;

        sigma_acc = 0.2;
        sigma_gyr = 0.05;

        result_dir = "/home/david/optim";

        cov_gyr = Eigen::Matrix3d::Identity() * std::pow(sigma_gyr, 2);
        cov_acc = Eigen::Matrix3d::Identity() * std::pow(sigma_acc, 2);
    }

    DmsaSLAMObj.config.lidarToImuTform = lidar2imu;
    config.lidarToImuTform = lidar2imu;

    pubMap = nh.advertise<sensor_msgs::PointCloud2>("/dmsa_slam/map", 1);
    pubSubmap = nh.advertise<sensor_msgs::PointCloud2>("/dmsa_slam/submap", 1);
    pubPose = nh.advertise<geometry_msgs::PoseStamped>("/dmsa_slam/pose", 1);
    pubTraj = nh.advertise<sensor_msgs::PointCloud2>("/dmsa_slam/traj", 1);

    // livox imu acceleration scaling
    if (acceleration_in_g)
    {
        accUnitScale = 9.81;
    }

    DmsaSLAMObj = DmsaSlam(config);
}

dmsa_slam_ros::~dmsa_slam_ros()
{
}

void dmsa_slam_ros::spin()
{
    // Outputting the words
    std::cout << "You entered the following rosbags:" << std::endl;
    for (const auto &rosbagdir : bagnames)
    {
        std::cout << rosbagdir << std::endl;
    };

    std::vector<std::string> topics;

    topics.push_back(lidarSubTopicName);
    topics.push_back(imuSubTopicName);

    for (auto rosbagDir : bagnames)
    {
        rosbag::Bag bag;

        try
        {
            bag.open(rosbagDir);
        }
        catch (...)
        {
            std::cerr << "Rosbag directory (" << rosbagDir << ") is invalid, processing is aborted\n";
            return;
        }

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for (rosbag::MessageInstance const m : view)
        {
            sensor_msgs::PointCloud2::ConstPtr pc2Ptr = m.instantiate<sensor_msgs::PointCloud2>();

            if (pc2Ptr != nullptr)
                callbackPointCloud(pc2Ptr);

            sensor_msgs::Imu::ConstPtr imuDataPtr = m.instantiate<sensor_msgs::Imu>();

            if (imuDataPtr != nullptr)
                callbackImuData(imuDataPtr);
        }

        bag.close();
    }

    // Save the cloud to a .pcd file
    std::string filename = result_dir + "/PointCloud.pcd";
    if (io::savePCDFileASCII(filename, DmsaSLAMObj.KeyframeMap.globalPoints) == -1)
    {
        PCL_ERROR("Failed to save PCD file\n");
    }

    // save poses
    DmsaSLAMObj.savePoses(result_dir);

    std::cout << "Processing of rosbags/s finished . . . " << std::endl;

    ros::Rate rate(1000);

    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }

}

void dmsa_slam_ros::callbackImuData(const sensor_msgs::Imu::ConstPtr &msg)
{

    // save measurements
    Eigen::Vector3d accVec = accUnitScale*Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d angVelVec = Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    double stamp = msg->header.stamp.toSec();


    /* ADD IMU MEASUREMENTS TO DMSA SLAM */
    DmsaSLAMObj.processImuMeasurements(accVec, angVelVec, stamp);
}

void dmsa_slam_ros::publishPointCloudsAndPose()
{
    if (DmsaSLAMObj.KeyframeMap.keyframeDataBuffer.getNumElements() <= 0) return;

    sensor_msgs::PointCloud2 mapMsg,submapMsg, trajMsg;

    PointCloud<PointXYZ> currPosition;

    currPosition.points.resize(1);
    currPosition.points[0].x = DmsaSLAMObj.currTraj->controlPoses.globalPoses.Translations(0,0);
    currPosition.points[0].y = DmsaSLAMObj.currTraj->controlPoses.globalPoses.Translations(1,0);
    currPosition.points[0].z = DmsaSLAMObj.currTraj->controlPoses.globalPoses.Translations(2,0);

    pcl::toROSMsg(DmsaSLAMObj.currTraj->globalPoints, submapMsg);
    pcl::toROSMsg(currPosition, trajMsg);

    submapMsg.header.frame_id = "map";
    trajMsg.header.frame_id = "map";

    pubSubmap.publish(submapMsg);
    pubTraj.publish(trajMsg);

    geometry_msgs::PoseStamped currPose;

    currPose.pose.position.x = DmsaSLAMObj.currTraj->controlPoses.globalPoses.Translations(0,0);
    currPose.pose.position.y = DmsaSLAMObj.currTraj->controlPoses.globalPoses.Translations(1,0);
    currPose.pose.position.z = DmsaSLAMObj.currTraj->controlPoses.globalPoses.Translations(2,0);

    Matrix3d R = axang2rotm(DmsaSLAMObj.currTraj->controlPoses.globalPoses.Orientations.col(0));
    Quaterniond q(R);

    currPose.pose.orientation.x = q.x();
    currPose.pose.orientation.y = q.y();
    currPose.pose.orientation.z = q.z();
    currPose.pose.orientation.w = q.w();

    currPose.header.frame_id = "map";
    pubPose.publish(currPose);

    // publish map not every cycle
    if (DmsaSLAMObj.pcBuffer->getNumUpdates() % 20 == 0)
    {
        pcl::toROSMsg(DmsaSLAMObj.KeyframeMap.globalPoints, mapMsg);

        mapMsg.header.frame_id = "map";

        pubMap.publish(mapMsg);
    }

    ros::spinOnce();
}

void dmsa_slam_ros::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    PointCloudPlus::Ptr newPC(new PointCloudPlus);

    newPC->resize(msg->height * msg->width);

    int arrayPosition;
    uint8_t ring_tmp8;
    uint16_t ring_tmp;
    uint32_t relStampNano;
    double stampMsg = msg->header.stamp.toSec();

    // only relevant for unknown sensor type
    if (config.sensor == "unknown" && lastPcMsgStamp < 0.0)
    {
        lastPcMsgStamp = stampMsg;
        return;
    }

    double deltaTPcs = stampMsg - lastPcMsgStamp;

    float tmpStampFloat;
    double tmpStampDouble;

    for (uint k = 0; k < msg->height * msg->width; ++k)
    {
        arrayPosition = k * msg->point_step;

        newPC->at(k).isStatic = 0;

        // xyz
        memcpy(&newPC->at(k).x, &msg->data[arrayPosition + msg->fields[0].offset], sizeof(float));
        memcpy(&newPC->at(k).y, &msg->data[arrayPosition + msg->fields[1].offset], sizeof(float));
        memcpy(&newPC->at(k).z, &msg->data[arrayPosition + msg->fields[2].offset], sizeof(float));

        if (config.sensor == "hesai")
        {
            // stamp and ring
            memcpy(&tmpStampDouble, &msg->data[arrayPosition + msg->fields[4].offset], sizeof(double));
            memcpy(&ring_tmp, &msg->data[arrayPosition + msg->fields[5].offset], sizeof(uint16_t));

            newPC->at(k).stamp = tmpStampDouble;
            newPC->at(k).id = (int)ring_tmp;
        }
        else if (config.sensor == "ouster")
        {
            // stamp and ring
            memcpy(&relStampNano, &msg->data[arrayPosition + msg->fields[4].offset], sizeof(uint32_t));
            memcpy(&ring_tmp8, &msg->data[arrayPosition + msg->fields[6].offset], sizeof(uint8_t));

            tmpStampDouble = stampMsg + 1e-9 * (double)relStampNano;

            newPC->at(k).stamp = tmpStampDouble;
            newPC->at(k).id = (int)ring_tmp8;
        }
        else if (config.sensor == "robosense")
        {
            // stamp and ring
            memcpy(&tmpStampDouble, &msg->data[arrayPosition + msg->fields[5].offset], sizeof(double));
            memcpy(&ring_tmp, &msg->data[arrayPosition + msg->fields[4].offset], sizeof(uint16_t));

            newPC->at(k).stamp = tmpStampDouble;
            newPC->at(k).id = (int)ring_tmp;
        }
        else if (config.sensor == "velodyne")
        {
            // stamp and ring
            memcpy(&tmpStampFloat, &msg->data[arrayPosition + msg->fields[5].offset], sizeof(float));
            memcpy(&ring_tmp, &msg->data[arrayPosition + msg->fields[4].offset], sizeof(uint16_t));

            newPC->at(k).stamp = stampMsg + static_cast<double>(tmpStampFloat);
            newPC->at(k).id = (int)ring_tmp;
        }
        else if (config.sensor == "livoxXYZRTLT_s")
        {
            // stamp and ring
            memcpy(&tmpStampDouble, &msg->data[arrayPosition + msg->fields[6].offset], sizeof(double));

            newPC->at(k).stamp = tmpStampDouble;

            // add artificial ring index
            newPC->at(k).id = k % 1000;
        }
        else if (config.sensor == "livoxXYZRTLT_ns")
        {
            // stamp and ring
            memcpy(&tmpStampDouble, &msg->data[arrayPosition + msg->fields[6].offset], sizeof(double));

            // multiply point stamp with 1e-9 to recieve correct stamp - this is a workaround since there is a bug in the livox2 driver
            newPC->at(k).stamp = 1e-9 * tmpStampDouble;

            // add artificial ring index
            newPC->at(k).id = k % 1000;
        }
        else if (config.sensor == "unknown")
        {
            // use heuristic stamp
            newPC->at(k).stamp = stampMsg + deltaTPcs * (double)k / (double)(msg->height * msg->width);

            // add artificial ring index
            newPC->at(k).id = k % 1000;
        }
    }

    /* PROCESS POINT CLOUD IN DMSA SLAM */
    DmsaSLAMObj.processPointCloud(newPC);

    // publish in ros
    publishPointCloudsAndPose();

    // save data in directory
    if (saveMap && DmsaSLAMObj.pcBuffer->getNumUpdates() % 20 == 0 && DmsaSLAMObj.KeyframeMap.keyframeDataBuffer.getNumUpdates() > 0)
    {
        // save poses
        DmsaSLAMObj.savePoses(result_dir);

        // Save the cloud to a .pcd file
        std::string filename = result_dir + "/PointCloud.pcd";
        if (io::savePCDFileASCII(filename, DmsaSLAMObj.KeyframeMap.globalPoints) == -1)
        {
            PCL_ERROR("Failed to save PCD file\n");
        }
    }



    // update last stamp
    lastPcMsgStamp = stampMsg;
}
