/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include <string>
#include <eigen3/Eigen/Core>

#ifndef CONFIG_H
#define CONFIG_H

using namespace Eigen;

struct Config
{
    int n_clouds = 5;
    int num_control_points = 6;
    std::string sensor = "hesai";
    bool liveView = true;
    bool optimize_sliding_window_keyframes = true;
    int last_n_keyframes_for_optim = 10;
    int max_num_points_per_scan = 3000;
    float minDistDS = 30.0;
    double alpha_keyframe_optim = 0.3;
    int num_iter_keyframe_optim = 10;

    double alpha_sliding_window_imu = 0.05;
    double alpha_sliding_window_no_imu = 0.3;
    double max_step_sliding_window_imu = 0.05;
    double max_step_sliding_window_no_imu = 0.3;
    double dist_new_keyframe = 2.0;
    double dist_static_points_keyframe = 30.0;
    double min_overlap_new_keyframe = 0.75;
    int num_iter_sliding_window_optim = 15;
    int oldest_k_keyframes_as_static_points = 10;
    float min_dist = 0.0;

    double dt_res = 0.001;
    bool use_imu = true;
    double timeshift_to_imu = 0.0;
    int min_num_points_gauss = 6;
    double delta_t_pcs = 0.1;
    double imu_factor_weight_submap;

    bool use_gravity_term_in_keyframe_opt = true;
    double balancing_factor_gravity = 1.0;

    bool use_odometry_term_in_keyframe_opt = true;
    double balancing_factor_odometry = 1000.0;

    float min_grid_size_keyframe_opt = 0.15f;

    Matrix3d cov_acc = 0.3 * 0.3 * Matrix3d::Identity();
    Matrix3d cov_gyr = 0.01 * 0.01 * Matrix3d::Identity();

    Matrix4f lidarToImuTform = Matrix4f::Identity();

    double epsilon_keyframe_opt;

    float decay_rate_sw = 0.95;
    float decay_rate_key = 0.95;

    bool select_best_set_key = false;
    int min_num_points_gauss_key = 6;
};

#endif
