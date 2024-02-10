/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <string>

#include "dmsa_slam_ros.h"
#include "DMSA/DmsaSlam.h"

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "dmsa_slam_ros");

    dmsa_slam_ros SLAM_obj;

    SLAM_obj.spin();

    return 0;
}
