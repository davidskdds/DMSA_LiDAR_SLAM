/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#define PCL_NO_PRECOMPILE

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/pcl_base.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <math.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>

#ifndef POINTSTAMPID_H
#define POINTSTAMPID_H

struct PointStampId
{
    PCL_ADD_POINT4D;
    double stamp;
    int id;
    int isStatic;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointStampId,
                                  (float, x, x)(float, y, y)(float, z, z)(double, stamp, stamp)(int, id, id)(int, isStatic, isStatic))

#endif
