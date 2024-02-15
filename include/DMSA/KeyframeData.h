/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include <eigen3/Eigen/Core>

#ifndef KEYFRAMEDATA_H
#define KEYFRAMEDATA_H

using namespace Eigen;
using namespace pcl;

class KeyframeData
{
public:
    PointCloud<PointNormal>::Ptr pointCloudLocal;

    VectorXi ringIds;

    float gridSize;

    Vector3d measuredGravity;

    Vector3d relativeTransl;
    Vector3d relativeOrient;

    Matrix3d relativeOrientMat;
};

#endif
