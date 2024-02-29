/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#ifndef POINTCLOUDPLUS_H
#define POINTCLOUDPLUS_H

#include "PointStampId.h"
#include <boost/shared_ptr.hpp>

class PointCloudPlus : public pcl::PointCloud<PointStampId>
{
public:
    float gridSize = 0.0f;
    using Ptr = boost::shared_ptr<PointCloudPlus>;
    using ConstPtr = boost::shared_ptr<const PointCloudPlus>;

    double getMinStamp()
    {
        double stamp = points[0].stamp;

        for (auto &point : points) if (point.stamp < stamp) stamp = point.stamp;

        return stamp;
    }

    double getMaxStamp()
    {
        double stamp = points[0].stamp;

        for (auto &point : points) if (point.stamp > stamp) stamp = point.stamp;

        return stamp;
    }

    void addStaticPoints(const pcl::PointCloud<pcl::PointNormal> &pcStatic)
    {
        int currSz = points.size();

        points.resize(currSz + static_cast<int>(pcStatic.size()));

        for (int k = currSz; k < points.size(); ++k)
        {
            points[k].getVector3fMap() = pcStatic.points[k - currSz].getVector3fMap();

            points[k].stamp = 0.0;
            points[k].id = -1;
        }
    }

    void removeStaticPoints()
    {
        if (points.size() == 0)
            return;

        for (int k = 0; k < points.size(); ++k)
        {
            if (points[k].id < 0)
            {
                points.resize(k);
                break;
            }
        }
    }
};

#endif
