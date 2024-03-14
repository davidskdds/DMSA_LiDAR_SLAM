/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#ifndef IMUBUFFER_H
#define IMUBUFFER_H

#include <eigen3/Eigen/Core>

struct ImuBuffer
{

    Eigen::Matrix3Xd AccMeas;
    Eigen::Matrix3Xd AngVelMeas;
    Eigen::VectorXd Stamps;
    Eigen::Vector3d bias_gyr;

    int oldestIndex = 0;
    int maxNumMeas = 10000;
    int numUpdates = 0;

    ImuBuffer(int maxNumMeasIn) : maxNumMeas{maxNumMeasIn}
    {
        // init buffers
        AccMeas.resize(3, maxNumMeas);
        AngVelMeas.resize(3, maxNumMeas);
        Stamps.resize(maxNumMeas);

        bias_gyr.setZero();
    }

    ImuBuffer()
    {
        // init buffers
        AccMeas.resize(3, maxNumMeas);
        AngVelMeas.resize(3, maxNumMeas);
        Stamps.resize(maxNumMeas);

        bias_gyr.setZero();
    }

    void addMeasurement(Eigen::Vector3d &AccVec, Eigen::Vector3d &AngVelVec, double stamp)
    {
        AccMeas.col(oldestIndex) = AccVec;
        AngVelMeas.col(oldestIndex) = AngVelVec - bias_gyr;
        Stamps(oldestIndex) = stamp;

        // update index
        ++oldestIndex;
        if (oldestIndex == maxNumMeas)
            oldestIndex = 0;

        ++numUpdates;

        if (numUpdates == 50)
        {
            // estimate ang vel bias
            bias_gyr = AngVelMeas.block(0, 0, 3, numUpdates).rowwise().mean();
        }
    }

    double getClosestMeasurement(double &t, Eigen::Ref<Eigen::Vector3d> AccVec, Eigen::Ref<Eigen::Vector3d> AngVelVec)
    {
        double measurementDiff = 0.0;

        if (numUpdates <= maxNumMeas || oldestIndex == 0)
        {
            // find corresponding index
            auto pointerToVal = std::lower_bound(Stamps.data(), Stamps.data() + std::min(maxNumMeas - 1, numUpdates - 1), t);

            // copy data
            auto index = std::distance(Stamps.data(), pointerToVal);

            AccVec = AccMeas.col(index);
            AngVelVec = AngVelMeas.col(index);

            measurementDiff = std::abs(t - *pointerToVal);
        }
        else
        {
            // search on the right hand side of oldest id
            auto pointerToValRight = std::lower_bound(Stamps.data() + oldestIndex, Stamps.data() + maxNumMeas - 1, t);

            // search on the left hand side
            auto pointerToValLeft = std::lower_bound(Stamps.data(), Stamps.data() + oldestIndex - 1, t);

            // check wich is closer and copy data
            if (std::abs(t - *pointerToValRight) < std::abs(t - *pointerToValLeft))
            {
                auto index = std::distance(Stamps.data(), pointerToValRight);

                // right side is closer
                if (true) // index == 0)
                {
                    AccVec = AccMeas.col(index);
                    AngVelVec = AngVelMeas.col(index);
                }
                else
                {
                    // linear interpolation
                    AccVec = AccMeas.col(index - 1) + (AccMeas.col(index) - AccMeas.col(index - 1)) * (t - Stamps(index - 1)) / (Stamps(index) - Stamps(index - 1));
                    AngVelVec = AngVelMeas.col(index - 1) + (AngVelMeas.col(index) - AngVelMeas.col(index - 1)) * (t - Stamps(index - 1)) / (Stamps(index) - Stamps(index - 1));
                }

                measurementDiff = t - *pointerToValRight;
            }
            else
            {
                // left side is closer
                auto index = std::distance(Stamps.data(), pointerToValLeft);

                // right side is closer
                AccVec = AccMeas.col(index);
                AngVelVec = AngVelMeas.col(index);

                measurementDiff = t - *pointerToValLeft;
            }
        }

        return measurementDiff;
    }

    double getLatestStamp() const
    {
        if (numUpdates == 0)
            return -1.0;
        else if (oldestIndex == 0)
            return Stamps(maxNumMeas - 1);
        else
            return Stamps(oldestIndex - 1);
    }

    double getOldestStamp() const
    {
        if (numUpdates == 0)
            return -1.0;
        else if (numUpdates < maxNumMeas)
            return Stamps(0);
        else
            return Stamps(oldestIndex);
    }

    bool getBufferIdFromLinearIter(const int &i, int &bufferId) const
    {
        if (i > numUpdates - 1)
            return false;
        if (i > maxNumMeas - 1)
            return false;

        if (numUpdates < maxNumMeas)
            bufferId = i;
        else
        {
            bufferId = (oldestIndex + i) % maxNumMeas;
        }

        return true;
    }
};

#endif
