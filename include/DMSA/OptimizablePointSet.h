/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include <eigen3/Eigen/Core>
#include "PointCloudPlus.h"
#include <iostream>

#ifndef OPTIMIZABLEPOINTSET_H
#define OPTIMIZABLEPOINTSET_H

using namespace pcl;

template <typename PointT>
class OptimizablePointSet
{
public:
    PointCloud<PointT> globalPoints;

    float minGridSize = 0.3;

    virtual Eigen::VectorXd &getAdditionalErrorTerms()
    {
        std::cerr << "virtual Eigen::VectorXf& getAdditionalErrorTerms() is undefined";
        return placeholder;
    }

    virtual void updateGlobalPoints() { std::cerr << "virtual void updateGlobalPoints() is undefined"; }

    virtual int updateAdditionalErrors()
    {   // return number of error values
        return 0;
    }

    virtual void getPoseParameters(Eigen::VectorXd &params) { std::cerr << "virtual void getPoseParameters(Eigen::VectorXd& params) is undefined"; }

    virtual void setPoseParameters(const Eigen::VectorXd &params) { std::cerr << "virtual void setPoseParameters(Eigen::VectorXd params) is undefined"; }

    virtual const int &getIdOfPoint(int &globalPointIndex)
    {
        std::cerr << "virtual const int& getIdOfPoint(int& indexInGlobalPoints) is undefined";
        return globalPointIndex;
    }

    virtual void centralize() { std::cerr << "virtual void centralize() is undefined"; }
    virtual void decentralize() { std::cerr << "virtual void decentralize() is undefined"; }

    virtual float getWeightOfPointSet(std::vector<int> &ids) { return 1.0f; }

private:
    Eigen::VectorXd placeholder;
};

#endif
