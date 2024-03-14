/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#ifndef GAUSSIANS_H
#define GAUSSIANS_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace Eigen;

// General case, no normals available
template <typename PointCloudType>
inline bool splitSet(const PointCloudType &cloud, std::vector<int> &IdsSet, std::vector<int> &IdsSet2, int &minNumPts)
{
    // This just returns false and does nothing else
    return false;
}

// Specialization for pcl::PointCloud<PointNormal>
template <>
inline bool splitSet(const pcl::PointCloud<pcl::PointNormal> &cloud, std::vector<int> &IdsSet, std::vector<int> &IdsSet2, int &minNumPts)
{
    // find two normal vectors that are maximum opposite
    float minDiffFromZero = std::numeric_limits<float>::max();

    float currDiff;
    std::vector<int> minDiffSet(2);

    for (auto & id1 : IdsSet)
    {
        for (auto & id2 : IdsSet)
        {
            if (id1 == id2) continue;

            currDiff = (cloud.points[id1].getNormalVector3fMap() + cloud.points[id2].getNormalVector3fMap()).norm();

            if (currDiff < minDiffFromZero)
            {
                minDiffFromZero = currDiff;
                minDiffSet[0] = id1;
                minDiffSet[1] = id2;
            }
        }
    }

    // if there are no opposite vectors, stop here
    if (minDiffFromZero > 0.5f) return false;

    // create splitting plane
    int & id1 = minDiffSet[0];
    int & id2 = minDiffSet[1];

    Vector3f refNormalVec = cloud.points[id1].getNormalVector3fMap();
    Vector3f refNormalVec2 = cloud.points[id2].getNormalVector3fMap();

    // create new id vectors
    std::vector<int> newIdsSet1, newIdsSet2;
    newIdsSet1.reserve(IdsSet.size());
    newIdsSet2.reserve(IdsSet.size());

    float diff1, diff2;

    for (auto & id : IdsSet)
    {
        diff1 = (refNormalVec-cloud.points[id].getNormalVector3fMap()).norm();
        diff2 = (refNormalVec2-cloud.points[id].getNormalVector3fMap()).norm();

        if (diff1< diff2) newIdsSet1.push_back(id);
        else newIdsSet2.push_back(id);

    }

    IdsSet = newIdsSet1;
    IdsSet2 = newIdsSet2;

    return true;

}

class Gaussians
{
public:
    std::vector<VectorXi> connectedPointIds;
    std::vector<Matrix3f> infoMats;
    VectorXi numPointsPerSet;
    VectorXf rebalancingWeights;
    VectorXf obervationWeights;


    float score;
    bool limited_cov = false;

    int numPointSets = 0;
    int maxSets = 3000;
    int numPoints;

    int numRawSets = 0;

    Gaussians()
    {
        // init
        connectedPointIds.resize(maxSets);
        infoMats.resize(maxSets);
        numPointsPerSet.resize(maxSets);
        numPointsPerSet.setZero();
        rebalancingWeights.resize(maxSets);
        obervationWeights.resize(maxSets);
        obervationWeights.setZero();


        numPointSets = 0;
        numPoints = 0;
        numRawSets = 0;
    }

    void reset()
    {
        numPointsPerSet.setZero();
        obervationWeights.setZero();

        numPointSets = 0;
        numPoints = 0;

        score = 0.000;

    }

    void addPointSet(std::vector<int> &ids, MatrixX3f &subset, float observationWeight = 1.0)
    {

        // prevent overflow
        if (numPointSets >= maxSets)
        {
            // increase size
            maxSets = 2 * maxSets;

            connectedPointIds.resize(maxSets);
            infoMats.resize(maxSets);
            numPointsPerSet.conservativeResize(maxSets);
            rebalancingWeights.conservativeResize(maxSets);
            obervationWeights.conservativeResize(maxSets);
        }

        MatrixXf centered = subset.rowwise() - subset.colwise().mean();
        Matrix3f cov = (centered.adjoint() * centered) / float(subset.rows() - 1);
        float infoDet = cov.inverse().determinant();

        // update score
        score = score + std::log(1.0f + std::abs(infoDet) );

        // limit covariance
        limitCovariance(cov);

        // save information matrix
        infoMats[numPointSets] = cov.inverse();

        // save observation weight
        obervationWeights(numPointSets) = observationWeight;

        // copy ids
        connectedPointIds.at(numPointSets) = Map<VectorXi, Unaligned>(ids.data(), ids.size());

        // save number of points
        numPointsPerSet(numPointSets) = ids.size();

        // update
        ++numPointSets;
        numPoints += ids.size();
    }

    void updateRebalancingWeights()
    {
        rebalancingWeights.head(numPointSets) = numPointsPerSet.head(numPointSets).cast<float>().array().pow(-1).matrix();

        // add information weights
        rebalancingWeights.head(numPointSets) = rebalancingWeights.head(numPointSets).cwiseProduct(obervationWeights.head(numPointSets));

        rebalancingWeights.head(numPointSets) = rebalancingWeights.head(numPointSets) / rebalancingWeights.head(numPointSets).mean();
        // rebalancingWeights.setConstant(1.0);
    }

    void limitCovariance(Matrix3f &io_cov)
    {

        EigenSolver<Matrix3f> eigensolver;
        eigensolver.compute(io_cov);

        Vector3f eigenValues = eigensolver.eigenvalues().real();
        Matrix3f eigenVectors = eigensolver.eigenvectors().real();

        // modify eigen values
        for (int k = 0; k < 3; ++k)
        {
            eigenValues(k) = std::max(eigenValues(k), 0.0001f);
        }

        // create diagonal matrix
        DiagonalMatrix<float, 3> diagonal_matrix(eigenValues(0), eigenValues(1), eigenValues(2));

        // update covariance
        io_cov = eigenVectors * diagonal_matrix * eigenVectors.inverse();
    }
};

#endif
