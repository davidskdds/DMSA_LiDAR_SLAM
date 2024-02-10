/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "PointStampId.h"

#ifndef GAUSSIANS_H
#define GAUSSIANS_H

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

    Eigen::Vector3f meanSet1, meanSet2;
    int numSet1, numSet2;
    std::vector<int> newIdsSet1, newIdsSet2;

    newIdsSet1.resize(IdsSet.size());
    newIdsSet2.resize(IdsSet.size());

    // split normal vectors with xy-plane, xz-plane and yz-plane and analyse distance between the two custers to split double walls
    for (int dim = 0; dim < 3; ++dim)
    {
        // reset
        numSet1 = 0;
        numSet2 = 0;

        meanSet1 << 0.0, 0.0, 0.0;
        meanSet2 << 0.0, 0.0, 0.0;

        for (int k = 0; k < IdsSet.size(); ++k)
        {

            if (cloud.points[IdsSet[k]].getNormalVector3fMap()(dim) > 0.0)
            {
                meanSet1 += cloud.points[IdsSet[k]].getNormalVector3fMap();
                newIdsSet1[numSet1] = IdsSet[k];
                ++numSet1;
            }
            else
            {
                meanSet2 += cloud.points[IdsSet[k]].getNormalVector3fMap();
                newIdsSet2[numSet2] = IdsSet[k];
                ++numSet2;
            }
        }

        if (numSet1 < minNumPts / 2 || numSet2 < minNumPts / 2)
            continue;

        // mean
        meanSet1 *= 1.0 / (float)numSet1;
        meanSet2 *= 1.0 / (float)numSet2;

        // analyse clusters
        if ((meanSet1 - meanSet2).norm() > 1.3)
        {
            newIdsSet1.resize(numSet1);
            newIdsSet2.resize(numSet2);

            IdsSet = newIdsSet1;
            IdsSet2 = newIdsSet2;
            // std::cout << "Splitted set dim: "<<dim<<" cluster1: "<<meanSet1.transpose()<<" cluster2: "<<meanSet2.transpose()<<std::endl;
            return true;
        }
    }

    return false;
}

class Gaussians
{
public:
    std::vector<VectorXi> connectedPointIds;
    std::vector<Matrix3f> infoMats;
    VectorXi numPointsPerSet;
    VectorXf rebalancingWeights;
    VectorXf obervationWeights;
    VectorXf infoDetsPerSet;

    float sumOfInfoNorms;
    float sumOfWeightedInfoNorms;
    float sumOfCovNorms;
    float score;
    float realDifferentialEntropy;
    int numSkips = 0;

    Matrix3f InformationBalance;

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
        infoDetsPerSet.resize(maxSets);
        obervationWeights.setZero();
        infoDetsPerSet.setZero();

        numPointSets = 0;
        numPoints = 0;
        numRawSets = 0;
    }

    void reset()
    {
        numPointsPerSet.setZero();
        obervationWeights.setZero();
        infoDetsPerSet.setZero();

        numPointSets = 0;
        numPoints = 0;

        sumOfInfoNorms = 0.000;
        sumOfCovNorms = 0.000;
        sumOfWeightedInfoNorms = 0.000;
        score = 0.000;
        realDifferentialEntropy = 0.000;
        numSkips = 0;
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
            infoDetsPerSet.conservativeResize(maxSets);
        }

        MatrixXf centered = subset.rowwise() - subset.colwise().mean();
        Matrix3f cov = (centered.adjoint() * centered) / float(subset.rows() - 1);
        float infoDet = cov.inverse().determinant();

        // update score
        score = score + std::log(1.0f + infoDet); //*std::pow(static_cast<float>(ids.size()),2);

        // limit covariance
        limit_cov(cov);

        // save information matrix
        infoMats[numPointSets] = cov.inverse();

        sumOfInfoNorms = sumOfInfoNorms + infoMats[numPointSets].norm();

        // sumOfWeightedInfoNorms = sumOfWeightedInfoNorms + (float) connectedPointIds[k].size() * infoMats[k].norm();

        realDifferentialEntropy = realDifferentialEntropy + log(cov.determinant());

        // limit precision matrix
        // limit_prec(infoMats[numPointSets]);

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

    void limit_cov(Matrix3f &io_cov)
    {

        EigenSolver<Matrix3f> eigensolver;
        eigensolver.compute(io_cov);

        Vector3f eigen_values = eigensolver.eigenvalues().real();
        Matrix3f eigen_vectors = eigensolver.eigenvectors().real();

        // modify eigen values
        for (int k = 0; k < 3; ++k)
        {
            eigen_values(k) = std::max(eigen_values(k), 0.0001f);
        }

        // create diagonal matrix
        DiagonalMatrix<float, 3> diagonal_matrix(eigen_values(0), eigen_values(1), eigen_values(2));

        // update covariance
        io_cov = eigen_vectors * diagonal_matrix * eigen_vectors.inverse();
    }
};

#endif
