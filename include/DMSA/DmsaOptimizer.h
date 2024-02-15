/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include <vector>
#include <eigen3/Eigen/Dense>
#include <limits>
#include <iostream>
#include <memory>
#include <eigen3/Eigen/Eigenvalues>
#include "Gaussians.h"
#include "OptimizablePointSet.h"

#ifndef DMSAOPTIMIZER_H
#define DMSAOPTIMIZER_H

using namespace std;
using namespace Eigen;
using namespace pcl;

struct DmsaOptimSettings
{
    int num_iter = 15;
    double epsilon = 1e-5;
    bool use_analytic_jacobi = false;
    double step_length_optim = 0.05;
    double max_step = 0.03;
    bool gauss_split = false;
    float grid_size_1_factor = 2.0;
    float grid_size_2_factor = 5.0;
    int min_num_points_per_set = 6;
    int min_num_gaussians = 30;
    float lambda_diag = 0.00001;
    bool use_centralization = true;
    float decay_rate = 0.7;
    bool select_best_set = false;
};

template <typename PointT>
class DmsaOptimizer
{
protected:
    Gaussians currentGauss;

    MatrixXd Jacobian;
    MatrixXd H;

    int minId = 0;
    int maxId = 0;

public:
    void optimizeSet(OptimizablePointSet<PointT> &pointSetToOptimize, DmsaOptimSettings settings = DmsaOptimSettings())
    {
        omp_set_dynamic(0);     // Explicitly disable dynamic teams
        omp_set_num_threads(4); // Use 4 threads for all consecutive parallel regions

        VectorXd paramVec, paramVecBestSet;
        VectorXd errorVec, optimStep;
        double maxElem;
        double adaptiveAlpha{1.0}, lastMeanScore{0.0}, currMeanScore, maxScore;

        // get min and max id
        updateMinMaxId(pointSetToOptimize);

        if (settings.use_centralization)
            pointSetToOptimize.centralize();

        for (int iter = 0; iter < settings.num_iter; ++iter)
        {
            // save params
            pointSetToOptimize.getPoseParameters(paramVec);

            // update reference poses and global points
            pointSetToOptimize.updateGlobalPoints();

            // reset gaussians
            currentGauss.reset();

            // create gaussians with first resolution
            if (settings.grid_size_1_factor > std::numeric_limits<float>::min())
                createGaussianSets(pointSetToOptimize, settings.grid_size_1_factor * pointSetToOptimize.minGridSize, settings.min_num_points_per_set, settings.gauss_split);

            // create gaussians with second resolution
            if (settings.grid_size_2_factor > std::numeric_limits<float>::min())
                createGaussianSets(pointSetToOptimize, settings.grid_size_2_factor * pointSetToOptimize.minGridSize, settings.min_num_points_per_set, settings.gauss_split);

            // calc mean score
            currMeanScore = currentGauss.score / static_cast<float>(currentGauss.numPointSets);

            if (iter == 0)
            {
                lastMeanScore = currMeanScore;
                maxScore = currMeanScore;
                pointSetToOptimize.getPoseParameters(paramVecBestSet);
            }

            if (currMeanScore > maxScore)
            {
                maxScore = currMeanScore;
                paramVecBestSet = paramVec;
            }

            // reduce adaptive alpha
            if (currMeanScore < lastMeanScore)
                adaptiveAlpha *= settings.decay_rate;

            lastMeanScore = currMeanScore;

            if (currentGauss.numPointSets < settings.min_num_gaussians)
            {
                std::cout << "Number of gaussians (" << currentGauss.numPointSets << ") is smaller than threshold, dmsa optimization is aborted after iteration " << iter << " . . . " << std::endl;
                break;
            }

            // update balances
            currentGauss.updateRebalancingWeights();

            // update errors
            updateErrorTerms(pointSetToOptimize, errorVec);

            // update Jacobian
            calcNumericJacobian(Jacobian, errorVec, pointSetToOptimize);

            // calculate H matrix
            H = Jacobian.transpose() * Jacobian;

            // add lambda
            H.diagonal().array() += settings.lambda_diag;

            // LM optimization step
            optimStep = -adaptiveAlpha * settings.step_length_optim * H.inverse() * Jacobian.transpose() * errorVec;

            // stop in case of nan
            if (optimStep.array().isNaN().any())
            {
                pointSetToOptimize.setPoseParameters(paramVec);
                std::cout << "Stop optimization because of NaN after iteration " << iter << " . . . " << std::endl;

                break;
            }

            // limit increment for robustness
            maxElem = std::max(optimStep.maxCoeff(), -optimStep.minCoeff());

            if (maxElem > settings.max_step)
                optimStep = (settings.max_step / maxElem) * optimStep;

            // add to current trajectory
            paramVec = paramVec + optimStep.cast<double>();

            pointSetToOptimize.setPoseParameters(paramVec);

            // epsilon stop criterion
            if (optimStep.norm() < settings.epsilon)
            {
                std::cout << "Optimization step is smaller than epsilon after iteration " << iter << " . . . " << std::endl;
                break;
            }
        }

        if (settings.select_best_set == true)
            pointSetToOptimize.setPoseParameters(paramVecBestSet);

        if (settings.use_centralization)
            pointSetToOptimize.decentralize();

        pointSetToOptimize.updateGlobalPoints();
    }

protected:
    void updateMinMaxId(OptimizablePointSet<PointT> &pointSetToOptimize)
    {
        maxId = std::numeric_limits<int>::min();
        minId = std::numeric_limits<int>::max();

        for (int k = 0; k < pointSetToOptimize.globalPoints.points.size(); ++k)
        {
            if (pointSetToOptimize.getIdOfPoint(k) > maxId)
                maxId = pointSetToOptimize.getIdOfPoint(k);
            if (pointSetToOptimize.getIdOfPoint(k) < minId)
                minId = pointSetToOptimize.getIdOfPoint(k);
        }
    }

    void calcNumericJacobian(MatrixXd &Jacobian, VectorXd &error0, OptimizablePointSet<PointT> &pointSetToOptimize)
    {
        VectorXd paramVecOrigin, paramVecLoop;
        VectorXd errorVec;

        pointSetToOptimize.getPoseParameters(paramVecOrigin);

        // adjust dimensions of Jacobian
        Jacobian.conservativeResize(error0.size(), paramVecOrigin.size());

        double increment = 1.0 * sqrt(std::numeric_limits<float>::epsilon());
        double one_div_incr = 1.0 / increment;

        for (int k = 0; k < paramVecOrigin.size(); ++k)
        {
            // reset param vector
            paramVecLoop = paramVecOrigin;

            // add increment for derivation
            paramVecLoop(k) += static_cast<double>(increment);

            pointSetToOptimize.setPoseParameters(paramVecLoop);

            pointSetToOptimize.updateGlobalPoints();

            updateErrorTerms(pointSetToOptimize, errorVec);

            // update jacobian column
            Jacobian.col(k) = one_div_incr * (errorVec - error0);
        }

        // set original parameters
        pointSetToOptimize.setPoseParameters(paramVecOrigin);
    }

    void updateErrorTerms(OptimizablePointSet<PointT> &pointSetToOptimize, VectorXd &errorVec)
    {

        int numElemAddError = pointSetToOptimize.updateAdditionalErrors();

        errorVec.conservativeResize(currentGauss.numPointSets + numElemAddError);

        // update point to gaussian errors
        for (int k = 0; k < currentGauss.numPointSets; ++k)
        {
            Eigen::Vector3f mean, diffVec;

            // calculate mean
            mean.setZero();

            for (int j = 0; j < currentGauss.connectedPointIds[k].size(); ++j)
            {
                mean = mean + pointSetToOptimize.globalPoints.points[currentGauss.connectedPointIds[k](j)].getVector3fMap();
            }

            mean = mean / static_cast<float>(currentGauss.connectedPointIds[k].size());

            errorVec(k) = 0.0;

            // loop over points within sets
            for (int j = 0; j < currentGauss.connectedPointIds[k].size(); ++j)
            {
                diffVec = pointSetToOptimize.globalPoints.points[currentGauss.connectedPointIds[k](j)].getVector3fMap() - mean;

                errorVec(k) += static_cast<double>(currentGauss.rebalancingWeights(k) * diffVec.transpose() * currentGauss.infoMats[k] * diffVec);
            }
        }

        // update additional error terms
        if (numElemAddError > 0)
            errorVec.tail(numElemAddError) = pointSetToOptimize.getAdditionalErrorTerms();
    }

    void createGaussianSets(OptimizablePointSet<PointT> &pointSetToOptimize, float resolution, int minNumberPts, const bool &splitEnabled)
    {
        // init
        std::vector<int> ringSet;
        MatrixX3f currGridSet;

        // octree
        pcl::octree::OctreePointCloud<PointT> octree(resolution); // set voxel size

        // Set the input point cloud to the octree
        octree.setInputCloud(pointSetToOptimize.globalPoints.makeShared());

        // Construct the octree
        octree.addPointsFromInputCloud();

        int splitted{0}, nonSplitted{0};

        // iterate over octree
        for (auto it = octree.leaf_depth_begin(); it != octree.leaf_depth_end(); ++it)
        {
            std::vector<int> indices;
            std::vector<int> indices2;

            it.getLeafContainer().getPointIndices(indices);

            ++currentGauss.numRawSets;

            // check rings
            ringSet.resize(indices.size());
            for (int k = 0; k < indices.size(); ++k)
                ringSet[k] = pointSetToOptimize.getIdOfPoint(indices[k]);

            if ((ringSet.size() >= minNumberPts) && *max_element(ringSet.begin(), ringSet.end()) != *min_element(ringSet.begin(), ringSet.end()))
            {
                // try to split the point set based on normal vector directions
                if (splitEnabled && splitSet(pointSetToOptimize.globalPoints, indices, indices2, minNumberPts) == true)
                {
                    // add first set
                    copyPointsIntoEigMatrix(pointSetToOptimize, indices, currGridSet);

                    ringSet.resize(indices.size());
                    for (int k = 0; k < indices.size(); ++k)
                        ringSet[k] = pointSetToOptimize.getIdOfPoint(indices[k]);

                    if (indices.size() > minNumberPts && *max_element(ringSet.begin(), ringSet.end()) != *min_element(ringSet.begin(), ringSet.end()))
                    {
                        currentGauss.addPointSet(indices, currGridSet, pointSetToOptimize.getWeightOfPointSet(indices));
                    }

                    // add second set
                    copyPointsIntoEigMatrix(pointSetToOptimize, indices2, currGridSet);

                    ringSet.resize(indices.size());
                    for (int k = 0; k < indices.size(); ++k)
                        ringSet[k] = pointSetToOptimize.getIdOfPoint(indices[k]);

                    if (indices2.size() > minNumberPts && *max_element(ringSet.begin(), ringSet.end()) != *min_element(ringSet.begin(), ringSet.end()))
                    {
                        currentGauss.addPointSet(indices2, currGridSet, pointSetToOptimize.getWeightOfPointSet(indices2));
                    }

                    ++splitted;
                }
                else
                {
                    // add one set
                    copyPointsIntoEigMatrix(pointSetToOptimize, indices, currGridSet);

                    currentGauss.addPointSet(indices, currGridSet, pointSetToOptimize.getWeightOfPointSet(indices));
                    ++nonSplitted;
                }
            }
        }

        // if(splitEnabled) std::cout << "Splitted: "<<splitted<<" non-splitted: "<<nonSplitted<<std::endl;
    }

    void copyPointsIntoEigMatrix(OptimizablePointSet<PointT> &pointSetToOptimize, std::vector<int> &indices, MatrixX3f &pointsOut)
    {
        pointsOut.conservativeResize(indices.size(), 3);

        int pointSetId = 0;

        for (auto id : indices)
        {
            pointsOut.row(pointSetId) = Map<RowVector3f>(pointSetToOptimize.globalPoints.points[id].data);
            ++pointSetId;
        }
    }
};

#endif
