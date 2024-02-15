/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include <eigen3/Eigen/Core>
#include "OptimizablePointSet.h"
#include "ConsecutivePoses.h"
#include "KeyframeData.h"


#ifndef MAPMANAGEMENT_H
#define MAPMANAGEMENT_H

using namespace pcl;
using namespace Eigen;

class MapManagement : public OptimizablePointSet<PointNormal>
{
public:
    bool isInitialized = false;

    StampedConsecutivePoses keyframePoses;

    RingBuffer<KeyframeData> keyframeDataBuffer;

    PointCloud<PointStampId>::Ptr activePoints;

    vector<int> keyframeIds;
    vector<int> ringIds;

    int maxNumKeyframes;

    // new origin for centralization
    Vector3d newOrigin;

    bool useGravityErrorTerms = false;
    bool useOdometryErrorTerms = false;
    VectorXd gravityErrorTerm;
    Vector3d gravity;

    VectorXd odometryErrorTerm;

    VectorXd additionalErrors;

    double std_dev_acc = 0.3;
    Matrix3d odometryTranslCovInv;
    Matrix3d odometryOrientCovInv;

    Eigen::Matrix3d Cov_grav_inv;
    double balancingFactorGrav = 1.0;
    double balancingFactorOdom = 1000.0;

    MapManagement(int n_max = 30) : keyframePoses(n_max)
    {
        maxNumKeyframes = n_max;

        keyframeDataBuffer.init(n_max);

        activePoints = PointCloud<PointStampId>::Ptr(new PointCloud<PointStampId>());

        gravity << 0.0, 0.0, -9.805;

        Matrix3d Cov_grav = std::pow(std_dev_acc, 2) * Matrix3d::Identity();
        Cov_grav_inv = Cov_grav.inverse();

        odometryTranslCovInv = (std::pow(0.01, 2) * Matrix3d::Identity()).inverse();
        odometryOrientCovInv = (std::pow(0.01, 2) * Matrix3d::Identity()).inverse();
    }

    void centralize()
    {
        return;
        newOrigin = keyframePoses.relativePoses.Translations.col(0);
        keyframePoses.relativePoses.Translations.col(0).setZero();
        keyframePoses.relative2global();
    }
    void decentralize()
    {
        return;
        keyframePoses.global2relative();
        keyframePoses.relativePoses.Translations.col(0) = newOrigin;
        keyframePoses.relative2global();
    }

    void updateGlobalPoints()
    {
        int globalId = 0;
        Matrix4f currTransform = Matrix4f::Identity();
        Matrix3f currRot;

        this->minGridSize = std::numeric_limits<float>::max();

        for (int k = 0; k < keyframeDataBuffer.getNumElements(); ++k)
        {
            // update grid size
            this->minGridSize = std::min(this->minGridSize, keyframeDataBuffer.at(k).gridSize);

            PointCloud<PointNormal> &currCloud = *keyframeDataBuffer.at(k).pointCloudLocal;

            currRot = axang2rotm(keyframePoses.globalPoses.Orientations.col(k)).cast<float>();

            currTransform.block(0, 0, 3, 3) = currRot;
            currTransform.block(0, 3, 3, 1) = (keyframePoses.globalPoses.Translations.col(k)).cast<float>();

            for (auto &point : currCloud)
            {
                globalPoints.points[globalId].getVector4fMap() = currTransform * point.getVector4fMap();

                globalPoints.points[globalId].getNormalVector3fMap() = currRot * point.getNormalVector3fMap();

                ++globalId;
            }
        }
    }

    Eigen::VectorXd &getAdditionalErrorTerms()
    {
        if (useGravityErrorTerms == true && useOdometryErrorTerms == false)
            return gravityErrorTerm;

        if (useGravityErrorTerms == false && useOdometryErrorTerms == true)
            return odometryErrorTerm;

        return additionalErrors;
    }

    int updateAdditionalErrors()
    {
        if (useGravityErrorTerms == false && useOdometryErrorTerms == false)
            return 0;

        if (useGravityErrorTerms == true && useOdometryErrorTerms == false)
        {
            this->updateGravityErrors();

            return gravityErrorTerm.size();
        }

        if (useGravityErrorTerms == false && useOdometryErrorTerms == true)
        {
            this->updateOdometryErrors();

            return odometryErrorTerm.size();
        }

        this->updateGravityErrors();

        this->updateOdometryErrors();

        additionalErrors.conservativeResize(gravityErrorTerm.size() + odometryErrorTerm.size());

        additionalErrors << gravityErrorTerm, odometryErrorTerm;

        return additionalErrors.size();
    }

    void getPoseParameters(Eigen::VectorXd &params)
    {
        keyframePoses.relativePoses.getParamsAsVector(params);
    }

    void setPoseParameters(const Eigen::VectorXd &params)
    {
        keyframePoses.relativePoses.setParamsFromVector(params);

        keyframePoses.relative2global();
    }

    const int &getIdOfPoint(int &globalPointIndex)
    {
        // return keyframeIds[globalPointIndex];
        return ringIds[globalPointIndex];
    }

    void updateGravityErrors()
    {
        // resize
        gravityErrorTerm.conservativeResize(keyframeDataBuffer.getNumElements());

        // reset
        gravityErrorTerm.setZero();

        Eigen::Vector3d diffVec;

        for (int k = 1; k < keyframeDataBuffer.getNumElements(); ++k)
        {
            // calculate gravity error
            diffVec = axang2rotm(keyframePoses.globalPoses.Orientations.col(k)) * keyframeDataBuffer.at(k).measuredGravity - gravity;

            // save
            gravityErrorTerm(k) = diffVec.transpose() * Cov_grav_inv * diffVec;
            gravityErrorTerm(k) *= balancingFactorGrav;
        }
    }

    void updateOdometryErrors()
    {
        odometryErrorTerm.conservativeResize(keyframeDataBuffer.getNumElements() - 1);
        odometryErrorTerm.setZero();

        Vector3d translDiff, orientDiff;

        for (int k = 1; k < keyframeDataBuffer.getNumElements(); ++k)
        {
            translDiff = keyframeDataBuffer.at(k).relativeTransl - keyframePoses.relativePoses.Translations.col(k);

            orientDiff = rotm2axang(axang2rotm(keyframePoses.relativePoses.Orientations.col(k)).transpose() * keyframeDataBuffer.at(k).relativeOrientMat);

            odometryErrorTerm(k - 1) += translDiff.transpose() * odometryTranslCovInv * translDiff;
            odometryErrorTerm(k - 1) += orientDiff.transpose() * odometryOrientCovInv * orientDiff;

            odometryErrorTerm(k - 1) *= balancingFactorOdom;
        }
    }

    std::shared_ptr<MapManagement> getSubmap(int fromId, int toId)
    {
        int nFrames = toId - fromId + 1;

        std::shared_ptr<MapManagement> subset = std::shared_ptr<MapManagement>(new MapManagement(nFrames));

        subset->minGridSize = std::numeric_limits<float>::max();

        for (int k = fromId; k <= toId; ++k)
        {
            Ref<Vector3d> currTransl = this->keyframePoses.globalPoses.Translations.col(k);
            Ref<Vector3d> currOrient = this->keyframePoses.globalPoses.Orientations.col(k);
            double &currStamp = this->keyframePoses.stamps(k);

            subset->addKeyframe(currTransl, currOrient, currStamp, this->keyframeDataBuffer.at(k));

            subset->minGridSize = std::min(subset->minGridSize, this->keyframeDataBuffer.at(k).gridSize);
        }

        subset->keyframePoses.global2relative();

        return subset;
    }

    void updatePosesFromSubmap(int fromId, int toId, MapManagement &submap)
    {
        submap.keyframePoses.global2relative();

        int numParams = toId - fromId + 1;

        this->keyframePoses.relativePoses.Translations.block(0, fromId + 1, 3, numParams - 1) = submap.keyframePoses.relativePoses.Translations.block(0, 1, 3, numParams - 1);
        this->keyframePoses.relativePoses.Orientations.block(0, fromId + 1, 3, numParams - 1) = submap.keyframePoses.relativePoses.Orientations.block(0, 1, 3, numParams - 1);

        this->keyframePoses.relative2global();
    }

    void getGlobalKeyframeCloud(int keyId, PointCloud<PointNormal>::Ptr pcTarget)
    {

        Matrix4f transformToGlobal = Matrix4f::Identity();

        transformToGlobal.block(0, 0, 3, 3) = axang2rotm(keyframePoses.globalPoses.Orientations.col(keyId)).cast<float>();
        transformToGlobal.block(0, 3, 3, 1) = (keyframePoses.globalPoses.Translations.col(keyId)).cast<float>();

        transformPointCloudWithNormals(*keyframeDataBuffer.at(keyId).pointCloudLocal, *pcTarget, transformToGlobal);
    }

    int getNumPointsInBuffer()
    {
        int numPts = 0;

        for (int k = 0; k < keyframeDataBuffer.getNumElements(); ++k)
            numPts += keyframeDataBuffer.at(k).pointCloudLocal->size();

        return numPts;
    }

    void addKeyframe(Ref<Vector3d> position_w, Ref<Vector3d> orient_w, double stamp, KeyframeData keyframeData)
    {
        keyframePoses.relative2global();

        if (keyframeDataBuffer.isFull() == false)
        {
            int currId = keyframeDataBuffer.getNumElements();

            keyframePoses.globalPoses.Translations.col(currId) = position_w;
            keyframePoses.globalPoses.Orientations.col(currId) = orient_w;
            keyframePoses.stamps(currId) = stamp;
        }
        else
        {
            // shift
            keyframePoses.globalPoses.Translations.block(0, 0, 3, maxNumKeyframes - 1) = keyframePoses.globalPoses.Translations.block(0, 1, 3, maxNumKeyframes - 1);
            keyframePoses.globalPoses.Orientations.block(0, 0, 3, maxNumKeyframes - 1) = keyframePoses.globalPoses.Orientations.block(0, 1, 3, maxNumKeyframes - 1);

            keyframePoses.stamps.segment(0, maxNumKeyframes - 1) = keyframePoses.stamps.segment(1, maxNumKeyframes - 1);

            // add new
            keyframePoses.globalPoses.Translations.col(maxNumKeyframes - 1) = position_w;
            keyframePoses.globalPoses.Orientations.col(maxNumKeyframes - 1) = orient_w;
            keyframePoses.stamps(maxNumKeyframes - 1) = stamp;
        }

        keyframePoses.global2relative();

        // save odometry result
        if (keyframeDataBuffer.isFull() == false)
        {
            int currId = keyframeDataBuffer.getNumElements();

            keyframeData.relativeTransl = keyframePoses.relativePoses.Translations.col(currId);
            keyframeData.relativeOrient = keyframePoses.relativePoses.Orientations.col(currId);

            keyframeData.relativeOrientMat = axang2rotm(keyframeData.relativeOrient);
        }
        else
        {
            keyframeData.relativeTransl = keyframePoses.relativePoses.Translations.col(maxNumKeyframes - 1);
            keyframeData.relativeOrient = keyframePoses.relativePoses.Orientations.col(maxNumKeyframes - 1);

            keyframeData.relativeOrientMat = axang2rotm(keyframeData.relativeOrient);
        }

        // add keyframe data to ringbuffer
        keyframeDataBuffer.addElem(keyframeData);

        if (isInitialized == false)
            isInitialized = true;

        // update global points
        int numGlobalPts = getNumPointsInBuffer();

        globalPoints.resize(numGlobalPts);
        keyframeIds.resize(numGlobalPts);
        ringIds.resize(numGlobalPts);

        // update global map
        updateGlobalPoints();

        // update keyframe ids
        int globalId = 0;

        for (int k = 0; k < keyframeDataBuffer.getNumElements(); ++k)
        {
            for (int j = 0; j < static_cast<int>(keyframeDataBuffer.at(k).pointCloudLocal->points.size()); ++j)
            {
                keyframeIds[globalId] = k;
                ringIds[globalId] = keyframeDataBuffer.at(k).ringIds(j);

                ++globalId;
            }
        }
    }
};

#endif
