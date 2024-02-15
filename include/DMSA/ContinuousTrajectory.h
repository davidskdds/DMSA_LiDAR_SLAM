/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include "OptimizablePointSet.h"
#include "ConsecutivePoses.h"

#include <vector>
#include "PointCloudBuffer.h"
#include "ImuBuffer.h"
#include "ImuPreintegration.h"
#include <boost/math/interpolators/barycentric_rational.hpp>
#include <boost/math/interpolators/cubic_b_spline.hpp>

#ifndef CONTINUOUSTRAJECTORY_H
#define CONTINUOUSTRAJECTORY_H

using namespace Eigen;
using namespace std;

class ContinuousTrajectory : public OptimizablePointSet<PointStampId>
{
public:
    StampedConsecutivePoses SparsePoses;

    Poses DenseGlobalPoses;

    vector<Matrix4f> DenseTformsLocal2Global;

    VectorXd trajTime;
    Vector3d gravity;

    // preintegrated imu measurements
    vector<Matrix3d> preintImuRots;
    vector<Vector3d> preintRelPositions;
    vector<Vector3d> preintRelVelocity;

    Vector3d preintPosComplHor;

    vector<Matrix<double, 9, 9>> CovPVRot_inv;

    double t0;
    double horizon;

    double dt_res = 0.0001;

    bool validImuData = false;

    double balancingImu = 0.001f;
    bool useImuErrorTerms;

    int numParams;
    int n_total;

    // translation buffer for centralization
    Vector3d newOrigin;

    // imu
    Matrix3Xd accMeas;
    Matrix3Xd angVelMeas;

    VectorXd imuFactorError;

    VectorXi paramIndices;

    std::shared_ptr<PointCloudBuffer> regPcBuffer;

    vector<vector<int>> tformIdPerPoint;

    ContinuousTrajectory() {}

    void centralize()
    {
        newOrigin = SparsePoses.relativePoses.Translations.col(0);

        SparsePoses.relativePoses.Translations.col(0).setZero();

        SparsePoses.relative2global();

        for (auto &point : globalPoints)
        {
            if (point.isStatic > 0)
                point.getVector3fMap() = point.getVector3fMap() - newOrigin.cast<float>();
        }
    }
    void decentralize()
    {
        SparsePoses.global2relative();
        SparsePoses.relativePoses.Translations.col(0) = newOrigin;
        SparsePoses.relative2global();

        for (auto &point : globalPoints)
        {
            if (point.isStatic > 0)
                point.getVector3fMap() = point.getVector3fMap() + newOrigin.cast<float>();
        }
    }

    Eigen::VectorXd &getAdditionalErrorTerms()
    {
        return imuFactorError;
    }

    int updateAdditionalErrors()
    {
        if (useImuErrorTerms)
        {
            updateImuError();

            return imuFactorError.size();
        }
        else
            return 0;
    }

    void getPoseParameters(VectorXd &params)
    {
        SparsePoses.relativePoses.getParamsAsVector(params);
    }

    void setPoseParameters(const Eigen::VectorXd &params)
    {
        SparsePoses.relativePoses.setParamsFromVector(params);
    }

    void updateGlobalPoints()
    {
        // update dense transforms to global frame
        updateTrajDenseTforms();

        // transform points to global cloud
        int currId = 0;

        for (int pcId = 0; pcId < regPcBuffer->getNumElements(); ++pcId)
        {
            PointCloudPlus &currCloud = regPcBuffer->at(pcId);

            for (int k = 0; k < currCloud.size(); ++k)
            {
                PointStampId &pointOriginRef = currCloud.points[k];

                PointStampId &pointTargetRef = globalPoints.points[currId];

                int &currTformId = tformIdPerPoint[pcId][k];

                Eigen::Map<Vector4f> targetMap(pointTargetRef.data);

                targetMap = DenseTformsLocal2Global[currTformId] * pointOriginRef.getVector4fMap();

                ++currId;
            }
        }
    }

    void addStaticPoints(const pcl::PointCloud<PointStampId> &pcStatic)
    {
        int currSz = globalPoints.points.size();

        globalPoints.points.resize(currSz + static_cast<int>(pcStatic.size()));

        for (int k = currSz; k < globalPoints.points.size(); ++k)
        {
            globalPoints.points[k].getVector3fMap() = pcStatic.points[k - currSz].getVector3fMap();

            globalPoints.points[k].stamp = -1000.0;
            globalPoints.points[k].id = pcStatic.points[k - currSz].id;
            globalPoints.points[k].isStatic = 1;
        }
    }

    void removeStaticPoints()
    {
        if (globalPoints.points.size() == 0)
            return;

        for (int k = 0; k < globalPoints.points.size(); ++k)
        {
            if (globalPoints.points[k].isStatic > 0)
            {
                globalPoints.points.resize(k);
                return;
            }
        }
    }

    void updateTrajDenseTforms()
    {
        SparsePoses.relative2global();

        // interpolate orientations with slerp
        for (int k = 0; k < n_total; ++k)
        {
            // rotational interpolation
            getInterpRotation(SparsePoses.globalPoses, SparsePoses.stamps, trajTime(k), DenseGlobalPoses.Orientations.col(k));
        }

        // interpolation of translation
        for (int k = 0; k < 3; ++k)
        {

            // Map translations to std vec
            Eigen::VectorXd translations = SparsePoses.globalPoses.Translations.row(k);
            Map<VectorXd> TranslationsMap(translations.data(), translations.size());
            std::vector<double> vec_translations(TranslationsMap.data(), TranslationsMap.data() + TranslationsMap.size());

            // Map XYZ to std vec
            Map<VectorXd> paramStampsMap(SparsePoses.stamps.data(), SparsePoses.stamps.size());
            std::vector<double> vec_paramStamps(paramStampsMap.data(), paramStampsMap.data() + paramStampsMap.size());

            // interpolation
            boost::math::barycentric_rational<double> s(vec_paramStamps.data(), vec_translations.data(), vec_paramStamps.size(), 2);

            for (int j = 0; j < n_total; ++j)
                DenseGlobalPoses.Translations(k, j) = (double)s(trajTime(j));
        }

        // update dense transforms
        for (int k = 0; k < n_total; ++k)
        {
            DenseTformsLocal2Global[k].block(0, 0, 3, 3) = axang2rotm(DenseGlobalPoses.Orientations.col(k)).cast<float>();
            DenseTformsLocal2Global[k].block(0, 3, 3, 1) = DenseGlobalPoses.Translations.col(k).cast<float>();
        }
    }

    void registerPcBuffer(std::shared_ptr<PointCloudBuffer> &bufferPtrIn)
    {
        regPcBuffer = bufferPtrIn;

        globalPoints.resize(regPcBuffer->getNumPoints());

        // save minimum grid size for optimization
        this->minGridSize = std::numeric_limits<float>::max();

        for (int k = 0; k < regPcBuffer->getNumElements(); ++k)
            this->minGridSize = std::min(this->minGridSize, regPcBuffer->at(k).gridSize);

        // precalculate corresponding tform index for each point
        tformIdPerPoint.resize(regPcBuffer->getNumElements());

        int globalId = 0;

        for (int pcId = 0; pcId < regPcBuffer->getNumElements(); ++pcId)
        {
            PointCloudPlus &currCloud = regPcBuffer->at(pcId);

            tformIdPerPoint[pcId].resize(currCloud.size());

            for (int k = 0; k < currCloud.size(); ++k)
            {
                auto pointerToVal = std::lower_bound(this->trajTime.data(), this->trajTime.data() + this->trajTime.size(), currCloud.points[k].stamp - this->t0);
                tformIdPerPoint[pcId][k] = std::min((int)std::distance(this->trajTime.data(), pointerToVal), static_cast<int>(this->n_total - 1));

                // copy point to save additional information
                globalPoints.points[globalId] = currCloud.points[k];
                ++globalId;
            }
        }
    }

    void initGravityDir()
    {
        // Vector3d measuredGravity = accMeas.rowwise().mean();
        Vector3d measuredGravity = accMeas.col(0);

        Vector3d v1, v2;

        // init gravity
        v1 = gravity;
        v2 = -1.0 * measuredGravity;

        cout << "Measured gravity norm at initialization: " << measuredGravity.norm() << " [m/s2] " << endl;

        // Find the normalized cross product of the two vectors
        Vector3d axis = v1.cross(v2).normalized();

        // Find the angle between the two vectors
        double angle = acos(v1.dot(v2) / (v1.norm() * v2.norm()));

        // Construct the rotation matrix using Rodrigues' rotation formula
        Matrix3d K;
        K << 0.0, -axis.z(), axis.y(),
            axis.z(), 0.0, -axis.x(),
            -axis.y(), axis.x(), 0.0;
        Matrix3d R_to_grav = Matrix3d::Identity() + sin(angle) * K + (1 - cos(angle)) * K * K;

        AngleAxisd angleAxis(R_to_grav.transpose());

        Vector3d normedAxisAngle = angleAxis.angle() * angleAxis.axis();

        SparsePoses.relativePoses.Orientations.col(0) = normedAxisAngle;

        // update global params
        SparsePoses.relative2global();

        cout << "Estimated gravity direction: " << normedAxisAngle.transpose() << endl;
    }

    void initTraj(double t_min, double t_max, int numControlPoints, bool useImu, double dtResIn)
    {

        // init
        dt_res = dtResIn;
        useImuErrorTerms = useImu;

        t0 = t_min;
        horizon = t_max - t_min + dt_res;
        n_total = round(horizon / dt_res) + 1;

        // init data structures
        DenseTformsLocal2Global.resize(n_total);

        for (int k = 0; k < n_total; ++k)
            DenseTformsLocal2Global[k] = Matrix4f::Identity();

        accMeas.conservativeResize(3, n_total);
        angVelMeas.conservativeResize(3, n_total);

        DenseGlobalPoses.resize(n_total);

        trajTime = VectorXd::LinSpaced(n_total, 0.0, horizon);

        // calc total number of parameters
        numParams = numControlPoints;

        // set number of parameters
        SparsePoses = StampedConsecutivePoses(numParams);

        // init stamps of sparse poses
        SparsePoses.stamps = VectorXd::LinSpaced(numParams, 0.0, horizon);

        // init param indices
        paramIndices.resize(numParams);
        paramIndices = (SparsePoses.stamps / dt_res).array().round().matrix().cast<int>();

        preintImuRots.resize(numParams);
        preintRelPositions.resize(numParams);
        preintRelVelocity.resize(numParams);
        CovPVRot_inv.resize(numParams);

        imuFactorError.resize((numParams - 1));

        gravity << 0.0, 0.0, -9.805;
    }

    void transferImuMeasurements(ImuBuffer &imuBuffer)
    {

        double timediff;

        for (int k = 0; k < n_total; ++k)
        {
            double currGlobalTime = t0 + trajTime(k);

            timediff = imuBuffer.getClosestMeasurement(currGlobalTime, accMeas.col(k), angVelMeas.col(k));

            if (abs(timediff) > 0.1)
            {
                cout << "Traj timediff to closest imu measurement is: " << timediff << " [s] ! \n"
                     << endl;
            }
        }
    }

    void updateInitialGuess(bool &isInitialized, ContinuousTrajectory &oldTraj, bool useImu)
    {
        int lastKnownParamId = 0;

        if (!isInitialized)
        {
            // initialize gravity dir
            if (useImu)
                initGravityDir();

            isInitialized = true;

            return;
        }

        // update global parameter set
        oldTraj.SparsePoses.relative2global();

        for (int k = 0; k < SparsePoses.stamps.size(); ++k)
        {
            if (t0 + SparsePoses.stamps(k) < oldTraj.t0 + oldTraj.horizon)
                lastKnownParamId = k;
        }

        // get orientation for control poses from old trajectory by interpolation
        for (int k = 0; k <= lastKnownParamId; ++k)
        {
            getInterpRotation(oldTraj.SparsePoses.globalPoses, oldTraj.SparsePoses.stamps, SparsePoses.stamps(k) + t0 - oldTraj.t0, SparsePoses.globalPoses.Orientations.col(k));
        }

        // get position of control poses from old trajectory by cubic interpolation
        Vector3d v0;

        for (int k = 0; k < 3; ++k)
        {

            // Map translations to std vec
            VectorXd translations = oldTraj.SparsePoses.globalPoses.Translations.row(k);
            Map<VectorXd> TranslationsMap(translations.data(), translations.size());
            vector<double> vec_translations(TranslationsMap.data(), TranslationsMap.data() + TranslationsMap.size());

            // Map XYZ to std vec
            Map<VectorXd> paramStampsMap(oldTraj.SparsePoses.stamps.data(), oldTraj.SparsePoses.stamps.size());
            vector<double> vec_paramStamps(paramStampsMap.data(), paramStampsMap.data() + paramStampsMap.size());

            // spline interpolation
            boost::math::barycentric_rational<double> s(vec_paramStamps.data(), vec_translations.data(), vec_paramStamps.size(), 2);

            for (int j = 0; j <= lastKnownParamId; ++j)
                SparsePoses.globalPoses.Translations(k, j) = (double)s(SparsePoses.stamps(j) + t0 - oldTraj.t0);

            // numeric differentation for velocity
            v0(k) = s.prime(SparsePoses.stamps(lastKnownParamId) + t0 - oldTraj.t0);
        }

        // update relative parameter set
        SparsePoses.global2relative();

        if (useImu)
        {

            Vector3d pos0 = SparsePoses.globalPoses.Translations.col(lastKnownParamId);
            Vector3d axang0 = SparsePoses.globalPoses.Orientations.col(lastKnownParamId);
            double t0, tend;

            Vector3d axang_end, pos_end, v_end;

            // update params

            for (int k = lastKnownParamId; k < numParams - 1; ++k)
            {
                t0 = SparsePoses.stamps(k);

                tend = SparsePoses.stamps(k + 1);

                // integrate parameters
                getImuIntegratedParams(t0, axang0, pos0, v0, tend, axang_end, pos_end, v_end);

                // save parameters
                SparsePoses.globalPoses.Orientations.col(k + 1) = axang_end;
                SparsePoses.globalPoses.Translations.col(k + 1) = pos_end;

                axang0 = axang_end;
                pos0 = pos_end;
                v0 = v_end;
            }

            // update relative parameter set
            SparsePoses.global2relative();
        }
        else
        {
            // predict const vel / angular vel
            for (int k = lastKnownParamId; k < numParams - 1; ++k)
            {
                SparsePoses.relativePoses.Orientations.col(k + 1) = SparsePoses.relativePoses.Orientations.col(lastKnownParamId);
                SparsePoses.relativePoses.Translations.col(k + 1) = SparsePoses.relativePoses.Translations.col(lastKnownParamId);
            }

            // update relative parameters
            SparsePoses.relative2global();
        }
    }

    void getImuIntegratedParams(double t0, Vector3d axang0, Vector3d pos0, Vector3d v0,
                                double tend, Ref<Vector3d> axang_end, Ref<Vector3d> pos_end, Ref<Vector3d> v_end)
    {

        // integrate position and orientation within t0 and tend

        // get closest imu measurement index
        auto pointerToVal = lower_bound(trajTime.data(), trajTime.data() + trajTime.size() - 1, t0);
        auto index = distance(trajTime.data(), pointerToVal);

        double prev = *(pointerToVal - 1);
        double next = *pointerToVal;

        if (abs(t0 - prev) < abs(t0 - next))
            index = index - 1;

        // init variables
        Matrix3d R_imu2w = axang2rotm(axang0);
        Vector3d vel_w, pos_w;

        pos_w = pos0;
        vel_w = v0;

        double dt_res2 = dt_res * dt_res;
        double currTime = t0;

        // integration loop
        while (abs(currTime + dt_res - tend) < abs(currTime - tend) && index < n_total)
        {
            // update world position
            pos_w = pos_w + vel_w * dt_res + 0.5 * gravity * dt_res2 + 0.5 * R_imu2w * accMeas.col(index) * dt_res2;

            // update world velocity
            vel_w = vel_w + gravity * dt_res + R_imu2w * accMeas.col(index) * dt_res;

            // update rotation
            R_imu2w = R_imu2w * axang2rotm(dt_res * angVelMeas.col(index));

            // update indices
            index += 1;
            currTime += dt_res;
        }

        // save final parameters
        axang_end = rotm2axang(R_imu2w);
        pos_end = pos_w;
        v_end = vel_w;
    }

    void updatePreintFactors(const Ref<Matrix3d> gyr_cov, const Ref<Matrix3d> acc_cov)
    {
        ImuPreintegration imuPreintegration;

        preintImuRots[0] = Matrix3d::Identity();
        preintRelPositions[0].setZero();
        preintRelVelocity[0].setZero();

        int fromId, toId;

        Matrix<double, 9, 9> covMat;

        for (int k = 1; k < SparsePoses.numPoses; ++k)
        {
            fromId = paramIndices(k - 1);
            toId = paramIndices(k);

            // reset
            imuPreintegration.reset();

            // preintegration loop
            for (int t = fromId; t < toId; ++t)
            {
                // update preintegration factors
                imuPreintegration.addMeasurement(angVelMeas.col(t), accMeas.col(t), dt_res, gyr_cov, acc_cov);
            }

            // save preint factors
            preintImuRots[k] = imuPreintegration.getDeltaRot();
            preintRelPositions[k] = imuPreintegration.getDeltaPos();
            preintRelVelocity[k] = imuPreintegration.getDeltaVel();
            covMat = imuPreintegration.getCov_rot_v_p();
            CovPVRot_inv[k] = covMat.inverse();
        }

        // preintegrate over complete horizon

        // reset
        imuPreintegration.reset();

        // preintegration loop
        for (int t = 0; t < angVelMeas.cols(); ++t)
        {
            // update preintegration factors
            imuPreintegration.addMeasurement(angVelMeas.col(t), accMeas.col(t), dt_res, gyr_cov, acc_cov);
        }

        preintPosComplHor = imuPreintegration.getDeltaPos();
    }

    void getInterpRotation(const Poses &posesGlobal, const VectorXd &stamps, const double &t, Ref<Vector3d> axangNew)
    {
        // find corresponding index
        auto pointerToVal = lower_bound(stamps.data(), stamps.data() + stamps.size() - 1, t);

        auto rightIndex = distance(stamps.data(), pointerToVal);
        double t_rel;

        if (rightIndex != 0)
            t_rel = (t - stamps(rightIndex - 1)) / (stamps(rightIndex) - stamps(rightIndex - 1));
        else
            t_rel = 1.0f;

        if (rightIndex > 0)
        {
            axangNew = slerp(posesGlobal.Orientations.col(rightIndex - 1), posesGlobal.Orientations.col(rightIndex), t_rel);
        }
        else
        {
            axangNew = posesGlobal.Orientations.col(rightIndex);
        }
    }

    void getSubmapGravityEstimate(Vector3d &gravity_imu)
    {
        // sum up preint positions
        double one_div_t_res = 1.0 / dt_res;
        Vector3d v_start_w = one_div_t_res * (DenseGlobalPoses.Translations.col(1) - DenseGlobalPoses.Translations.col(0));
        Matrix3d R_imu2w_start = axang2rotm(SparsePoses.globalPoses.getFirstOrientation());

        gravity_imu = (R_imu2w_start.transpose() * (SparsePoses.globalPoses.getLastTranslation() - SparsePoses.globalPoses.getFirstTranslation() - v_start_w * horizon) - preintPosComplHor) / (0.5 * pow(horizon, 2));
    }

    void updateImuError()
    {
        /* preint ACCELERATION POS ERROR */
        SparsePoses.global2relative();

        // reset
        imuFactorError.setZero();

        // init
        Vector3d pos_error, rot_error;
        VectorXd combined_error(9);
        combined_error.setZero();

        Vector3d v_start_w, delta_p_model, v_end_world, delta_v_model, vel_error;
        Matrix3d R_imu2w_start;
        Matrix3d R_imu2w_end, R_tmp;
        double delta_t;

        double one_div_t_res = 1.0 / dt_res;

        // cout <<"\n\n"<<endl;

        for (int k = 1; k < paramIndices.size(); ++k)
        {

            // calculate position error
            R_imu2w_start = axang2rotm(SparsePoses.globalPoses.Orientations.col(k - 1));

            delta_t = SparsePoses.stamps(k) - SparsePoses.stamps(k - 1);

            // calc start velocity
            v_start_w = one_div_t_res * (DenseGlobalPoses.Translations.col(paramIndices(k - 1) + 1) - DenseGlobalPoses.Translations.col(paramIndices(k - 1)));

            // calc end velocity
            v_end_world = one_div_t_res * (DenseGlobalPoses.Translations.col(paramIndices(k)) - DenseGlobalPoses.Translations.col(paramIndices(k) - 1));

            delta_p_model = R_imu2w_start.transpose() * (SparsePoses.globalPoses.Translations.col(k) - SparsePoses.globalPoses.Translations.col(k - 1) - v_start_w * delta_t - 0.5 * pow(delta_t, 2) * gravity);

            // + R_imu2w_start*
            pos_error = delta_p_model - preintRelPositions[k];

            // calculate rotation error
            R_imu2w_end = axang2rotm(SparsePoses.relativePoses.Orientations.col(k));
            R_tmp = preintImuRots[k].transpose() * R_imu2w_end;

            rot_error = rotm2axang(R_tmp);

            // calculate velocity error
            delta_v_model = R_imu2w_start.transpose() * (v_end_world - v_start_w - gravity * delta_t);

            vel_error = delta_v_model - preintRelVelocity[k];

            // save error
            combined_error << rot_error, vel_error, pos_error;

            // save
            imuFactorError(k - 1) = combined_error.transpose() * CovPVRot_inv[k] * combined_error;

            imuFactorError(k - 1) *= balancingImu;
        }
    }

    const int &getIdOfPoint(int &globalPointIndex)
    {
        return globalPoints.points[globalPointIndex].id;
    }
};

#endif
