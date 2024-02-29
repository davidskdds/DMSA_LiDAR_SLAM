/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#ifndef IMUPREINTEGRATION_H
#define IMUPREINTEGRATION_H

#include <eigen3/Eigen/Core>
#include "helpers.h"

using namespace Eigen;

// this class performs IMU preintegration and covariance propagation according to "IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation"
// and "Supplementary Material to: IMU Preintegration on Manifold for Ecient Visual-Inertial Maximum-a-Posteriori Estimation" from Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza
// note: the bias update part is not included
// https://rpg.ifi.uzh.ch/docs/RSS15_Forster.pdf

class ImuPreintegration
{

private:
    Vector3d deltaPos;
    Vector3d deltaVel;
    Matrix3d deltaRot;

    Matrix<double, 9, 9> cov_rot_v_p;
    Matrix<double, 9, 9> A;
    Matrix<double, 9, 6> B;
    Matrix<double, 6, 6> MeasurementNoise;

    Matrix3d getJacobianR(const Vector3d rot)
    {
        double rotNorm = rot.norm();

        Matrix3d skewRot = skew(rot);

        if (rotNorm < EPSILON_ROT)
            return Matrix3d::Identity();

        return Matrix3d::Identity() - ((1.0 - cos(rotNorm)) / pow(rotNorm, 2)) * skewRot + ((rotNorm - sin(rotNorm)) / pow(rotNorm, 3)) * skewRot * skewRot;
    }

public:
    ImuPreintegration()
    {
        reset();
    }

    void addMeasurement(const Ref<Vector3d> omega, const Ref<Vector3d> acc, const double &dt, const Ref<Eigen::Matrix3d> gyr_cov, const Ref<Eigen::Matrix3d> acc_cov)
    {
        double dt2 = dt * dt;

        Matrix3d rotIncr = axang2rotm(dt * omega);

        // prepare cov update step
        A.setIdentity();
        B.setZero();

        A.block(0, 0, 3, 3) = rotIncr.transpose();

        A.block(3, 0, 3, 3) = -1.0 * deltaRot * skew(acc) * dt;

        A.block(6, 0, 3, 3) = -0.5 * deltaRot * skew(acc) * dt2;

        A.block(6, 3, 3, 3) = dt * Matrix3d::Identity();

        B.block(0, 0, 3, 3) = getJacobianR(rotm2axang(deltaRot)) * dt;

        B.block(3, 3, 3, 3) = deltaRot * dt;

        B.block(6, 3, 3, 3) = 0.5 * deltaRot * dt2;

        // prepare measurement noise
        MeasurementNoise.block(0, 0, 3, 3) = gyr_cov;
        MeasurementNoise.block(3, 3, 3, 3) = acc_cov;

        // propagate covariance
        cov_rot_v_p = A * cov_rot_v_p * A.transpose() + B * MeasurementNoise * B.transpose();

        // integrate measurements

        // update position
        deltaPos += deltaVel * dt + 0.5 * deltaRot * acc * dt2;

        // update velocity
        deltaVel += deltaRot * acc * dt;

        // update rotation
        deltaRot = deltaRot * rotIncr;
    }

    Vector3d getDeltaPos()
    {
        return deltaPos;
    }

    Vector3d getDeltaVel()
    {
        return deltaVel;
    }

    Matrix3d getDeltaRot()
    {
        return deltaRot;
    }

    Matrix<double, 9, 9> getCov_rot_v_p()
    {
        return cov_rot_v_p;
    }

    void reset()
    {
        deltaPos.setZero();
        deltaVel.setZero();
        deltaRot.setIdentity();
        cov_rot_v_p.setZero();
        MeasurementNoise.setZero();
    }
};

#endif
