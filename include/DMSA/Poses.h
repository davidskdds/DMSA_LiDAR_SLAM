/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include <eigen3/Eigen/Core>

#ifndef POSES_H
#define POSES_H

using namespace Eigen;

class Poses
{
public:
    Matrix3Xd Orientations;
    Matrix3Xd Translations;

    Poses(int n)
    {
        Orientations.resize(3, n);
        Translations.resize(3, n);

        Orientations.setZero();
        Translations.setZero();
    }

    Poses() {}

    void resize(int n)
    {
        Orientations.conservativeResize(3, n);
        Translations.conservativeResize(3, n);
    }

    Vector3d getFirstTranslation()
    {
        return Translations.col(0);
    }

    Vector3d getFirstOrientation()
    {
        return Orientations.col(0);
    }

    Vector3d getLastTranslation()
    {
        return Translations.col(Translations.cols() - 1);
    }

    Vector3d getLastOrientation()
    {
        return Orientations.col(Orientations.cols() - 1);
    }

    int getNumElemsVector()
    {
        return (Translations.cols() - 1) * 6;
    }

    void getParamsAsVector(VectorXd &params)
    {
        params.conservativeResize(Orientations.cols() * Orientations.rows() + Translations.cols() * Translations.rows() - 6);

        params << Orientations.block(0, 1, 3, Orientations.cols() - 1).reshaped(Orientations.cols() * Orientations.rows() - 3, 1),
            Translations.block(0, 1, 3, Translations.cols() - 1).reshaped(Translations.cols() * Translations.rows() - 3, 1);
    }

    void setParamsFromVector(const VectorXd &params)
    {
        Orientations.block(0, 1, 3, Orientations.cols() - 1) = params.head(Orientations.cols() * Orientations.rows() - 3).reshaped(Orientations.rows(), Orientations.cols() - 1);
        Translations.block(0, 1, 3, Translations.cols() - 1) = params.tail(Translations.cols() * Translations.rows() - 3).reshaped(Translations.rows(), Translations.cols() - 1);
    }

    void getParamsAsVectorFull(VectorXd &params)
    {
        params.conservativeResize(Orientations.cols() * Orientations.rows() + Translations.cols() * Translations.rows());

        params << Orientations.block(0, 0, 3, Orientations.cols()).reshaped(Orientations.cols() * Orientations.rows(), 1),
            Translations.block(0, 0, 3, Translations.cols()).reshaped(Translations.cols() * Translations.rows(), 1);
    }

    void setParamsFromVectorFull(const VectorXd &params)
    {
        Orientations.block(0, 0, 3, Orientations.cols()) = params.head(Orientations.cols() * Orientations.rows()).reshaped(Orientations.rows(), Orientations.cols());
        Translations.block(0, 0, 3, Translations.cols()) = params.tail(Translations.cols() * Translations.rows()).reshaped(Translations.rows(), Translations.cols());
    }
};

class StampedPoses : public Poses
{
public:
    VectorXd Stamps;

    StampedPoses(int n) : Poses(n)
    {
        Stamps.resize(n);
    }

    void resize(int n)
    {
        Orientations.conservativeResize(3, n);
        Translations.conservativeResize(3, n);

        Stamps.conservativeResize(n);
    }
};

#endif
