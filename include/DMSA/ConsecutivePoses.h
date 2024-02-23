/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#include "Poses.h"
#include "helpers.h"

#ifndef CONSECUTIVEPOSES_H
#define CONSECUTIVEPOSES_H

class ConsecutivePoses
{
public:
    Poses relativePoses;
    Poses globalPoses;
    int numPoses = 0;

    ConsecutivePoses(int n) : relativePoses(n), globalPoses(n), numPoses(n) {}

    ConsecutivePoses() {}

    void relative2global()
    {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d T(0.0, 0.0, 0.0);

        for (int k = 0; k < numPoses; ++k)
        {
            // update translation
            T = T + R * relativePoses.Translations.col(k);
            globalPoses.Translations.col(k) = T;

            // update rotation
            R = R * axang2rotm(relativePoses.Orientations.col(k));

            // save global rotation
            globalPoses.Orientations.col(k) = rotm2axang(R);
        }
    }

    void global2relative()
    {
        // copy init values
        relativePoses.Orientations.col(0) = globalPoses.Orientations.col(0);
        relativePoses.Translations.col(0) = globalPoses.Translations.col(0);

        // init params
        Eigen::Matrix3d R1, R2;
        Eigen::Vector3d T1, T2;

        for (int k = numPoses - 1; k > 0; --k)
        {
            R1 = axang2rotm(globalPoses.Orientations.col(k - 1));
            T1 = globalPoses.Translations.col(k - 1);

            R2 = axang2rotm(globalPoses.Orientations.col(k));
            T2 = globalPoses.Translations.col(k);

            // save relative rotation and translations
            relativePoses.Orientations.col(k) = rotm2axang(R1.transpose() * R2);
            relativePoses.Translations.col(k) = R1.transpose() * (T2 - T1);
        }
    }

    void resize(int n)
    {
        relativePoses.resize(n);
        globalPoses.resize(n);
        numPoses = n;
    }
};

class StampedConsecutivePoses : public ConsecutivePoses
{
public:
    VectorXd stamps;

    StampedConsecutivePoses() {}

    StampedConsecutivePoses(int n) : ConsecutivePoses(n), stamps(n)
    {
        stamps.setZero();
    }

    void resize(int n)
    {
        relativePoses.resize(n);
        globalPoses.resize(n);

        stamps.conservativeResize(n);
    }
};

#endif
