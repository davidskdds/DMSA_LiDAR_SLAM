/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#ifndef OUTPUTMANAGEMENT_H
#define OUTPUTMANAGEMENT_H

#include <eigen3/Eigen/Core>
#include "ConsecutivePoses.h"
#include "helpers.h"

using namespace Eigen;
using namespace std;

struct nonKeyframePose
{
    Vector3d Translation;
    Vector3d Orientation;
    double Stamp;
    bool relativeToKeyframe = true;
    int relatedKeyframeId;
};

class OutputManagement
{
public:
    StampedPoses staticKeyframePoses;
    vector<nonKeyframePose> nonKeyframePoses;

    vector<bool> orderIsKey;

    int overallCounter = 0;
    int staticKeyframeCounter = 0;
    int keyframeCounter = 0;
    int nonKeyframeCounter = 0;

    int maxNum;

    OutputManagement(int n_max = 200000) : staticKeyframePoses(n_max), maxNum(n_max), orderIsKey(n_max), nonKeyframePoses(n_max)
    {
    }
    void addStaticKeyframePose(Ref<Vector3d> transl, Ref<Vector3d> orient, double &stamp)
    {
        staticKeyframePoses.Translations.col(staticKeyframeCounter) = transl;
        staticKeyframePoses.Orientations.col(staticKeyframeCounter) = orient;
        staticKeyframePoses.Stamps(staticKeyframeCounter) = stamp;

        ++staticKeyframeCounter;
    }

    void informAboutNewKeyframe()
    {
        orderIsKey[overallCounter] = true;

        ++keyframeCounter;
        ++overallCounter;
    }

    void addNonKeyframePose(Ref<Vector3d> translFromKeyframe, Ref<Vector3d> orientFromKeyframe, double &stamp, int keyframeId)
    {
        nonKeyframePoses[nonKeyframeCounter].Orientation = orientFromKeyframe;
        nonKeyframePoses[nonKeyframeCounter].Translation = translFromKeyframe;
        nonKeyframePoses[nonKeyframeCounter].Stamp = stamp;
        nonKeyframePoses[nonKeyframeCounter].relatedKeyframeId = keyframeId + staticKeyframeCounter;

        orderIsKey[overallCounter] = false;

        ++nonKeyframeCounter;
        ++overallCounter;
    }

    void saveKeyframePoses()
    {
    }

    void addPoseToFile(double &stamp, Ref<Vector3d> pos, Ref<Vector3d> orient, std::ofstream &file)
    {
        // save timestamp
        file << std::setprecision(6) << std::fixed << stamp << " ";

        // save translation parameters
        file << std::setprecision(5) << std::fixed << pos(0) << " " << pos(1) << " " << pos(2) << " ";

        // calculate quaternion and save it
        Matrix3d R = axang2rotm(orient);
        Quaterniond q(R);

        file << std::setprecision(6) << std::fixed << q.x() << " " << q.y() << " " << q.z() << " " << q.w();

        // new row
        file << "\n";
    }

    void saveDensePoses(StampedConsecutivePoses activeKeyframePoses, std::string result_dir, std::string prefix = "")
    {
        std::ofstream pose_file;
        std::string PosesFileName;

        PosesFileName = result_dir + "/Poses" + prefix + ".txt";
        pose_file.open(PosesFileName);

        int keyId = 0;
        int nonKeyId = 0;

        double stamp;
        Matrix3d keyRot;
        Vector3d globalPos;
        Vector3d globalOrient;

        for (int k = 0; k < overallCounter; ++k)
        {
            if (orderIsKey[k] == true)
            {
                // add keyframe
                if (keyId < staticKeyframeCounter)
                {
                    stamp = staticKeyframePoses.Stamps(keyId);

                    Ref<Vector3d> pos = staticKeyframePoses.Translations.col(keyId);
                    Ref<Vector3d> orient = staticKeyframePoses.Orientations.col(keyId);

                    addPoseToFile(stamp, pos, orient, pose_file);
                }
                else
                {
                    if (keyId - staticKeyframeCounter >= activeKeyframePoses.stamps.size())
                        cerr << "Error in file creation: keyId-staticKeyframeCounter>=activeKeyframePoses.stamps.size() \n";

                    stamp = activeKeyframePoses.stamps(keyId - staticKeyframeCounter);
                    Ref<Vector3d> pos = activeKeyframePoses.globalPoses.Translations.col(keyId - staticKeyframeCounter);
                    Ref<Vector3d> orient = activeKeyframePoses.globalPoses.Orientations.col(keyId - staticKeyframeCounter);

                    addPoseToFile(stamp, pos, orient, pose_file);
                }

                ++keyId;
            }
            else
            {
                // add non-keyframe
                if (nonKeyframePoses[nonKeyId].relativeToKeyframe == true && nonKeyframePoses[nonKeyId].relatedKeyframeId < staticKeyframeCounter)
                    makeNonKeyframePoseGlobal(nonKeyId);

                // update stamp
                stamp = nonKeyframePoses[nonKeyId].Stamp;

                if (nonKeyframePoses[nonKeyId].relativeToKeyframe == true)
                {
                    if (nonKeyframePoses[nonKeyId].relatedKeyframeId - staticKeyframeCounter >= activeKeyframePoses.stamps.size())
                        cerr << "Error in file creation: nonKeyframePoses[nonKeyId].relatedKeyframeId-staticKeyframeCounter >= activeKeyframePoses.stamps.size() \n";

                    Ref<Vector3d> keyframePos = activeKeyframePoses.globalPoses.Translations.col(nonKeyframePoses[nonKeyId].relatedKeyframeId - staticKeyframeCounter);
                    Ref<Vector3d> keyframeOrient = activeKeyframePoses.globalPoses.Orientations.col(nonKeyframePoses[nonKeyId].relatedKeyframeId - staticKeyframeCounter);

                    keyRot = axang2rotm(keyframeOrient);

                    globalPos = keyRot * nonKeyframePoses[nonKeyId].Translation + keyframePos;
                    globalOrient = rotm2axang(keyRot * axang2rotm(nonKeyframePoses[nonKeyId].Orientation));

                    addPoseToFile(stamp, globalPos, globalOrient, pose_file);
                }
                else
                {
                    Ref<Vector3d> pos = nonKeyframePoses[nonKeyId].Translation;
                    Ref<Vector3d> orient = nonKeyframePoses[nonKeyId].Orientation;

                    addPoseToFile(stamp, pos, orient, pose_file);
                }

                ++nonKeyId;
            }
        }

        pose_file.flush();
        pose_file.close();
    }

    void makeNonKeyframePoseGlobal(int &nonKeyId)
    {
        Ref<Vector3d> keyframePos = staticKeyframePoses.Translations.col(nonKeyframePoses[nonKeyId].relatedKeyframeId);
        Ref<Vector3d> keyframeOrient = staticKeyframePoses.Orientations.col(nonKeyframePoses[nonKeyId].relatedKeyframeId);

        Matrix3d keyRot = axang2rotm(keyframeOrient);

        nonKeyframePoses[nonKeyId].Translation = keyRot * nonKeyframePoses[nonKeyId].Translation + keyframePos;
        nonKeyframePoses[nonKeyId].Orientation = rotm2axang(keyRot * axang2rotm(nonKeyframePoses[nonKeyId].Orientation));

        nonKeyframePoses[nonKeyId].relativeToKeyframe = false;
    }
};

#endif
