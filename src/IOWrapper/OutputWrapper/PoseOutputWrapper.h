/**
 * This file is part of DSO.
 *
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include "IOWrapper/Output3DWrapper.h"
#include "boost/thread.hpp"
#include "util/MinimalImage.h"

#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"
#include <Eigen/Core>
#include <fstream>
#include <iomanip>
#include <string>

namespace dso {

class FrameHessian;
class CalibHessian;
class FrameShell;

namespace IOWrap {

class PoseOutputWrapper : public Output3DWrapper {
public:
    inline PoseOutputWrapper(const std::string &filename) : maxKFIDsofar(0) {
        // T_rotateImuToCam << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1., 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        // tr = RrotateWorld * tr;
        // R = RrotateWorld * R;
        // cv::Mat RrotateWorld = (cv::Mat_<double>(3, 3) << 0., 0., 1., -1., 0., 0., 0., -1., 0.);

        rpgFilename = filename;
        std::string dirname = filename.substr(0, filename.find_last_of('/'));
        printf("trying to create directory %s\n", dirname.c_str());
        std::string mkdirCmd = "mkdir -p " + dirname;
        std::system(mkdirCmd.c_str());

        outfile.open(filename);
        if (!outfile.is_open()) {
            fprintf(stderr, "Couldnt open rpgfile: %s", filename.c_str());
            return;
        }
        outfile << "# Timestamp tx ty tz qx qy qz qw" << std::endl;
        outfile << std::fixed;
        outfile.close();
        printf("OUT: Created PoseOutputWrapper\n");
    }

    virtual ~PoseOutputWrapper() { printf("OUT: Destroyed PoseOutputWrapper\n"); }

    virtual void publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>,
                                             Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) override {
    }

    virtual void publishKeyframes(std::vector<FrameHessian *> &frames, bool final, CalibHessian *HCalib) override {
        if (final == true) {
            for (FrameHessian *f : frames) {
                if (maxKFIDsofar > f->shell->incoming_id) {
                    return;
                } else {
                    // printf("OUT: KF %d (%s) (id %d, tme %f): %d active, %d marginalized, %d immature points. CameraToWorld:\n",
                    // f->frameID,
                    //        final ? "final" : "non-final", f->shell->incoming_id, f->shell->timestamp, (int)f->pointHessians.size(),
                    //        (int)f->pointHessiansMarginalized.size(), (int)f->immaturePoints.size());
                    std::cout << f->shell->camToWorld.matrix3x4() << "\n";
                    outfile.open(rpgFilename, std::ios_base::app);
                    auto worldToCam = f->shell->camToWorld;
                    auto t = worldToCam.translation();
                    auto q = worldToCam.unit_quaternion();
                    printf("OUT: F %d, ts %f, pos %f, %f, %f, quat %f, %f, %f, %f\n", f->shell->incoming_id, f->shell->timestamp, t.x(),
                           t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
                    outfile << std::setprecision(6) << f->shell->timestamp << std::setprecision(16) << " " << t.x() << " " << t.y() << " "
                            << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
                    outfile.close();
                    maxKFIDsofar = f->shell->incoming_id;
                }
            }
        }
    }

    virtual void publishCamPose(FrameShell *frame, CalibHessian *HCalib) override {
        // outfile.open(rpgFilename, std::ios_base::app);
        // auto t = frame->camToWorld.translation();
        // auto q = frame->camToWorld.unit_quaternion();
        // printf("OUT: F %d, ts %f, pos %f, %f, %f, quat %f, %f, %f, %f\n", frame->incoming_id, currentTimestamp, t.x(), t.y(), t.z(),
        // q.x(),
        //        q.y(), q.z(), q.w());
        // outfile << currentTimestamp << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << "
        // "
        //         << q.w() << std::endl;
        // currentTimestamp += fakeTimestamp;
        // outfile.close();
        // ++timesCalled;
    }

    virtual void pushLiveFrame(FrameHessian *image) override {
        // can be used to get the raw image / intensity pyramid.
    }

    virtual void pushDepthImage(MinimalImageB3 *image) override {
        // can be used to get the raw image with depth overlay.
    }
    virtual bool needPushDepthImage() override { return false; }

    virtual void pushDepthImageFloat(MinimalImageF *image, FrameHessian *KF) override {}

private:
    std::ofstream outfile;
    std::string rpgFilename;
    int maxKFIDsofar;
};

} // namespace IOWrap

} // namespace dso
