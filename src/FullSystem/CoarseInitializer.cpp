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

/*
 * KFBuffer.cpp
 *
 *  Created on: Jan 7, 2014
 *      Author: engelj
 */

#include "FullSystem/CoarseInitializer.h"
#include "FullSystem/FullSystem.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/PixelSelector.h"
#include "FullSystem/PixelSelector2.h"
#include "FullSystem/Residuals.h"
#include "util/nanoflann.h"

#if !defined(__SSE3__) && !defined(__SSE2__) && !defined(__SSE1__)
#include "SSE2NEON.h"
#endif

namespace dso {

CoarseInitializer::CoarseInitializer(int ww, int hh) : thisToNext_aff(0, 0), thisToNext(SE3()) {
    for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) {
        points[lvl] = 0;
        numPoints[lvl] = 0;
    }

    JbBuffer = new Vec10f[ww * hh];
    JbBuffer_new = new Vec10f[ww * hh];

    frameID = -1;
    fixAffine = true;
    printDebug = false;

    wM.diagonal()[0] = wM.diagonal()[1] = wM.diagonal()[2] = SCALE_XI_ROT;
    wM.diagonal()[3] = wM.diagonal()[4] = wM.diagonal()[5] = SCALE_XI_TRANS;
    wM.diagonal()[6] = SCALE_A;
    wM.diagonal()[7] = SCALE_B;
}
CoarseInitializer::~CoarseInitializer() {
    for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) {
        if (points[lvl] != 0)
            delete[] points[lvl];
    }

    delete[] JbBuffer;
    delete[] JbBuffer_new;
}

bool CoarseInitializer::trackFrame(FrameHessian *newFrameHessian, std::vector<IOWrap::Output3DWrapper *> &wraps) {
    newFrame = newFrameHessian;

    for (IOWrap::Output3DWrapper *ow : wraps)
        ow->pushLiveFrame(newFrameHessian);

    int maxIterations[] = {5, 5, 10, 30, 50};

    alphaK = 2.5 * 2.5; //*freeDebugParam1*freeDebugParam1;
    alphaW = 150 * 150; //*freeDebugParam2*freeDebugParam2;
    regWeight = 0.8;    //*freeDebugParam4;
    couplingWeight = 1; //*freeDebugParam5;

    if (!snapped) {
        thisToNext.translation().setZero();
        for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) {
            int npts = numPoints[lvl];
            Pnt *ptsl = points[lvl];
            for (int i = 0; i < npts; i++) {
                ptsl[i].iR = 1;
                ptsl[i].idepth_new = 1;
                ptsl[i].lastHessian = 0;
            }
        }
    }

    SE3 refToNew_current = thisToNext; //// usually unit rigid body motion
    AffLight refToNew_aff_current = thisToNext_aff;

    if (firstFrame->ab_exposure > 0 && newFrame->ab_exposure > 0)
        refToNew_aff_current = AffLight(logf(newFrame->ab_exposure / firstFrame->ab_exposure), 0); // coarse approximation.

    Vec3f latestRes = Vec3f::Zero();
    for (int lvl = pyrLevelsUsed - 1; lvl >= 0; lvl--) {

        if (lvl < pyrLevelsUsed - 1) //// propagate iR values from coarser to finer pyramid lvl, skip finest lvl
            propagateDown(lvl + 1);

        //// setup variables for Optimization
        Mat88f H, Hsc; //// sc == schur compliment i think
        Vec8f b, bsc;  //// sc == schur compliment i think
        resetPoints(lvl);
        Vec3f resOld = calcResAndGS(lvl, H, b, Hsc, bsc, refToNew_current, refToNew_aff_current, false);
        applyStep(lvl);

        float lambda = 0.1;
        float eps = 1e-4;
        int fails = 0;

        if (printDebug) {
            printf("lvl %d, it %d (l=%f) %s: %.3f+%.5f -> %.3f+%.5f (%.3f->%.3f) (|inc| = %f)! \t", lvl, 0, lambda, "INITIA",
                   sqrtf((float)(resOld[0] / resOld[2])), sqrtf((float)(resOld[1] / resOld[2])), sqrtf((float)(resOld[0] / resOld[2])),
                   sqrtf((float)(resOld[1] / resOld[2])), (resOld[0] + resOld[1]) / resOld[2], (resOld[0] + resOld[1]) / resOld[2], 0.0f);
            std::cout << refToNew_current.log().transpose() << " AFF " << refToNew_aff_current.vec().transpose() << "\n";
        }

        int iteration = 0;
        while (true) { //// levenberg marquardt
            Mat88f Hl = H;
            for (int i = 0; i < 8; i++)
                Hl(i, i) *= (1 + lambda);
            Hl -= Hsc * (1 / (1 + lambda));
            Vec8f bl = b - bsc * (1 / (1 + lambda));

            Hl = wM * Hl * wM * (0.01f / (w[lvl] * h[lvl]));
            bl = wM * bl * (0.01f / (w[lvl] * h[lvl]));

            Vec8f inc;       //// calculate increment, dim 1..6 = se3, dim 7..8 affine lighting
            if (fixAffine) { //// affine lighting not considered, only movement
                inc.head<6>() = -(wM.toDenseMatrix().topLeftCorner<6, 6>() * (Hl.topLeftCorner<6, 6>().ldlt().solve(bl.head<6>())));
                inc.tail<2>().setZero();
            } else
                inc = -(wM * (Hl.ldlt().solve(bl))); //=-H^-1 * b.

            SE3 refToNew_new = SE3::exp(inc.head<6>().cast<double>()) * refToNew_current; // T_ref2new_(k+1)=se3Exp(deltaXi)*T_ref2new_(k)
            AffLight refToNew_aff_new = refToNew_aff_current;
            refToNew_aff_new.a += inc[6];
            refToNew_aff_new.b += inc[7];
            doStep(lvl, lambda, inc);

            Mat88f H_new, Hsc_new;
            Vec8f b_new, bsc_new;
            Vec3f resNew = calcResAndGS(lvl, H_new, b_new, Hsc_new, bsc_new, refToNew_new, refToNew_aff_new, false);
            Vec3f regEnergy = calcEC(lvl);

            float eTotalNew = (resNew[0] + resNew[1] + regEnergy[1]);
            float eTotalOld = (resOld[0] + resOld[1] + regEnergy[0]);

            bool accept = eTotalOld > eTotalNew;

            if (printDebug) {
                printf("lvl %d, it %d (l=%f) %s: %.5f + %.5f + %.5f -> %.5f + %.5f + %.5f (%.2f->%.2f) (|inc| = %f)! \t", lvl, iteration,
                       lambda, (accept ? "ACCEPT" : "REJECT"), sqrtf((float)(resOld[0] / resOld[2])),
                       sqrtf((float)(regEnergy[0] / regEnergy[2])), sqrtf((float)(resOld[1] / resOld[2])),
                       sqrtf((float)(resNew[0] / resNew[2])), sqrtf((float)(regEnergy[1] / regEnergy[2])),
                       sqrtf((float)(resNew[1] / resNew[2])), eTotalOld / resNew[2], eTotalNew / resNew[2], inc.norm());
                std::cout << refToNew_new.log().transpose() << " AFF " << refToNew_aff_new.vec().transpose() << "\n";
            }

            if (accept) {

                if (resNew[1] == alphaK * numPoints[lvl])
                    snapped = true;
                H = H_new;
                b = b_new;
                Hsc = Hsc_new;
                bsc = bsc_new;
                resOld = resNew;
                refToNew_aff_current = refToNew_aff_new;
                refToNew_current = refToNew_new;
                applyStep(lvl);
                optReg(lvl);
                lambda *= 0.5;
                fails = 0;
                if (lambda < 0.0001)
                    lambda = 0.0001;
            } else {
                fails++;
                lambda *= 4;
                if (lambda > 10000)
                    lambda = 10000;
            }

            bool quitOpt = false;

            if (!(inc.norm() > eps) || iteration >= maxIterations[lvl] || fails >= 2) {
                Mat88f H, Hsc;
                Vec8f b, bsc;

                quitOpt = true;
            }

            if (quitOpt)
                break;
            iteration++;
        }
        latestRes = resOld;
    }

    thisToNext = refToNew_current; //// constant motion model
    thisToNext_aff = refToNew_aff_current;

    for (int i = 0; i < pyrLevelsUsed - 1; i++)
        propagateUp(i);

    frameID++;
    if (!snapped)
        snappedAt = 0;

    if (snapped && snappedAt == 0)
        snappedAt = frameID;

    debugPlot(0, wraps);

    return snapped && frameID > snappedAt + 5;
} // namespace dso

void CoarseInitializer::debugPlot(int lvl, std::vector<IOWrap::Output3DWrapper *> &wraps) {
    bool needCall = false;
    for (IOWrap::Output3DWrapper *ow : wraps)
        needCall = needCall || ow->needPushDepthImage();
    if (!needCall)
        return;

    int wl = w[lvl], hl = h[lvl];
    Eigen::Vector3f *colorRef = firstFrame->dIp[lvl];

    MinimalImageB3 iRImg(wl, hl);

    for (int i = 0; i < wl * hl; i++)
        iRImg.at(i) = Vec3b(colorRef[i][0], colorRef[i][0], colorRef[i][0]);

    int npts = numPoints[lvl];

    float nid = 0, sid = 0;
    for (int i = 0; i < npts; i++) {
        Pnt *point = points[lvl] + i;
        if (point->isGood) {
            nid++;
            sid += point->iR;
        }
    }
    float fac = nid / sid;

    for (int i = 0; i < npts; i++) {
        Pnt *point = points[lvl] + i;

        if (!point->isGood)
            iRImg.setPixel9(point->u + 0.5f, point->v + 0.5f, Vec3b(0, 0, 0));

        else
            iRImg.setPixel9(point->u + 0.5f, point->v + 0.5f, makeRainbow3B(point->iR * fac));
    }

    // IOWrap::displayImage("idepth-R", &iRImg, false);
    for (IOWrap::Output3DWrapper *ow : wraps)
        ow->pushDepthImage(&iRImg);
}

// calculates residual, Hessian and Hessian-block neede for re-substituting depth.
Vec3f CoarseInitializer::calcResAndGS(int lvl, Mat88f &H_out, Vec8f &b_out, Mat88f &H_out_sc, Vec8f &b_out_sc, const SE3 &refToNew,
                                      AffLight refToNew_aff, bool plot) {
    int wl = w[lvl], hl = h[lvl];
    Eigen::Vector3f *colorRef = firstFrame->dIp[lvl];
    Eigen::Vector3f *colorNew = newFrame->dIp[lvl];

    Mat33f RKi = (refToNew.rotationMatrix() * Ki[lvl]).cast<float>();
    Vec3f t = refToNew.translation().cast<float>();
    Eigen::Vector2f r2new_aff = Eigen::Vector2f(exp(refToNew_aff.a), refToNew_aff.b);

    float fxl = fx[lvl];
    float fyl = fy[lvl];
    float cxl = cx[lvl];
    float cyl = cy[lvl];

    Accumulator11 E;
    acc9.initialize(); //// sets to 0
    E.initialize();

    int npts = numPoints[lvl];
    Pnt *ptsl = points[lvl];
    for (int i = 0; i < npts; i++) {

        Pnt *point = ptsl + i;

        point->maxstep = 1e10;
        if (!point->isGood) { //// accumulate energy of bad points
            E.updateSingle((float)(point->energy[0]));
            point->energy_new = point->energy;
            point->isGood_new = false;
            continue;
        }

        //// 8by1 vectors for residuals and jacobians/partial derivatives
        VecNRf dp0;
        VecNRf dp1;
        VecNRf dp2;
        VecNRf dp3;
        VecNRf dp4;
        VecNRf dp5;
        VecNRf dp6;
        VecNRf dp7;
        VecNRf dd;
        VecNRf r;
        JbBuffer_new[i].setZero();

        // sum over residuals pattern
        bool isGood = true;
        float energy = 0;
        for (int idx = 0; idx < patternNum; idx++) {
            int dx = patternP[idx][0];
            int dy = patternP[idx][1];

            //// warping of patter image point, calc new pixel position given, current refToNew Transformation
            //// why is translation vector scaled with inverse depth and not RKi * (u,v,1).transpose() ?????
            Vec3f pt = RKi * Vec3f(point->u + dx, point->v + dy, 1) + t * point->idepth_new;
            float u = pt[0] / pt[2];
            float v = pt[1] / pt[2];
            float Ku = fxl * u + cxl;                     //// actual pixel position in new frame
            float Kv = fyl * v + cyl;                     //// actual pixel position in new frame
            float new_idepth = point->idepth_new / pt[2]; //// scale inverse depth with z value of warped point

            if (!(Ku > 1 && Kv > 1 && Ku < wl - 2 && Kv < hl - 2 && new_idepth > 0)) { //// warped pixel position out of bounds
                isGood = false;
                break;
            }

            Vec3f hitColor = getInterpolatedElement33(colorNew, Ku, Kv, wl); //// pixel intensity of warped point in new image
            float rlR = getInterpolatedElement31(colorRef, point->u + dx, point->v + dy, wl); //// pixel intensity of pattern of ref image

            if (!std::isfinite(rlR) || !std::isfinite((float)hitColor[0])) { //// interpolation gone wrong?
                isGood = false;
                break;
            }

            float residual = hitColor[0] - r2new_aff[0] * rlR - r2new_aff[1];                   //// I_new(p_warp) - exp(a)*I(p_ref) + b;
            float hw = fabs(residual) < setting_huberTH ? 1 : setting_huberTH / fabs(residual); //// huber weighting
            energy += hw * residual * residual * (2 - hw);                                      //// accumulate not the typical huber loss

            float dxdd = (t[0] - t[2] * u) / pt[2]; //// for depth residual?
            float dydd = (t[1] - t[2] * v) / pt[2]; //// for depth residual?

            //// analytic jacobian calculation
            if (hw < 1)
                hw = sqrtf(hw);
            float dxInterp = hw * hitColor[1] * fxl;
            float dyInterp = hw * hitColor[2] * fyl;
            dp0[idx] = new_idepth * dxInterp;
            dp1[idx] = new_idepth * dyInterp;
            dp2[idx] = -new_idepth * (u * dxInterp + v * dyInterp);
            dp3[idx] = -u * v * dxInterp - (1 + v * v) * dyInterp;
            dp4[idx] = (1 + u * u) * dxInterp + u * v * dyInterp;
            dp5[idx] = -v * dxInterp + u * dyInterp;
            dp6[idx] = -hw * r2new_aff[0] * rlR; //// illumination residual
            dp7[idx] = -hw * 1;                  //// illumination residual
            dd[idx] = dxInterp * dxdd + dyInterp * dydd;
            r[idx] = hw * residual;

            float maxstep = 1.0f / Vec2f(dxdd * fxl, dydd * fyl).norm();
            if (maxstep < point->maxstep)
                point->maxstep = maxstep;

            // immediately compute dp*dd' and dd*dd' in JbBuffer1.
            JbBuffer_new[i][0] += dp0[idx] * dd[idx];
            JbBuffer_new[i][1] += dp1[idx] * dd[idx];
            JbBuffer_new[i][2] += dp2[idx] * dd[idx];
            JbBuffer_new[i][3] += dp3[idx] * dd[idx];
            JbBuffer_new[i][4] += dp4[idx] * dd[idx];
            JbBuffer_new[i][5] += dp5[idx] * dd[idx];
            JbBuffer_new[i][6] += dp6[idx] * dd[idx];
            JbBuffer_new[i][7] += dp7[idx] * dd[idx];
            JbBuffer_new[i][8] += r[idx] * dd[idx];
            JbBuffer_new[i][9] += dd[idx] * dd[idx];
        }

        if (!isGood || energy > point->outlierTH * 20) {
            E.updateSingle((float)(point->energy[0]));
            point->isGood_new = false;
            point->energy_new = point->energy;
            continue;
        }

        // add into energy.
        E.updateSingle(energy);
        point->isGood_new = true;
        point->energy_new[0] = energy;

        // update Hessian matrix.
        for (int i = 0; i + 3 < patternNum; i += 4) //// called twice
            acc9.updateSSE(_mm_load_ps(((float *)(&dp0)) + i), _mm_load_ps(((float *)(&dp1)) + i), _mm_load_ps(((float *)(&dp2)) + i),
                           _mm_load_ps(((float *)(&dp3)) + i), _mm_load_ps(((float *)(&dp4)) + i), _mm_load_ps(((float *)(&dp5)) + i),
                           _mm_load_ps(((float *)(&dp6)) + i), _mm_load_ps(((float *)(&dp7)) + i), _mm_load_ps(((float *)(&r)) + i));

        //// i = 0 0 0 0 4 4 4 4 8 8 8 8 12 12 12 12 ... but because patternNum == 8 this does not get executed?!
        // printf("patternnum %d\n", patternNum);
        for (int i = ((patternNum >> 2) << 2); i < patternNum; i++)
            acc9.updateSingle((float)dp0[i], (float)dp1[i], (float)dp2[i], (float)dp3[i], (float)dp4[i], (float)dp5[i], (float)dp6[i],
                              (float)dp7[i], (float)r[i]);
    }

    E.finish();
    acc9.finish();

    // calculate alpha energy, and decide if we cap it.
    //// what is this alpha energy all about
    Accumulator11 EAlpha;
    EAlpha.initialize();
    for (int i = 0; i < npts; i++) {
        Pnt *point = ptsl + i;
        if (!point->isGood_new) {
            E.updateSingle((float)(point->energy[1])); //// energy 1 is accumulated
        } else {
            point->energy_new[1] = (point->idepth_new - 1) * (point->idepth_new - 1);
            E.updateSingle((float)(point->energy_new[1])); //// energy 1 is accumulated
        }
    }
    EAlpha.finish();
    float alphaEnergy = alphaW * (EAlpha.A + refToNew.translation().squaredNorm() * npts);
    // printf("AE = %f * %f + %f\n", alphaW, EAlpha.A, refToNew.translation().squaredNorm() * npts);

    // compute alpha opt.
    float alphaOpt;
    if (alphaEnergy > alphaK * npts) {
        alphaOpt = 0;
        alphaEnergy = alphaK * npts;
    } else {
        alphaOpt = alphaW; //// alphaW = 150*150
    }

    acc9SC.initialize();
    for (int i = 0; i < npts; i++) { //// big question mark
        Pnt *point = ptsl + i;
        if (!point->isGood_new)
            continue;

        point->lastHessian_new = JbBuffer_new[i][9];

        JbBuffer_new[i][8] += alphaOpt * (point->idepth_new - 1);
        JbBuffer_new[i][9] += alphaOpt;

        if (alphaOpt == 0) {
            JbBuffer_new[i][8] += couplingWeight * (point->idepth_new - point->iR);
            JbBuffer_new[i][9] += couplingWeight;
        }

        JbBuffer_new[i][9] = 1 / (1 + JbBuffer_new[i][9]);
        acc9SC.updateSingleWeighted((float)JbBuffer_new[i][0], (float)JbBuffer_new[i][1], (float)JbBuffer_new[i][2],
                                    (float)JbBuffer_new[i][3], (float)JbBuffer_new[i][4], (float)JbBuffer_new[i][5],
                                    (float)JbBuffer_new[i][6], (float)JbBuffer_new[i][7], (float)JbBuffer_new[i][8],
                                    (float)JbBuffer_new[i][9]);
    }
    acc9SC.finish();

    // printf("nelements in H: %d, in E: %d, in Hsc: %d / 9!\n", (int)acc9.num, (int)E.num, (int)acc9SC.num*9);
    H_out = acc9.H.topLeftCorner<8, 8>();       // / acc9.num;
    b_out = acc9.H.topRightCorner<8, 1>();      // / acc9.num;
    H_out_sc = acc9SC.H.topLeftCorner<8, 8>();  // / acc9.num;
    b_out_sc = acc9SC.H.topRightCorner<8, 1>(); // / acc9.num;

    H_out(0, 0) += alphaOpt * npts;
    H_out(1, 1) += alphaOpt * npts;
    H_out(2, 2) += alphaOpt * npts;

    Vec3f tlog = refToNew.log().head<3>().cast<float>();
    b_out[0] += tlog[0] * alphaOpt * npts;
    b_out[1] += tlog[1] * alphaOpt * npts;
    b_out[2] += tlog[2] * alphaOpt * npts;

    return Vec3f(E.A, alphaEnergy, E.num);
}

float CoarseInitializer::rescale() {
    float factor = 20 * thisToNext.translation().norm();
    //	float factori = 1.0f/factor;
    //	float factori2 = factori*factori;
    //
    //	for(int lvl=0;lvl<pyrLevelsUsed;lvl++)
    //	{
    //		int npts = numPoints[lvl];
    //		Pnt* ptsl = points[lvl];
    //		for(int i=0;i<npts;i++)
    //		{
    //			ptsl[i].iR *= factor;
    //			ptsl[i].idepth_new *= factor;
    //			ptsl[i].lastHessian *= factori2;
    //		}
    //	}
    //	thisToNext.translation() *= factori;

    return factor;
}

Vec3f CoarseInitializer::calcEC(int lvl) {
    if (!snapped)
        return Vec3f(0, 0, numPoints[lvl]);
    AccumulatorX<2> E;
    E.initialize();
    int npts = numPoints[lvl];
    for (int i = 0; i < npts; i++) {
        Pnt *point = points[lvl] + i;
        if (!point->isGood_new)
            continue;
        float rOld = (point->idepth - point->iR);
        float rNew = (point->idepth_new - point->iR);
        E.updateNoWeight(Vec2f(rOld * rOld, rNew * rNew));

        // printf("%f %f %f!\n", point->idepth, point->idepth_new, point->iR);
    }
    E.finish();

    // printf("ER: %f %f %f!\n", couplingWeight*E.A1m[0], couplingWeight*E.A1m[1], (float)E.num.numIn1m);
    return Vec3f(couplingWeight * E.A1m[0], couplingWeight * E.A1m[1], E.num);
}

//// regularizes iR value depending on neighbors' iR values and regWeight constant
void CoarseInitializer::optReg(int lvl) { //// is called in propagateDown with srcLvl-1, so from finest to second last coarsest lvl
    int npts = numPoints[lvl];
    Pnt *ptsl = points[lvl];
    if (!snapped) { //// if points not snapped set iR to 1
        for (int i = 0; i < npts; i++)
            ptsl[i].iR = 1;
        return;
    }

    for (int i = 0; i < npts; i++) {
        Pnt *point = ptsl + i;
        if (!point->isGood)
            continue;

        float idnn[10];
        int nnn = 0;
        for (int j = 0; j < 10; j++) {
            if (point->neighbours[j] == -1)
                continue;
            Pnt *other = ptsl + point->neighbours[j];
            if (!other->isGood)
                continue;
            idnn[nnn] = other->iR;
            nnn++;
        }

        if (nnn > 2) {
            std::nth_element(idnn, idnn + nnn / 2, idnn + nnn);
            point->iR = (1 - regWeight) * point->idepth + regWeight * idnn[nnn / 2];
        }
    }
}

//// propagate inverse depth from finer pyramid lvls to coarser pyramid lvls (parent)
void CoarseInitializer::propagateUp(int srcLvl) {
    assert(srcLvl + 1 < pyrLevelsUsed);
    // set idepth of target

    int nptss = numPoints[srcLvl];
    int nptst = numPoints[srcLvl + 1];
    Pnt *ptss = points[srcLvl];
    Pnt *ptst = points[srcLvl + 1];

    // set to zero.
    for (int i = 0; i < nptst; i++) {
        Pnt *parent = ptst + i;
        parent->iR = 0;
        parent->iRSumNum = 0;
    }

    for (int i = 0; i < nptss; i++) {
        Pnt *point = ptss + i;
        if (!point->isGood)
            continue;

        Pnt *parent = ptst + point->parent;
        parent->iR += point->iR * point->lastHessian;
        parent->iRSumNum += point->lastHessian;
    }

    for (int i = 0; i < nptst; i++) {
        Pnt *parent = ptst + i;
        if (parent->iRSumNum > 0) {
            parent->idepth = parent->iR = (parent->iR / parent->iRSumNum);
            parent->isGood = true;
        }
    }

    optReg(srcLvl + 1);
}

//// propagate inverse depth from coarser pyramid lvls (parent) to finer pyramid lvls
void CoarseInitializer::propagateDown(int srcLvl) {
    assert(srcLvl > 0); //// cant be done for finest pyramid lvl
    // set idepth of target

    int nptst = numPoints[srcLvl - 1]; //// num points in finer pyramid lvl
    Pnt *ptss = points[srcLvl];        //// points in coarser pyramid lvl
    Pnt *ptst = points[srcLvl - 1];    //// points in finer pyramid lvl

    for (int i = 0; i < nptst; i++) {
        Pnt *point = ptst + i; //// point in finer pyramid lvl
        Pnt *parent = ptss + point->parent;

        if (!parent->isGood || parent->lastHessian < 0.1) //// skip parents with bad lastHessian value
            continue;
        if (!point->isGood) { //// point in finer pyramid lvl can be set to good if parent exists
            point->iR = point->idepth = point->idepth_new = parent->iR;
            point->isGood = true;
            point->lastHessian = 0;
        } else {
            float newiR =
                (point->iR * point->lastHessian * 2 + parent->iR * parent->lastHessian) / (point->lastHessian * 2 + parent->lastHessian);
            point->iR = point->idepth = point->idepth_new = newiR;
        }
    }
    //// regularize the newly updated/set iR value of finer pyramid lvl
    optReg(srcLvl - 1);
}

void CoarseInitializer::makeGradients(Eigen::Vector3f **data) {
    for (int lvl = 1; lvl < pyrLevelsUsed; lvl++) {
        int lvlm1 = lvl - 1;
        int wl = w[lvl], hl = h[lvl], wlm1 = w[lvlm1];

        Eigen::Vector3f *dINew_l = data[lvl];
        Eigen::Vector3f *dINew_lm = data[lvlm1];

        for (int y = 0; y < hl; y++)
            for (int x = 0; x < wl; x++)
                dINew_l[x + y * wl][0] = 0.25f * (dINew_lm[2 * x + 2 * y * wlm1][0] + dINew_lm[2 * x + 1 + 2 * y * wlm1][0] +
                                                  dINew_lm[2 * x + 2 * y * wlm1 + wlm1][0] + dINew_lm[2 * x + 1 + 2 * y * wlm1 + wlm1][0]);

        for (int idx = wl; idx < wl * (hl - 1); idx++) {
            dINew_l[idx][1] = 0.5f * (dINew_l[idx + 1][0] - dINew_l[idx - 1][0]);
            dINew_l[idx][2] = 0.5f * (dINew_l[idx + wl][0] - dINew_l[idx - wl][0]);
        }
    }
}

//// initially sets camera intrinsics for all pyramid levels, heuristically selects pixels and sets stati in pyramid levels, initializes
//// idepthmap to 1, sets energy outlier threshold according to pattern, and finds nn nearest neighbors for points in pyramid lvl 0-4, finds
//// single closest parent for all points in coarser pyramid lvl
void CoarseInitializer::setFirst(CalibHessian *HCalib, FrameHessian *newFrameHessian) {

    makeK(HCalib); //// calc downscaled intrinsics for all pyramid levels
    firstFrame = newFrameHessian;

    PixelSelector sel(w[0], h[0]);

    float *statusMap = new float[w[0] * h[0]];
    bool *statusMapB = new bool[w[0] * h[0]];

    float densities[] = {0.03, 0.05, 0.15, 0.5, 1}; //// determine percentual amount of pixels selected in pyramic levels
    for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) {
        sel.currentPotential = 3;
        int npts;
        if (lvl == 0)
            npts = sel.makeMaps(firstFrame, statusMap, densities[lvl] * w[0] * h[0], 1, false, 2);
        else
            npts = makePixelStatus(firstFrame->dIp[lvl], statusMapB, w[lvl], h[lvl], densities[lvl] * w[0] * h[0]);

        if (points[lvl] != 0)
            delete[] points[lvl];
        points[lvl] = new Pnt[npts];

        // set idepth map to initially 1 everywhere.
        int wl = w[lvl], hl = h[lvl];
        Pnt *pl = points[lvl];
        int nl = 0;
        for (int y = patternPadding + 1; y < hl - patternPadding - 2; y++) //// loop across image, leaving out 1 borderpixel and patternpad
            for (int x = patternPadding + 1; x < wl - patternPadding - 2; x++) {
                // if(x==2) printf("y=%d!\n",y);
                if ((lvl != 0 && statusMapB[x + y * wl]) || (lvl == 0 && statusMap[x + y * wl] != 0)) { ////
                    // assert(patternNum==9);
                    pl[nl].u = x + 0.1; //// understand this index? why +0.1
                    pl[nl].v = y + 0.1; //// understand this index? why +0.1
                    pl[nl].idepth = 1;
                    pl[nl].iR = 1;
                    pl[nl].isGood = true;
                    pl[nl].energy.setZero();
                    pl[nl].lastHessian = 0;
                    pl[nl].lastHessian_new = 0;
                    pl[nl].my_type = (lvl != 0) ? 1 : statusMap[x + y * wl];

                    //// this is not needed since none of the variables are used
                    Eigen::Vector3f *cpt = firstFrame->dIp[lvl] + x + y * w[lvl];
                    float sumGrad2 = 0;
                    for (int idx = 0; idx < patternNum; idx++) { //// does this have any effect on anything?
                        int dx = patternP[idx][0];
                        int dy = patternP[idx][1];
                        float absgrad = cpt[dx + dy * w[lvl]].tail<2>().squaredNorm();
                        sumGrad2 += absgrad;
                    }

                    //// sumgrad2 is just used here in this commented section, seems like an alternative
                    //	float gth = setting_outlierTH * (sqrtf(sumGrad2)+setting_outlierTHSumComponent);
                    //	pl[nl].outlierTH = patternNum*gth*gth;

                    pl[nl].outlierTH = patternNum * setting_outlierTH; //// 8*outlierTH

                    nl++;
                    assert(nl <= npts);
                }
            }

        numPoints[lvl] = nl;
    }
    delete[] statusMap;
    delete[] statusMapB;

    makeNN();

    thisToNext = SE3();
    snapped = false;
    frameID = snappedAt = 0;

    for (int i = 0; i < pyrLevelsUsed; i++)
        dGrads[i].setZero();
}

//// sets Energy to 0, sets idept_new = idepth, if point is not good iR is set to mean of neighbors iR
void CoarseInitializer::resetPoints(int lvl) {
    Pnt *pts = points[lvl];
    int npts = numPoints[lvl];
    for (int i = 0; i < npts; i++) {
        pts[i].energy.setZero();
        pts[i].idepth_new = pts[i].idepth;

        if (lvl == pyrLevelsUsed - 1 && !pts[i].isGood) {
            float snd = 0, sn = 0;
            for (int n = 0; n < 10; n++) {
                if (pts[i].neighbours[n] == -1 || !pts[pts[i].neighbours[n]].isGood)
                    continue;
                snd += pts[pts[i].neighbours[n]].iR;
                sn += 1;
            }

            if (sn > 0) {
                pts[i].isGood = true;
                pts[i].iR = pts[i].idepth = pts[i].idepth_new = snd / sn;
            }
        }
    }
}

//// assigns idepth value by calculating some step, step calculation i dont understand
void CoarseInitializer::doStep(int lvl, float lambda, Vec8f inc) {

    const float maxPixelStep = 0.25;
    const float idMaxStep = 1e10;
    Pnt *pts = points[lvl];
    int npts = numPoints[lvl];
    for (int i = 0; i < npts; i++) {
        if (!pts[i].isGood)
            continue;

        float b = JbBuffer[i][8] + JbBuffer[i].head<8>().dot(inc);
        float step = -b * JbBuffer[i][9] / (1 + lambda);

        float maxstep = maxPixelStep * pts[i].maxstep;
        if (maxstep > idMaxStep)
            maxstep = idMaxStep;

        if (step > maxstep)
            step = maxstep;
        if (step < -maxstep)
            step = -maxstep;

        float newIdepth = pts[i].idepth + step;
        if (newIdepth < 1e-3) //// if depth > 1000
            newIdepth = 1e-3;
        if (newIdepth > 50) //// if depth < 0.02
            newIdepth = 50;
        pts[i].idepth_new = newIdepth;
    }
}

//// assigns bad points idepth iR value, assigns good points energy, isGood, idepth and lastHessian to new values and swaps Jacobian Buffers
void CoarseInitializer::applyStep(int lvl) {
    Pnt *pts = points[lvl];
    int npts = numPoints[lvl];
    for (int i = 0; i < npts; i++) {
        if (!pts[i].isGood) {
            pts[i].idepth = pts[i].idepth_new = pts[i].iR;
            continue;
        }
        pts[i].energy = pts[i].energy_new;
        pts[i].isGood = pts[i].isGood_new;
        pts[i].idepth = pts[i].idepth_new;
        pts[i].lastHessian = pts[i].lastHessian_new;
    }
    std::swap<Vec10f *>(JbBuffer, JbBuffer_new);
}

void CoarseInitializer::makeK(CalibHessian *HCalib) {
    w[0] = wG[0];
    h[0] = hG[0];

    fx[0] = HCalib->fxl();
    fy[0] = HCalib->fyl();
    cx[0] = HCalib->cxl();
    cy[0] = HCalib->cyl();

    for (int level = 1; level < pyrLevelsUsed; ++level) {
        w[level] = w[0] >> level;
        h[level] = h[0] >> level;
        fx[level] = fx[level - 1] * 0.5;
        fy[level] = fy[level - 1] * 0.5;
        cx[level] = (cx[0] + 0.5) / ((int)1 << level) - 0.5; //// value of a << b == a*2^b
        cy[level] = (cy[0] + 0.5) / ((int)1 << level) - 0.5; //// value of a << b == a*2^b
    }

    for (int level = 0; level < pyrLevelsUsed; ++level) {
        K[level] << fx[level], 0.0, cx[level], 0.0, fy[level], cy[level], 0.0, 0.0, 1.0;
        Ki[level] = K[level].inverse();
        fxi[level] = Ki[level](0, 0);
        fyi[level] = Ki[level](1, 1);
        cxi[level] = Ki[level](0, 2);
        cyi[level] = Ki[level](1, 2);
    }
}

//// finds nn nearest neighbors for points in all pyramid lvls in same pyramid lvl, and finds one closest parent in coarser pyramid lvl
void CoarseInitializer::makeNN() {
    const float NNDistFactor = 0.05;

    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, FLANNPointcloud>, FLANNPointcloud, 2> KDTree;

    // build indices
    FLANNPointcloud pcs[PYR_LEVELS];
    KDTree *indexes[PYR_LEVELS];
    for (int i = 0; i < pyrLevelsUsed; i++) {
        pcs[i] = FLANNPointcloud(numPoints[i], points[i]);                                //// make point clouds for each pyramid lvl
        indexes[i] = new KDTree(2, pcs[i], nanoflann::KDTreeSingleIndexAdaptorParams(5)); //// research on KDTree
        indexes[i]->buildIndex();
    }

    const int nn = 10; //// find 10 nearest neighbors i guess

    // find NN & parents
    for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) {
        Pnt *pts = points[lvl]; //// use pts as shortcut in this fn, thats all
        int npts = numPoints[lvl];

        int ret_index[nn];
        float ret_dist[nn];
        nanoflann::KNNResultSet<float, int, int> resultSet(nn); //// 10 nearest neighbors
        nanoflann::KNNResultSet<float, int, int> resultSet1(1); //// nearest neighbor

        for (int i = 0; i < npts; i++) {
            // resultSet.init(pts[i].neighbours, pts[i].neighboursDist );
            resultSet.init(ret_index, ret_dist);
            Vec2f pt = Vec2f(pts[i].u, pts[i].v);
            indexes[lvl]->findNeighbors(resultSet, (float *)&pt, nanoflann::SearchParams()); //// actual neighbor search
            int myidx = 0;
            float sumDF = 0;
            for (int k = 0; k < nn; k++) {
                pts[i].neighbours[myidx] = ret_index[k];
                float df = expf(-ret_dist[k] * NNDistFactor); //// distance = e^(-dist*constant)
                sumDF += df;
                pts[i].neighboursDist[myidx] = df;
                assert(ret_index[k] >= 0 && ret_index[k] < npts);
                myidx++;
            }
            for (int k = 0; k < nn; k++)
                pts[i].neighboursDist[k] *= 10 / sumDF;

            if (lvl < pyrLevelsUsed - 1) { //// for levels set parents in coarser pyramid lvls
                resultSet1.init(ret_index, ret_dist);
                pt = pt * 0.5f - Vec2f(0.25f, 0.25f); //// point in lower pyramid lvl
                indexes[lvl + 1]->findNeighbors(resultSet1, (float *)&pt, nanoflann::SearchParams());

                pts[i].parent = ret_index[0];
                pts[i].parentDist = expf(-ret_dist[0] * NNDistFactor);

                assert(ret_index[0] >= 0 && ret_index[0] < numPoints[lvl + 1]);
            } else {
                pts[i].parent = -1; //// coarsest pyramid level has no parents
                pts[i].parentDist = -1;
            }
        }
    }

    // done.

    for (int i = 0; i < pyrLevelsUsed; i++)
        delete indexes[i];
}
} // namespace dso
