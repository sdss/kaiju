#include <iostream>
#include <time.h>       /* time */
#include <cmath>
#include <thread>
#include "utils.h"
#include "robot.h"
#include "robotGrid.h"

// define position of fibers in beta arm's reference frame
// 0,0,0 is beta axis,
// +x extends toward beta arm head,
// +y points towards increasing beta arm angle
// +z (defined by right hand rule) points towards M2 mirror

// xyz pos of fiber in beta neutral position
// taken from peter's solid model (not MPS)
// const double metFiberData[] = {14.3458, 0, 0}; // from uw shop
// Eigen::Vector3d metFiberNeutral(metFiberData);

// const double apFiberData[] = {14.9953, 0.375, 0};
// Eigen::Vector3d apFiberNeutral(apFiberData);

// const double bossFiberData[] = {14.9953, -0.375, 0};
// Eigen::Vector3d bossFiberNeutral(bossFiberData);

const double alphaLen = 7.4;
const double betaLen = 15; // mm to fiber

// conor's optimal data to obtain full extension for top fiber (ap)
// .75 is fiber to fiber spacing in snowflake
// .375 is half that
const double met2sciX = sqrt(0.75*0.75 - 0.375*0.375);
const double beta2sciX = sqrt(22.4*22.4-0.375*0.375) - alphaLen;
const double beta2metX = beta2sciX - met2sciX;


// min reach is distance from alpha axis to science fiber when beta = 180
// const double minReach = hypot(beta2sciX, 0.375) - alphaLen;
// max reach is the distance from alpha axis to science fiber when beta = 0
// const double maxReach = betaLen + alphaLen;

// these max/min Reaches are the slightly more restrictive ones
// based on the small angular offset of the science fibers
// with respect to the beta axis...
// const double maxReach = hypot(alphaLen+beta2sciX, 0.375);
// const double minReach = hypot(alphaLen-beta2sciX, 0.375);

const double maxReach = alphaLen + betaLen;
const double minReach = betaLen - alphaLen;

const double metFiberData[] = {beta2metX, 0, 0}; // from uw shop
Eigen::Vector3d metFiberNeutral(metFiberData);

const double apFiberData[] = {beta2sciX, 0.375, 0};
Eigen::Vector3d apFiberNeutral(apFiberData);

const double bossFiberData[] = {beta2sciX, -0.375, 0};
Eigen::Vector3d bossFiberNeutral(bossFiberData);

// create a vector that translates from beta orgin to alpha origin
// this is just the length of the alpha arm
const double alphaTransData[] = {alphaLen, 0, 0};
Eigen::Vector3d alphaTransV(alphaTransData);


const std::array<Eigen::Vector3d, 3> neutralFiberList{ {metFiberNeutral, apFiberNeutral, bossFiberNeutral} };

const int BOSS_FIBER_ID = 2;
const int AP_FIBER_ID = 1;
const int MET_FIBER_ID = 0;

// beta arm collision segments
// beta geometries in beta arm reference frame
const double focalZ = 30; // height to focal plane (where fiber lives)
const double betaAxis2End = 19.2 - 3.0; //mm
const double betaEndRadius = 1.2; // mm
const double betaEnvPt1Data[] = {0, 0, focalZ}; // beta axis
Eigen::Vector3d betaEnvPt1(betaEnvPt1Data);
const double betaEnvPt2Data[] = {betaAxis2End - betaEndRadius, 0, focalZ};
Eigen::Vector3d betaEnvPt2(betaEnvPt2Data);
const std::array<Eigen::Vector3d, 2> neutralBetaCollisionSegment{ {betaEnvPt1, betaEnvPt2} };
// radius containing beta arm for collision detection
const double betaCollisionRadius = 1.5; // mm (3mm wide)
const double fiducialBuffer = 1.5; // 3mm wide fiducial

// xyz pos of fiber in beta neutra position
// const double fiberNeutral_data[] = {betaLen, 0, 0};
// Eigen::Vector3d fiberNeutral(fiberNeutral_data);

Robot::Robot(int myid, double myxPos, double myyPos, double myAngStep, bool myHasApogee) {
    // std::cout << "robot constructor called" << std::endl;
    xPos = myxPos;
    yPos = myyPos;
    angStep = myAngStep;
    transXY = Eigen::Vector3d(myxPos, myyPos, 0);
    id = myid;
    hasApogee = myHasApogee;
    hasBoss = true; // break this out into a config/constructor
    betaCollisionSegment = neutralBetaCollisionSegment;
    // std::pair<betaGeometry, std::vector<double>> betaPair = getBetaGeom(8);
    // betaModel = betaPair.first;
    // betaOrientation = betaPair.first;
    // modelRadii = betaPair.second;

    alphaBetaArr <<  -angStep,  angStep,
                             0,  angStep,
                      angStep,  angStep,
                     -angStep,         0,
                      angStep,         0,
                     -angStep, -angStep,
                             0, -angStep,
                      angStep, -angStep;
}

void Robot::setCollisionBuffer(double newBuffer){
    collisionBuffer = newBuffer;
}


// bool Robot::checkFiberXYGlobal(double xFiberGlobal, double yFiberGlobal, int fiberID){
//     double xFiberLocal = xFiberGlobal - xPos;
//     double yFiberLocal = yFiberGlobal - yPos;
//     bool canReach = checkFiberXYLocal(xFiberLocal, yFiberLocal, fiberID);
//     return canReach;
// }

// bool Robot::checkFiberXYLocal(double xFiberLocal, double yFiberLocal, int fiberID){
//     double rad = hypot(xFiberLocal, yFiberLocal);
//     bool canReach = false;
//     double beta2Fiber = hypot(neutralFiberList[fiberID](0), neutralFiberList[fiberID](1));
//     double maxReach = alphaLen + beta2Fiber;
//     double minReach = betaLen - alphaLen;
//     if (rad > minReach and rad < maxReach) {
//         canReach = true;
//     }
//     return canReach;

// }

void Robot::setFiberXY(double xFiberGlobal, double yFiberGlobal, int fiberID){
    // Target testTarget(0, xFiberGlobal, yFiberGlobal, 1, fiberID);
    auto targPtr = std::make_shared<Target>(0, xFiberGlobal, yFiberGlobal, 1, fiberID );
    bool canReach = isValidTarget(targPtr);
    if (!canReach){
        throw std::runtime_error("cannot reach target xy");
    }
    auto newAlphaBeta = alphaBetaFromFiberXY(xFiberGlobal, yFiberGlobal, fiberID);
    setAlphaBeta(newAlphaBeta[0], newAlphaBeta[1]);
    // should I explicitly delete the targPtr? or does this happen automatically

}


void Robot::addNeighbor(int robotInd){
    neighbors.push_back(robotInd);
}

void Robot::addFiducial(std::array<double, 2> fiducial){

    fiducials.push_back(Eigen::Vector3d(fiducial[0], fiducial[1], focalZ));
}

void Robot::setAlphaBeta(double newAlpha, double newBeta){
    // targetAssigned = true;
    alpha = newAlpha;
    beta = newBeta;
    double alphaRad = alpha * M_PI / 180.0;
    double betaRad = beta * M_PI / 180.0;

    // Eigen::Transform<float, 3, Eigen::Affine> t = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    // t.rotate(Eigen::AngleAxisf(betaRad, Eigen::Vector3d::UnitZ()));


    // we begin from beta arm reference frame (thats what betaNeutral is in)
    // and we rotate beta about the origin 1st
    betaRot = Eigen::AngleAxisd(betaRad, Eigen::Vector3d::UnitZ());
    // next create rotation about alpha
    alphaRot = Eigen::AngleAxisd(alphaRad, Eigen::Vector3d::UnitZ());
    // full transform is
    // 1. rotate about beta
    // 2. translate by alpha arm length
    // 3. rotate about alpha
    // 4. translate by xy Pos of positioner
    // result is orientation of positioner in 3d wrt origin of
    // robot grid
    // fullTransform = transXY*alphaRot*alphaTransT*betaRot;
    // fullTransform = (alphaTransT*betaRot).matrix();

    // std::cout << "alpha beta crap" << std::endl;
    // std::cout << "beta model size: " << betaModel.size() << std::endl;
    // std::cout << "beta orientation size: " << betaOrientation.size() << std::endl;
    // std::cout << "beta before: " << betaOrientation[1][0] << " " << betaOrientation[1][1] << " " << betaOrientation[1][2] << std::endl;
    for (int ii = 0; ii < betaCollisionSegment.size(); ii++){
        // first rotate about beta (we start in beta ref frame)
        // next translate in x by the alpha arm length
        // we should be able to figure out how to compute
        // this matrix outside this loop....
        betaCollisionSegment[ii] = transXY + (alphaRot * (
            alphaTransV + (betaRot * neutralBetaCollisionSegment[ii])
            ));
    }
    // std::cout << "beta after: " << betaOrientation[1][0] << " " << betaOrientation[1][1] << " " << betaOrientation[1][2] << std::endl;
    // fiber_XYZ = transXY + (alphaRot * (alphaTransV + (betaRot * fiberNeutral)));
    metFiberPos = transXY + (alphaRot * (alphaTransV + (betaRot * metFiberNeutral)));
    apFiberPos = transXY + (alphaRot * (alphaTransV + (betaRot * apFiberNeutral)));
    bossFiberPos = transXY + (alphaRot * (alphaTransV + (betaRot * bossFiberNeutral)));

}

void Robot::setAlphaBetaRand(){
    double a = randomSample() * 359.99999;
    double b = randomSample() * 180.0;
    setAlphaBeta(a, b);
    // std::cout << "robot " << id << " set rand " << betaOrientation.size() << std::endl;
}

std::array<double, 2> Robot::randomXYUniform(){
	  std::array<double, 2> xy = sampleAnnulus(minReach, maxReach);
    return xy;
}

void Robot::setXYUniform(){
    // perhaps get rid of this and just use setAlphaBetaRand()?
    auto xy = sampleAnnulus(minReach, maxReach);
    // use a science fiber ID (matches min/max reach)
    // std::cout.precision(20);
    auto ab = alphaBetaFromFiberXY(xy[0]+xPos, xy[1]+yPos, 1);
    // if (ab[1] > 180.0){
    //     std::cout << "x " << xy[0] << " y " << xy[1] << " a " << ab[0]
    //     << " b " << ab[1] << std::endl;
    // }
    setAlphaBeta(ab[0], ab[1]);
}

bool Robot::isCollided(){
    // so scope really fucked me on this one?
    // lots of returns fixed it.
    double dist2, collideDist2;
    // check collisions with neighboring robots
    for (auto robot : neighbors){

        // squared distance
        dist2 = dist3D_Segment_to_Segment(
                betaCollisionSegment[0], betaCollisionSegment[1],
                robot->betaCollisionSegment[0], robot->betaCollisionSegment[1]
            );

        collideDist2 = (2*betaCollisionRadius+collisionBuffer)*
                        (2*betaCollisionRadius+robot->collisionBuffer);
        if (dist2 < collideDist2){
            // std::cout << "dist " << dist2 - collide_dist_squared << std::endl;
            return true;
        }

    }
    // std::cout << "testing collision" << std::endl;

    return isFiducialCollided();
}

bool Robot::isFiducialCollided(){
    // std::cout << "isFiducialCollided" << std::endl;
    double dist2, collideDist2;
    // std::cout << "n fiducials " << fiducials.size() << std::endl;
    for (auto fiducial : fiducials){
        // squared distance
        dist2 = dist3D_Point_to_Segment(
                fiducial, betaCollisionSegment[0], betaCollisionSegment[1]
                );
        collideDist2 = (betaCollisionRadius+collisionBuffer+fiducialBuffer)*
                        (betaCollisionRadius+collisionBuffer+fiducialBuffer);

        if (dist2 < collideDist2){
            // std::cout << "we're collided! " << sqrt(dist2) << std::endl;
            return true;
        }
    }
    return false;
}


void Robot::decollide(){
    // remove assigned target if present
    assignedTarget.reset();
    for (int ii=0; ii<1000; ii++){
        setXYUniform();
        nDecollide ++;
        if (!isCollided()){
            break;
        }
    }
    // are we still collided?
    if (isCollided()){
        throw std::runtime_error("Unable do decollide robot!!!");
    }
}


void Robot::smoothPath(double epsilon){
    // smooth a previously generated path
    double interpSmoothAlpha, interpSmoothBeta;
    int npts;
    Eigen::Vector2d atemp, btemp;
    RamerDouglasPeucker(alphaPath, epsilon, smoothAlphaPath);
    // bias alpha positive direction because we are approaching zero
    npts = smoothAlphaPath.size();
    for (int ii=1; ii<npts-1; ii++){
        // only shift internal (not end) points
        smoothAlphaPath[ii](1) = smoothAlphaPath[ii](1);// + epsilon;
    }

    RamerDouglasPeucker(betaPath, epsilon, smoothBetaPath);
    // bias beta negative direction because we are approaching 180
    // linearly interpolate smooth paths to same step values
    // as computed
    // bias alpha positive direction because we are approaching zero
    npts = smoothBetaPath.size();
    for (int ii=1; ii<npts-1; ii++){
        // only shift internal (not end) points
        smoothBetaPath[ii](1) = smoothBetaPath[ii](1);// - epsilon;
    }

    // calculate smoothed alpha betas at every step
    int nDensePoints = alphaPath.size();
    for (int ii=0; ii<nDensePoints; ii++){
        double xVal = alphaPath[ii](0);
        atemp(0) = xVal; // interpolation step
        btemp(0) = xVal;
        interpSmoothAlpha = linearInterpolate(smoothAlphaPath, xVal);
        // bias alpha in positive direction because we're approaching zero
        atemp(1) = interpSmoothAlpha;
        interpSmoothAlphaPath.push_back(atemp);
        interpSmoothBeta = linearInterpolate(smoothBetaPath, xVal);
        btemp(1) = interpSmoothBeta;
        interpSmoothBetaPath.push_back(btemp);

        // populate interpXY points for alpha/beta ends
        setAlphaBeta(interpSmoothAlpha, interpSmoothBeta);
        atemp(1) = betaCollisionSegment[0](0); // xAlphaEnd
        interpAlphaX.push_back(atemp);
        atemp(1) = betaCollisionSegment[0](1); // yAlphaEnd
        interpAlphaY.push_back(atemp);
        atemp(1) = betaCollisionSegment.back()(0); // xBetaEnd
        interpBetaX.push_back(atemp);
        atemp(1) = betaCollisionSegment.back()(1); // yBetaEnd
        interpBetaY.push_back(atemp);

        atemp(1) = 0; // not collided
        if (isCollided()){
            atemp(1) = 1;
        }
        interpCollisions.push_back(atemp);

    }

}

void Robot::stepTowardFold(int stepNum){
    double currAlpha = alpha;
    double currBeta = beta;
    Eigen::Vector2d alphaPathPoint;
    Eigen::Vector2d betaPathPoint;
    Eigen::Vector2d temp;
    alphaPathPoint(0) = stepNum;
    betaPathPoint(0) = stepNum;
    if (currBeta==180 and currAlpha==0){
        // done folding don't move
        alphaPathPoint(1) = currAlpha;
        betaPathPoint(1) = currBeta;
        alphaPath.push_back(alphaPathPoint);
        betaPath.push_back(betaPathPoint);

        temp(0) = stepNum;
        temp(1) = betaCollisionSegment[0](0); // xAlphaEnd
        roughAlphaX.push_back(temp);
        temp(1) = betaCollisionSegment[0](1); // yAlphaEnd
        roughAlphaY.push_back(temp);
        temp(1) = betaCollisionSegment.back()(0); // xBetaEnd
        roughBetaX.push_back(temp);
        temp(1) = betaCollisionSegment.back()(1); // yBetaEnd
        roughBetaY.push_back(temp);

        return;
    }
    // this is for keeping track of last step
    // only updates if robot hasn't reached fold
    lastStepNum = stepNum;
    // begin trying options pick first that works
    for (int ii=0; ii<alphaBetaArr.rows(); ii++){
        double nextAlpha = currAlpha + alphaBetaArr(ii, 0);
        double nextBeta = currBeta + alphaBetaArr(ii, 1);
        if (nextAlpha > 360){
            nextAlpha = 360;
        }
        if (nextAlpha < 0){
            nextAlpha = 0;
        }
        if (nextBeta > 180){
            nextBeta = 180;
        }
        if (nextBeta < 0){
            nextBeta = 0;
        }
        // if next choice results in no move skip it
        // always favor a move
        if (nextBeta==currBeta and nextAlpha==currAlpha){
            continue;
        }
        setAlphaBeta(nextAlpha, nextBeta);
        if (!isCollided()){
            alphaPathPoint(1) = nextAlpha;
            betaPathPoint(1) = nextBeta;
            alphaPath.push_back(alphaPathPoint);
            betaPath.push_back(betaPathPoint);

            // add alpha/beta xy points

            temp(0) = stepNum;
            // std::cout << "beta orientation size: " << betaOrientation.size() << std::endl;
            // std::cout << "beta model size: " << betaModel.size() << std::endl;
            temp(1) = betaCollisionSegment[0](0); // xAlphaEnd
            // std::cout << "step toward fold " << ii << std::endl;

            roughAlphaX.push_back(temp);

            temp(1) = betaCollisionSegment[0](1); // yAlphaEnd
            roughAlphaY.push_back(temp);
            temp(1) = betaCollisionSegment.back()(0); // xBetaEnd
            roughBetaX.push_back(temp);
            temp(1) = betaCollisionSegment.back()(1); // yBetaEnd
            roughBetaY.push_back(temp);

            return;
        }
    }

    // no move options worked,
    // settle for a non-move
    setAlphaBeta(currAlpha, currBeta);
    alphaPathPoint(1) = currAlpha;
    betaPathPoint(1) = currBeta;
    alphaPath.push_back(alphaPathPoint);
    betaPath.push_back(betaPathPoint);

    // add alpha/beta xy points
    // Eigen::Vector2d temp;
    temp(0) = stepNum;
    temp(1) = betaCollisionSegment[0](0); // xAlphaEnd
    roughAlphaX.push_back(temp);
    temp(1) = betaCollisionSegment[0](1); // yAlphaEnd
    roughAlphaY.push_back(temp);
    temp(1) = betaCollisionSegment.back()(0); // xBetaEnd
    roughBetaX.push_back(temp);
    temp(1) = betaCollisionSegment.back()(1); // yBetaEnd
    roughBetaY.push_back(temp);
}


std::array<double, 2> Robot::alphaBetaFromFiberXY(double xFiberGlobal, double yFiberGlobal, int fiberID){
    // origin is at alpha axis
    // +x is aligned with alpha angle = 0
    // fiberID = 0 metrology
    // fiberID = 1 apogee
    // fiberID = 2 boss
    // law of cosines at work here...
    double x = xFiberGlobal - xPos;
    double y = yFiberGlobal - yPos;
    double xyMag = hypot(x, y);
    // beta2Fiber is length from beta axis to fiber
    // default to metrology to avoid numerical issues
    // with trig and 0's etc
    double beta2Fiber = neutralFiberList[fiberID](0);
    double fao = 0;  // fiber angle offset
    if (fiberID > 0){
        // note beta2Fiber should be 15? should it just be hard coded?
        // these are science fibers...
        beta2Fiber = hypot(neutralFiberList[fiberID](0), neutralFiberList[fiberID](1));
        // note atan2 is signed
        fao = atan2(neutralFiberList[fiberID](1), neutralFiberList[fiberID](0));
    }
    // gamma angle is angle between alpha arm at beta axis and
    // vector from beta axis to (x,y)
    double gammaAngRad = acos(
        (xyMag*xyMag - alphaLen*alphaLen - beta2Fiber*beta2Fiber)/
        (-2*alphaLen*beta2Fiber)
    );

    // solve for beta angle in radians
    double betaAngRad = M_PI - gammaAngRad - fao;

    // rot angle is angle between (x,y) and x axis
    double rotAngRad = atan2(y, x);
    // delta angle is angle between alpha arm at alpha axis
    // and vector towards (x,y)
    double deltaAngRad = acos(
        (beta2Fiber*beta2Fiber - xyMag*xyMag - alphaLen*alphaLen)/
        (-2*xyMag*alphaLen)
    );

    double alphaAngRad = rotAngRad - deltaAngRad;

    double betaAngDeg = betaAngRad * 180 / M_PI;
    double alphaAngDeg = alphaAngRad * 180 / M_PI;
    while (alphaAngDeg < 0){
        alphaAngDeg += 360;
    }
    std::array<double, 2> outArr = {alphaAngDeg, betaAngDeg};
    return outArr;
}

bool Robot::isValidTarget(double x, double y, int fiberID){
    // first a quick position cut
    double targDist = hypot(x - xPos, y - yPos);
    if (targDist > maxReach or targDist < minReach) {
        return false;
    }
    if (fiberID == AP_FIBER_ID and !hasApogee){
        return false;
    }
    auto ab = alphaBetaFromFiberXY(x, y, fiberID);
    // check alpha beta valid
    if (isnan(ab[0]) or isnan(ab[1])){
        return false;
    }
    // check alpha beta in range
    if (ab[0]<0 or ab[0]>=360){
        return false;
    }
    if (ab[1]<0 or ab[1]>180){
        return false;
    }
    // save current alpha beta
    double savedAlpha, savedBeta;
    savedAlpha = alpha;
    savedBeta = beta;
    setAlphaBeta(ab[0], ab[1]);
    bool isValid = true;
    if (isFiducialCollided()){
        isValid = false;
    }
    // reset alpha beta
    setAlphaBeta(savedAlpha, savedBeta);
    return isValid;
}

void Robot::assignTarget(int targetInd, double x, double y, int fiberID){
    if (!isValidTarget(x, y, fiberID)){
        throw std::runtime_error("assignTarget failure, target not valid");
    }
    assignedTargetInd = targetInd;
    auto ab = alphaBetaFromFiberXY(x, y, fiberID);
    setAlphaBeta(ab[0], ab[1]);
}

bool Robot::isAssigned(){
    return assignedTargetInd != 0;
}

// bool Robot::canSwapTarget(std::shared_ptr<Robot> robot){
//     if (!robot->isAssigned() or !isAssigned()){
//         return false;
//     }
//     if (robot->isValidTarget(assignedTarget) & isValidTarget(robot->assignedTarget)){
//         return true;
//     }
//     return false;
// }





