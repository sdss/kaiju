#include <iostream>
#include <time.h>       /* time */
#include <cmath>
#include <thread>
#include "utils.h"
#include "robot.h"
#include "robotGrid.h"


// alpha beta array - the ordered list of moves to try
const double ab_data[] = {
    -ang_step,  ang_step,
            0,  ang_step,
     ang_step,  ang_step,
    -ang_step,         0,
     ang_step,         0,
    -ang_step, -ang_step,
            0, -ang_step,
     ang_step, -ang_step
};

Eigen::Array<double, 8, 2> alphaBetaArr(ab_data);

// xyz pos of fiber in beta neutra position
const double fiberNeutral_data[] = {beta_arm_len, 0, 0};
Eigen::Vector3d fiberNeutral(fiberNeutral_data);


// create a vector that translates from beta orgin to alpha origin
// this is just the length of the alpha arm
const double alpha_trans_data[] = {alpha_arm_len, 0, 0};
Eigen::Vector3d alphaTransV(alpha_trans_data);


// move this to robot class?
// 0,0 is origin of
std::array<double, 2> alphaBetaFromXY(double x, double y){
    // law of cosines at work here...
    double xyMag = hypot(x, y);
    double alphaAngRad = acos(
        (-1*beta_arm_len*beta_arm_len + alpha_arm_len*alpha_arm_len + xyMag*xyMag)/(2*alpha_arm_len*xyMag)
    );
    double gammaAngRad = acos(
        (-1*xyMag*xyMag + alpha_arm_len*alpha_arm_len + beta_arm_len*beta_arm_len)/
        (2*alpha_arm_len*beta_arm_len)
    );
    alphaAngRad = -1*alphaAngRad;
    double betaAngRad = M_PI - gammaAngRad;
    double betaAngDeg = betaAngRad * 180 / M_PI;
    double rotAng = atan2(y, x);
    alphaAngRad = alphaAngRad + rotAng;
    double alphaAngDeg = alphaAngRad * 180 / M_PI;
    while (alphaAngDeg < 0){
        alphaAngDeg += 360;
    }
    std::array<double, 2> outArr;
    outArr[0] = alphaAngDeg;
    outArr[1] = betaAngDeg;
    return outArr;
}


Robot::Robot(int myid, double myxPos, double myyPos) {
    xPos = myxPos;
    yPos = myyPos;
    transXY = Eigen::Vector3d(myxPos, myyPos, 0);
    id = myid;
    betaModel = betaArmPts;
    betaOrientation = betaArmPts;
    modelRadii = betaRadVec;
}

void Robot::pathToFile(){
    FILE * pFile;
    char buffer[50];
    sprintf(buffer, "pathAlpha_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (auto & point : alphaPath){
        fprintf(pFile, "%.2f %.8f\n", point(0), point(1));
    }
    fclose(pFile);

    sprintf(buffer, "pathBeta_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (auto & point : betaPath){
        fprintf(pFile, "%.2f %.8f\n", point(0), point(1));
    }
    fclose(pFile);
}

void Robot::smoothPathToFile(){
    FILE * pFile;
    char buffer[50];
    sprintf(buffer, "smoothpathAlpha_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (auto & point : smoothAlphaPath){
        fprintf(pFile, "%.2f %.8f\n", point(0), point(1));
    }
    fclose(pFile);

    sprintf(buffer, "smoothpathBeta_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (auto & point : smoothBetaPath){
        fprintf(pFile, "%.2f %.8f\n", point(0), point(1));
    }
    fclose(pFile);
}

void Robot::ismoothPathToFile(){
    FILE * pFile;
    char buffer[50];
    sprintf(buffer, "ismoothpathAlpha_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (auto & point : interpSmoothAlphaPath){
        fprintf(pFile, "%.2f %.8f\n", point(0), point(1));
    }
    fclose(pFile);

    sprintf(buffer, "ismoothpathBeta_%04d.txt", id);
    pFile = fopen(buffer, "w");
    for (auto & point : interpSmoothBetaPath){
        fprintf(pFile, "%.2f %.8f\n", point(0), point(1));
    }
    fclose(pFile);
}

void Robot::addNeighbor(Robot * rNeighbor){
    neighbors.push_back(rNeighbor);
}


void Robot::setAlphaBeta(double newAlpha, double newBeta){
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
    for (int ii = 0; ii < betaModel.size(); ii++){
        // first rotate about beta (we start in beta ref frame)
        // next translate in x by the alpha arm length
        // we should be able to figure out how to compute
        // this matrix outside this loop....
        betaOrientation[ii] = transXY + (alphaRot * (alphaTransV + (betaRot * betaModel[ii])));

        // std::cout << "test : " << betaOrientation[ii] <<  " : test\n" << std::endl;
    }
    fiber_XYZ = transXY + (alphaRot * (alphaTransV + (betaRot * fiberNeutral)));

}

void Robot::setAlphaBetaRand(){
    double a = randomSample() * 359.99999;
    double b = randomSample() * 180.0;
    setAlphaBeta(a, b);
}

void Robot::setXYUniform(){
    std::array<double, 2> xy = sampleAnnulus(min_reach, max_reach);
    std::array<double, 2> ab = alphaBetaFromXY(xy[0], xy[1]);
    setAlphaBeta(ab[0], ab[1]);
}


bool Robot::isCollided(double radiusBuffer){
    double dist2, rad1, rad2, collideDist2;
    bool iAmCollided = false;
    for (Robot * robot : neighbors){
        for (int ii=0; ii<betaOrientation.size()-1; ii++){
            dist2 = dist3D_Segment_to_Segment(
                    betaOrientation[ii], betaOrientation[ii+1],
                    robot->betaOrientation[ii], robot->betaOrientation[ii+1]
                );
            rad1 = modelRadii[ii];
            rad2 = robot->modelRadii[ii];
            collideDist2 = (rad1+rad2+radiusBuffer)*(rad1+rad2+radiusBuffer);
            if (dist2 < collideDist2){
                // std::cout << "dist " << dist2 - collide_dist_squared << std::endl;
                iAmCollided = true;
                break;
            }
        }
    }
    return iAmCollided;
}



void Robot::decollide(){
    // randomly replace alpha beta values until collisions vanish
    int ii;
    for (ii=0; ii<300; ii++){
        setXYUniform();
        if (!isCollided()){
            break;
        }
    }
}


void Robot::smoothPath(){
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
    }


}

void Robot::stepTowardFold(int stepNum){
    double currAlpha = alpha;
    double currBeta = beta;
    Eigen::Vector2d alphaPathPoint;
    Eigen::Vector2d betaPathPoint;
    alphaPathPoint(0) = stepNum;
    betaPathPoint(0) = stepNum;
    if (currBeta==180 and currAlpha==0){
        // done folding don't move
        alphaPathPoint(1) = currAlpha;
        betaPathPoint(1) = currBeta;
        alphaPath.push_back(alphaPathPoint);
        betaPath.push_back(betaPathPoint);
        return;
    }

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
            alphaPathPoint(1) = currAlpha;
            betaPathPoint(1) = currBeta;
            alphaPath.push_back(alphaPathPoint);
            betaPath.push_back(betaPathPoint);
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
}

bool robotSort(const Robot& robot1, const Robot& robot2){
    if (robot1.beta == 180.0 and robot2.beta < 180.0){
        return false;
    }
    else if (robot1.beta < 180.0 and robot2.beta == 180.0) {
        return true;
    }


    // steps to go
    double r1steps = robot1.alpha + 180 - robot1.beta;
    double r2steps = robot2.alpha + 180 - robot2.beta;
    return (r1steps > r2steps);
}




