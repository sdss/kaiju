    #include <iostream>
#include <time.h>       /* time */
#include <cmath>
#include <thread>
#include <deque>
#include "utils.h"
#include "robot.h"
#include "robotGrid.h"


const int BOSS_FIBER_ID = 2;
const int AP_FIBER_ID = 1;
const int MET_FIBER_ID = 0;


Robot::Robot(
    int id, std::string holeID, vec3 basePos, vec3 iHat, vec3 jHat, vec3 kHat,
    vec3 dxyz, double alphaLen, double alphaOffDeg, double betaOffDeg,
    double elementHeight, double scaleFac, vec2 metBetaXY, vec2 bossBetaXY, vec2 apBetaXY,
    std::array<vec2, 2> collisionSegBetaXY, double angStep, bool hasApogee, double collisionBuffer, bool lefthanded
    ):
    id(id), holeID(holeID), basePos(basePos), iHat(iHat), jHat(jHat),
    kHat(kHat), dxyz(dxyz), alphaLen(alphaLen), alphaOffDeg(alphaOffDeg),
    betaOffDeg(betaOffDeg), elementHeight(elementHeight), scaleFac(scaleFac), metBetaXY(metBetaXY),
    bossBetaXY(bossBetaXY), apBetaXY(apBetaXY),
    collisionSegBetaXY(collisionSegBetaXY), angStep(angStep), hasApogee(hasApogee), collisionBuffer(collisionBuffer),
    lefthanded(lefthanded)
{

    xPos = basePos[0];
    yPos = basePos[1];
    minReach = metBetaXY[0] - alphaLen; // close enough
    maxReach = metBetaXY[0] + alphaLen;
    betaLen = collisionSegBetaXY[1][0];


    hasBoss = true; // break this out into a config/constructor?
}

void Robot::setCollisionBuffer(double newBuffer){
    // perhaps modify collision segment based on
    // collision buffer?
    collisionBuffer = newBuffer;
}


void Robot::setFiberToWokXYZ(vec3 wokXYZ, FiberType fiberID){
    // warning doesn't check for collisions

    auto newAlphaBeta = alphaBetaFromWokXYZ(wokXYZ, fiberID);
    if (std::isnan(newAlphaBeta[0]) or std::isnan(newAlphaBeta[1])){
        throw std::runtime_error("cannot reach target xy");
    }
    setAlphaBeta(newAlphaBeta[0], newAlphaBeta[1]);

}

double Robot::score(){
    // if robot is offline, force score to always be zero
    if (isOffline){
        return 0;
    }
    double alphaDist = alpha - destinationAlpha;
    double betaDist = beta - destinationBeta;
    return alphaDist*alphaDist + betaDist*betaDist;
}


double Robot::getMaxReach() {
	return(maxReach);
}

void Robot::addRobotNeighbor(int robotID){
    robotNeighbors.push_back(robotID);
}

void Robot::addFiducialNeighbor(int fiducialID){
    fiducialNeighbors.push_back(fiducialID);
}

void Robot::addGFANeighbor(int fiducialID){
    gfaNeighbors.push_back(fiducialID);
}

void Robot::setDestinationAlphaBeta(double talpha, double tbeta){
    // enforce limits here?

    destinationAlpha = talpha;
    destinationBeta = tbeta;

    hasDestinationAlphaBeta = true;
}

void Robot::setGreedPhobia(double newGreed, double newPhobia){
    greed = newGreed;
    phobia = newPhobia;
}

void Robot::saveAlphaBeta(){
    alphaInit = alpha;
    betaInit = beta;
}

void Robot::setAlphaBeta(double newAlpha, double newBeta){
    vec2 tmp2;
    vec3 tmp3;

    alpha = newAlpha;
    beta = newBeta;
    vec2 alphaBeta = {newAlpha, newBeta};

    // // metrology fiber
    tmp2 = positionerToTangent(
        alphaBeta, metBetaXY, alphaLen, alphaOffDeg, betaOffDeg
    );
    tmp3 = {tmp2[0], tmp2[1], 0};
    metWokXYZ = tangentToWok(
        tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );

    // boss fiber
    tmp2 = positionerToTangent(
        alphaBeta, bossBetaXY, alphaLen, alphaOffDeg, betaOffDeg
    );
    tmp3 = {tmp2[0], tmp2[1], 0};
    bossWokXYZ = tangentToWok(
        tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );

    // apogee fiber
    tmp2 = positionerToTangent(
        alphaBeta, apBetaXY, alphaLen, alphaOffDeg, betaOffDeg
    );
    tmp3 = {tmp2[0], tmp2[1], 0};
    apWokXYZ = tangentToWok(
        tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );

    // collision segment
    tmp2 = positionerToTangent(
        alphaBeta, collisionSegBetaXY[0], alphaLen, alphaOffDeg, betaOffDeg
    );
    tmp3 = {tmp2[0], tmp2[1], 0};
    collisionSegWokXYZ[0] = tangentToWok(
        tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );

    tmp2 = positionerToTangent(
        alphaBeta, collisionSegBetaXY[1], alphaLen, alphaOffDeg, betaOffDeg
    );
    tmp3 = {tmp2[0], tmp2[1], 0};
    collisionSegWokXYZ[1] = tangentToWok(
        tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );

}

void Robot::setAlphaBetaFast(double newAlpha, double newBeta){
    vec2 tmp2;
    vec3 tmp3;

    alpha = newAlpha;
    beta = newBeta;
    vec2 alphaBeta = {newAlpha, newBeta};

    // // metrology fiber
    // tmp2 = positionerToTangent(
    //     alphaBeta, metBetaXY, alphaLen, alphaOffDeg, betaOffDeg
    // );
    // tmp3 = {tmp2[0], tmp2[1], 0};
    // metWokXYZ = tangentToWok(
    //     tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
    //     dxyz[0], dxyz[1], dxyz[2]
    // );

    // // boss fiber
    // tmp2 = positionerToTangent(
    //     alphaBeta, bossBetaXY, alphaLen, alphaOffDeg, betaOffDeg
    // );
    // tmp3 = {tmp2[0], tmp2[1], 0};
    // bossWokXYZ = tangentToWok(
    //     tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
    //     dxyz[0], dxyz[1], dxyz[2]
    // );

    // // apogee fiber
    // tmp2 = positionerToTangent(
    //     alphaBeta, apBetaXY, alphaLen, alphaOffDeg, betaOffDeg
    // );
    // tmp3 = {tmp2[0], tmp2[1], 0};
    // apWokXYZ = tangentToWok(
    //     tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
    //     dxyz[0], dxyz[1], dxyz[2]
    // );

    // collision segment
    // collisionSegWokXYZ[0] = positionerToWok(
    //     alphaBeta,
    //     collisionSegBetaXY[0],
    //     alphaLen,
    //     alphaOffDeg,
    //     betaOffDeg,
    //     basePos,
    //     elementHeight,
    //     dxyz[0],
    //     dxyz[1]
    // );

    // collisionSegWokXYZ[1] = positionerToWok(
    //     alphaBeta,
    //     collisionSegBetaXY[1],
    //     alphaLen,
    //     alphaOffDeg,
    //     betaOffDeg,
    //     basePos,
    //     elementHeight,
    //     dxyz[0],
    //     dxyz[1]
    // );

    tmp2 = positionerToTangent(
        alphaBeta, collisionSegBetaXY[0], alphaLen, alphaOffDeg, betaOffDeg
    );
    tmp3 = {tmp2[0], tmp2[1], 0};
    collisionSegWokXYZ[0] = tangentToWok(
        tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );

    tmp2 = positionerToTangent(
        alphaBeta, collisionSegBetaXY[1], alphaLen, alphaOffDeg, betaOffDeg
    );
    tmp3 = {tmp2[0], tmp2[1], 0};
    collisionSegWokXYZ[1] = tangentToWok(
        tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );
}

bool Robot::isValidDither(vec2 newAlphaBeta){
    // check that there is no alpha or beta wrapping happening
    if (std::isnan(newAlphaBeta[0]) or std::isnan(newAlphaBeta[1])){
        return false;
    }
    if (std::abs(newAlphaBeta[0] - alpha) > 90){
        return false;
    }
    if (std::abs(newAlphaBeta[1] - beta) > 90){
        return false;
    }

    return true;



}

vec2 Robot::uniformDither(double radius){ //, FiberType fiberType){
    // warning no collisions are checked in this!
    vec3 fibWokXYZ;
    vec2 xyTangentNew, xyTangentBase, fiberBaseXY, dxy, ab;


    fibWokXYZ = apWokXYZ;
    fiberBaseXY = apBetaXY;

    auto tangentXYZ = wokToTangent(
        fibWokXYZ, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );

    xyTangentBase[0] = tangentXYZ[0];
    xyTangentBase[1] = tangentXYZ[1];

    dxy = sampleAnnulus(0, radius);
    xyTangentNew[0] = xyTangentBase[0] + dxy[0];
    xyTangentNew[1] = xyTangentBase[1] + dxy[1];

    ab = tangentToPositioner(
        xyTangentNew, fiberBaseXY, alphaLen, alphaOffDeg, betaOffDeg, lefthanded
    );

    while (!isValidDither(ab)){
        dxy = sampleAnnulus(0, radius);
        xyTangentNew[0] = xyTangentBase[0] + dxy[0];
        xyTangentNew[1] = xyTangentBase[1] + dxy[1];

        ab = tangentToPositioner(
            xyTangentNew, fiberBaseXY, alphaLen, alphaOffDeg, betaOffDeg, lefthanded
        );
    }

    return ab;
}

vec2 Robot::randomXYUniform(){
	vec2 xy = sampleAnnulus(minReach, maxReach);
    return xy;
}

void Robot::setXYUniform(){
    // perhaps get rid of this and just use setAlphaBetaRand()?
    vec2 xyTangent, ab;
    xyTangent = sampleAnnulus(minReach, maxReach);
    // use a science fiber ID (matches min/max reach)
    // std::cout.precision(20);


    ab = tangentToPositioner(
        xyTangent, metBetaXY, alphaLen, alphaOffDeg, betaOffDeg, lefthanded
    );

    while (std::isnan(ab[0]) or std::isnan(ab[1])){
        xyTangent = sampleAnnulus(minReach, maxReach);
        // use a science fiber ID (matches min/max reach)
        ab = tangentToPositioner(
            xyTangent, metBetaXY, alphaLen, alphaOffDeg, betaOffDeg, lefthanded
        );
    }

    setAlphaBeta(ab[0], ab[1]);
}


void Robot::smoothVelocity(int points){

    if (alphaPath.size()==0){
        throw std::runtime_error("Cannot smooth, no alphaPath, do path gen first");
    }


    // std::vector<double> movingWindow;
    std::vector<double> bufferedAlphaVel;
    std::vector<double> bufferedBetaVel;
    vec2 temp;

    // extend path on left (fake constant speed)
    // as if the positioner kept moving to make sure
    // we don't decelerate to a stop

    // target position
    double alphaStart = alphaPath[0][1];
    double betaStart = betaPath[0][1];
    double alphaEnd = alphaPath.back()[1];
    double betaEnd = betaPath.back()[1];

    for (int ii=0; ii < points; ii++){
        // movingWindow.push_back(1/float(points));
        // beta is moving towards 180 (positive)
        // alpha is moving towards 0 (negative)
        bufferedAlphaVel.push_back(-angStep);
        bufferedBetaVel.push_back(angStep);
    }


    // begin creating the buffered velocity starts at 0

    // calculate velocity vs step
    // add tail points to alpha and beta paths
    int tailPoints = 400; // should be more than enough
    auto apCopy = alphaPath;
    auto bpCopy = betaPath;
    auto apLast = alphaPath.back();
    auto bpLast = betaPath.back();
    for (int ii=0; ii < tailPoints; ii++){
        apCopy.push_back(apLast);
        bpCopy.push_back(bpLast);
    }

    for (int ii=1; ii < apCopy.size(); ii++){
        double av = apCopy[ii][1] - apCopy[ii-1][1];
        double bv = bpCopy[ii][1] - bpCopy[ii-1][1];
        bufferedAlphaVel.push_back(av);
        bufferedBetaVel.push_back(bv);
        // unbuffered version, for plotting if ya want
        alphaVel.push_back(av);
        betaVel.push_back(bv);
    }


    // do i need to add zeros to alpha/betaVel

    // add zero buffer to end of velocity array to make sure convolution
    // works
    double lastAlphaVel = bufferedAlphaVel.back();
    double lastBetaVel = bufferedBetaVel.back();
    for (int ii=0; ii < points; ii++){
        bufferedAlphaVel.push_back(lastAlphaVel); // add zeros on right of velocities
        bufferedBetaVel.push_back(lastBetaVel);
    }


    for (int ii=points; ii < bufferedAlphaVel.size()-points; ii++){

        double alphaAvg = 0;
        double betaAvg = 0;
        int pp = 0;
        for (int jj=0; jj<points; jj++){
            // choose numerically stable average (not yet implemented?)
            if (jj==0){
                alphaAvg += bufferedAlphaVel[jj+ii];
                betaAvg += bufferedBetaVel[jj+ii];
                pp++;
            }
            else{
                alphaAvg += bufferedAlphaVel[ii-jj];
                betaAvg += bufferedBetaVel[ii-jj];
                alphaAvg += bufferedAlphaVel[ii+jj];
                betaAvg += bufferedBetaVel[ii+jj];
                pp++;
                pp++;
            }
        }
        // std::cout << "points averaged " << pp << std::endl;
        alphaAvg = alphaAvg / ((float)pp);
        betaAvg = betaAvg / ((float)pp);
        smoothAlphaVel.push_back(alphaAvg);
        smoothBetaVel.push_back(betaAvg);

    }

    // convert back into position / time series, a cumulative sum
    double cumSumAlpha = alphaStart;
    double cumSumBeta = betaStart;
    int cumStep = 0;
    temp[0] = cumStep;
    temp[1] = cumSumAlpha;
    smoothedAlphaPath.push_back(temp);
    temp[1] = cumSumBeta;
    smoothedBetaPath.push_back(temp);
    cumStep++;

    for (int ii=0; ii < smoothAlphaVel.size(); ii++){
        cumSumAlpha += smoothAlphaVel[ii];
        cumSumBeta += smoothBetaVel[ii];
        // std::cout << "beta pos " << cumSumBeta << std::endl;
        temp[0] = cumStep;
        temp[1] = cumSumAlpha;
        smoothedAlphaPath.push_back(temp);
        temp[1] = cumSumBeta;
        smoothedBetaPath.push_back(temp);
        cumStep++;
    }

    // edge effects can have a small effect (eg last folded positioner) will
    // show a phantom deceleration on the last step due to the fact that its
    // last step is probably shorter than a full step...
    // to force the path generator to build a path to the exact end
    // add one final point with the exact ending point, don't do this
    // if the grid failed to converge!!! also the integration can
    // add a small error in position (like 1e-16) so just tag it
    // for the rmd smoother
    temp[0] = cumStep;
    temp[1] = alphaEnd;
    smoothedAlphaPath.push_back(temp);
    temp[1] = betaEnd;
    smoothedBetaPath.push_back(temp);

}

void Robot::simplifyPath(double epsilon){
    // smooth a previously generated path
    double interpSimplifiedAlpha, interpSimplifiedBeta;

    if (alphaPath.size()==0){
        throw std::runtime_error("Cannot simplify, no smoothed paths, pathgen, and smooth first");
    }
    // int npts;
    vec2 atemp, btemp;

    // because we extended the tail of the smoothed alpha beta paths to
    // allow deceleration, begin truncating it until we detect that the
    // smooth path not longer achieves the final position, this makes
    // the shortest move possible

    auto cpSAP = smoothedAlphaPath;
    auto cpSBP = smoothedBetaPath;
    auto alphaTarg = smoothedAlphaPath.back()[1];
    auto betaTarg = smoothedBetaPath.back()[1];
    double alphaLast, betaLast;
    std::vector<vec2> tempSimpAlphaPath, tempSimpBetaPath;

    RamerDouglasPeucker(smoothedAlphaPath, epsilon, simplifiedAlphaPath);
    RamerDouglasPeucker(smoothedBetaPath, epsilon, simplifiedBetaPath);

    // loop through the smoothed path and add an extra anchor to the
    // RMD points (were the robot first hits the target), this is to
    // send the shortest path possible to the bot.
    vec2 insertAlpha, insertBeta, currPos;


    // calculate simplified alpha betas at every step
    // this is used for collision detection after smoothing
    // if alphaPath point is outside interpolation range
    // then simply extrapolate that postion
    int nDensePoints = alphaPath.size();
    for (int ii=0; ii<nDensePoints; ii++){
        double xVal = alphaPath[ii][0];
        atemp[0] = xVal; // interpolation step
        btemp[0] = xVal;
        interpSimplifiedAlpha = linearInterpolate(simplifiedAlphaPath, xVal);
        // bias alpha in positive direction because we're approaching zero
        atemp[1] = interpSimplifiedAlpha;
        interpSimplifiedAlphaPath.push_back(atemp);
        interpSimplifiedBeta = linearInterpolate(simplifiedBetaPath, xVal);
        btemp[1] = interpSimplifiedBeta;
        interpSimplifiedBetaPath.push_back(btemp);

        // populate interpXY points for alpha/beta ends
        setAlphaBeta(interpSimplifiedAlpha, interpSimplifiedBeta);
        atemp[1] = collisionSegWokXYZ[0][0]; // xAlphaEnd
        interpAlphaX.push_back(atemp);
        atemp[1] = collisionSegWokXYZ[0][1]; // yAlphaEnd
        interpAlphaY.push_back(atemp);
        atemp[1] = collisionSegWokXYZ.back()[0]; // xBetaEnd
        interpBetaX.push_back(atemp);
        atemp[1] = collisionSegWokXYZ.back()[1]; // yBetaEnd
        interpBetaY.push_back(atemp);

    }

}


vec2 Robot::alphaBetaFromWokXYZ(vec3 wokXYZ, FiberType fiberType){
    // origin is at alpha axis
    // +x is aligned with alpha angle = 0

    vec2 tangentXY, fibBetaXY;

    auto tangentXYZ = wokToTangent(
        wokXYZ, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );

    // ignoring projection factor for now?
    tangentXY = {tangentXYZ[0], tangentXYZ[1]};

    if (fiberType == MetrologyFiber) {
        fibBetaXY = metBetaXY;
    }
    else if (fiberType == ApogeeFiber){
        fibBetaXY = apBetaXY;
    }
    else {
        // boss fiber
        fibBetaXY = bossBetaXY;
    }

    auto alphaBeta = tangentToPositioner(
        tangentXY, fibBetaXY, alphaLen, alphaOffDeg, betaOffDeg, lefthanded
    );

    return alphaBeta;
}

void Robot::assignTarget(long targetID){
    // assigns the target and set alpha beta accordingly
    int ii = std::count(validTargetIDs.begin(), validTargetIDs.end(), targetID);
    if (ii == 0){
        throw std::runtime_error("assignTarget failure, invalid target");
    }
    assignedTargetID = targetID;
}

bool Robot::isAssigned(){
    return assignedTargetID != -1;
}

void Robot::clearAssignment(){
    assignedTargetID = -1;
}





