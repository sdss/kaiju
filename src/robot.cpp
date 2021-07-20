#include <iostream>
#include <time.h>       /* time */
#include <cmath>
#include <thread>
#include <deque>
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

// const double alphaLen = 7.4;
// const double betaLen = 15; // mm to fiber

// conor's optimal data to obtain full extension for top fiber (ap)
// .75 is fiber to fiber spacing in snowflake
// .375 is half that
// const double met2sciX = sqrt(0.75*0.75 - 0.375*0.375);
// const double beta2sciX = sqrt(22.4*22.4-0.375*0.375) - alphaLen;
// const double beta2metX = 14.345792714510825; // beta2sciX - met2sciX;

// dist to metrology 14.345792714510825


// min reach is distance from alpha axis to science fiber when beta = 180
// const double minReach = hypot(beta2sciX, 0.375) - alphaLen;
// max reach is the distance from alpha axis to science fiber when beta = 0
// const double maxReach = betaLen + alphaLen;

// these max/min Reaches are the slightly more restrictive ones
// based on the small angular offset of the science fibers
// with respect to the beta axis...
// const double maxReach = hypot(alphaLen+beta2sciX, 0.375);
// const double minReach = hypot(alphaLen-beta2sciX, 0.375);

// const double maxReach = alphaLen + betaLen;
// const double minReach = betaLen - alphaLen;

// const double metFiberData[] = {beta2metX, 0, 0}; // from uw shop
// Eigen::Vector3d metFiberNeutral(metFiberData);

// const double apFiberData[] = {beta2sciX, 0.375, 0};
// Eigen::Vector3d apFiberNeutral(apFiberData);

// const double bossFiberData[] = {beta2sciX, -0.375, 0};
// Eigen::Vector3d bossFiberNeutral(bossFiberData);

// // create a vector that translates from beta orgin to alpha origin
// // this is just the length of the alpha arm
// const double alphaTransData[] = {alphaLen, 0, 0};
// Eigen::Vector3d alphaTransV(alphaTransData);

// const double metFiberData[] = {beta2metX, 0, 0}; // from uw shop
// vec3 metFiberNeutral(metFiberData);

// const double apFiberData[] = {beta2sciX, 0.375, 0};
// vec3 apFiberNeutral(apFiberData);

// const double bossFiberData[] = {beta2sciX, -0.375, 0};
// vec3 bossFiberNeutral(bossFiberData);

// // create a vector that translates from beta orgin to alpha origin
// // this is just the length of the alpha arm
// const double alphaTransData[] = {alphaLen, 0, 0};
// vec3 alphaTransV(alphaTransData);


// fix "neutral" positions in the global coord sys such that alpha==0
// points to -y (it was previously assumed that alpha==0 points toward +x)
// const double metFiberData[] = {0, -1*beta2metX, 0}; // from uw shop
// Eigen::Vector3d metFiberNeutral(metFiberData);

// const double apFiberData[] = {0.375, -1*beta2sciX, 0};
// Eigen::Vector3d apFiberNeutral(apFiberData);

// const double bossFiberData[] = {-0.375, -1*beta2sciX, 0};
// Eigen::Vector3d bossFiberNeutral(bossFiberData);

// // create a vector that translates from beta orgin to alpha origin
// // this is just the length of the alpha arm
// const double alphaTransData[] = {0, -1*alphaLen, 0};
// Eigen::Vector3d alphaTransV(alphaTransData);



// const std::array<vec3, 3> neutralFiberList{ {metFiberNeutral, apFiberNeutral, bossFiberNeutral} };

const int BOSS_FIBER_ID = 2;
const int AP_FIBER_ID = 1;
const int MET_FIBER_ID = 0;

// beta arm collision segments
// beta geometries in beta arm reference frame
// const double focalZ = 30; // height to focal plane (where fiber lives)
// const double betaAxis2End = 19.2 - 3.0; //mm
// const double betaEndRadius = 1.2; // mm

// const double betaEnvPt1Data[] = {0, 0, focalZ}; // beta axis
// vec3 betaEnvPt1(betaEnvPt1Data);
// const double betaEnvPt2Data[] = {betaAxis2End - betaEndRadius, 0, focalZ};
// vec3 betaEnvPt2(betaEnvPt2Data);

// again fix beta collision segment so it points in the right direction
// alpha==0 is global -y
// const double betaEnvPt1Data[] = {0, 0, focalZ}; // beta axis
// Eigen::Vector3d betaEnvPt1(betaEnvPt1Data);
// const double betaEnvPt2Data[] = {0, -1*(betaAxis2End - betaEndRadius), focalZ};
// Eigen::Vector3d betaEnvPt2(betaEnvPt2Data);


// const std::array<vec3, 2> neutralBetaCollisionSegment{ {betaEnvPt1, betaEnvPt2} };
// radius containing beta arm for collision detection
// const double betaCollisionRadius = 1.5; // mm (3mm wide)
// const double fiducialBuffer = 1.5; // 3mm wide fiducial

// xyz pos of fiber in beta neutra position
// const double fiberNeutral_data[] = {betaLen, 0, 0};
// Eigen::Vector3d fiberNeutral(fiberNeutral_data);

Robot::Robot(
    int id, std::string holeID, vec3 basePos, vec3 iHat, vec3 jHat, vec3 kHat,
    vec3 dxyz, double alphaLen, double alphaOffDeg, double betaOffDeg,
    double elementHeight, double scaleFac, vec2 metBetaXY, vec2 bossBetaXY, vec2 apBetaXY,
    std::array<vec2, 2> collisionSegBetaXY, double angStep, bool hasApogee
    ):
    id(id), holeID(holeID), basePos(basePos), iHat(iHat), jHat(jHat),
    kHat(kHat), dxyz(dxyz), alphaLen(alphaLen), alphaOffDeg(alphaOffDeg),
    betaOffDeg(betaOffDeg), elementHeight(elementHeight), scaleFac(scaleFac), metBetaXY(metBetaXY),
    bossBetaXY(bossBetaXY), apBetaXY(apBetaXY),
    collisionSegBetaXY(collisionSegBetaXY), angStep(angStep), hasApogee(hasApogee)
{
    // std::cout << "robot constructor called" << std::endl;
    // xPos = myxPos;
    // yPos = myyPos;
    // angStep = myAngStep;
    xPos = basePos[0];
    yPos = basePos[1];
    minReach = metBetaXY[0] - alphaLen; // close enough
    maxReach = metBetaXY[0] + alphaLen;

    // transXY = Eigen::Vector3d(xPos, yPos, 0);
    // id = myid;
    // hasApogee = myHasApogee;
    hasBoss = true; // break this out into a config/constructor
    // betaCollisionSegment = neutralBetaCollisionSegment;
    // std::pair<betaGeometry, std::vector<double>> betaPair = getBetaGeom(8);
    // betaModel = betaPair.first;
    // betaOrientation = betaPair.first;
    // modelRadii = betaPair.second;

    // alphaBetaArr <<  -angStep,  angStep,
    //                          0,  angStep,
    //                   angStep,  angStep,
    //                  -angStep,         0,
    //                   angStep,         0,
    //                  -angStep, -angStep,
    //                          0, -angStep,
    //                   angStep, -angStep;
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

void Robot::setDestinationAlphaBeta(double talpha, double tbeta){
    // enforce limits here?
    // double currAlpha = alpha;
    // double currBeta = beta;
    destinationAlpha = talpha;
    destinationBeta = tbeta;
    // setAlphaBeta(talpha, tbeta); // to set metFiberPos
    // targMetFiberPos = metFiberPos;
    hasDestinationAlphaBeta = true;
    // setAlphaBeta(currAlpha, currBeta);
}

void Robot::setAlphaBeta(double newAlpha, double newBeta){
    vec2 tmp2;
    vec3 tmp3;

    alpha = newAlpha;
    beta = newBeta;
    vec2 alphaBeta = {newAlpha, newBeta};

    // note we apply the offsets in the commands to the
    // positioners, not here...
    double alphaOffNull = 0;
    double betaOffNull = 0;

    // metrology fiber
    tmp2 = positionerToTangent(
        alphaBeta, metBetaXY, alphaLen, alphaOffNull, betaOffNull
    );
    tmp3 = {tmp2[0], tmp2[1], 0};
    metWokXYZ = tangentToWok(
        tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );

    // boss fiber
    tmp2 = positionerToTangent(
        alphaBeta, bossBetaXY, alphaLen, alphaOffNull, betaOffNull
    );
    tmp3 = {tmp2[0], tmp2[1], 0};
    bossWokXYZ = tangentToWok(
        tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );

    // apogee fiber
    tmp2 = positionerToTangent(
        alphaBeta, apBetaXY, alphaLen, alphaOffNull, betaOffNull
    );
    tmp3 = {tmp2[0], tmp2[1], 0};
    apWokXYZ = tangentToWok(
        tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );

    // collision segment
    tmp2 = positionerToTangent(
        alphaBeta, collisionSegBetaXY[0], alphaLen, alphaOffNull, betaOffNull
    );
    tmp3 = {tmp2[0], tmp2[1], 0};
    collisionSegWokXYZ[0] = tangentToWok(
        tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );

    tmp2 = positionerToTangent(
        alphaBeta, collisionSegBetaXY[1], alphaLen, alphaOffNull, betaOffNull
    );
    tmp3 = {tmp2[0], tmp2[1], 0};
    collisionSegWokXYZ[1] = tangentToWok(
        tmp3, basePos, iHat, jHat, kHat, elementHeight, scaleFac,
        dxyz[0], dxyz[1], dxyz[2]
    );
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

    // note we apply the offsets in the commands to the
    // positioners, not here...
    double alphaOffNull = 0;
    double betaOffNull = 0;

    ab = tangentToPositioner(
        xyTangent, metBetaXY, alphaLen, alphaOffNull, betaOffNull
    );

    while (std::isnan(ab[0]) or std::isnan(ab[1])){
        xyTangent = sampleAnnulus(minReach, maxReach);
        // use a science fiber ID (matches min/max reach)
        ab = tangentToPositioner(
            xyTangent, metBetaXY, alphaLen, alphaOffNull, betaOffNull
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

    // alphaVel.push_back(0);
    // betaVel.push_back(0);

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
        tangentXY, fibBetaXY, alphaLen, alphaOffDeg, betaOffDeg
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





