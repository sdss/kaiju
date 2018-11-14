#include "betaArm.h"

std::vector<double> curveRad, curveRadM1, curveRadM2, linRad5, linRad4, linRad3;
betaGeometry betaCurvePts, betaLinearPts5, betaLinearPts4, betaLinearPts3;

const int nCurvePts = 13;

// mm xyz points
const double CURVE_PTS[nCurvePts][3] = {
    {0.9895, 0,  7.8507},
    {2.3053, 0,  9.0969},
    {3.6596, 0,  10.5148},
    {5.2536, 0, 12.3113},
    {6.5389, 0, 14.0022},
    {7.6490, 0, 15.8129},
    {8.5728, 0, 17.7255},
    {9.3010, 0,  19.7206},
    {9.8264, 0, 21.7786},
    {10.1438, 0, 23.8787},
    {10.25, 0,  26},
    {10.25, 0, 30},
    {19.2-1.5, 0, 30} // 19.2 is flat end of head 1.5 is machined radius
};

const double CURVE_RAD[nCurvePts-1] = {
    3.4372, // base
    3.3300,
    3.1003,
    2.9949,
    2.9130,
    2.8715,
    2.8414,
    2.8200,
    2.8058,
    2.7977,
    2.7950,
    2.5 // beta head width / 2
};



void initBetaArms(){

    // curve arm geometries
    for (int ii=0; ii<nCurvePts; ii++){
        Eigen::Vector3d tempV(CURVE_PTS[ii]);
        betaCurvePts.push_back(tempV);
        if (ii<nCurvePts-1){
            curveRad.push_back(CURVE_RAD[ii]);
            curveRadM1.push_back(CURVE_RAD[ii] - 0.5);
            curveRadM2.push_back(CURVE_RAD[ii] - 1.0);
        }
    }

    // straight arm geometries
    Eigen::Vector3d origin, end1, end2, end3;
    origin << 0, 0, 0;
    end1 << 16.5 - 5.0/2.0, 0, 30;
    end2 << 16.5 - 4.0/2.0, 0, 30;
    end3 << 16.5 - 5.0/2.0, 0, 30;
    betaLinearPts5.push_back(origin);
    betaLinearPts5.push_back(end1);
    betaLinearPts4.push_back(origin);
    betaLinearPts4.push_back(end2);
    betaLinearPts3.push_back(origin);
    betaLinearPts3.push_back(end2);
    linRad5.push_back(2.5);
    linRad4.push_back(2.0);
    linRad3.push_back(1.5);

}

