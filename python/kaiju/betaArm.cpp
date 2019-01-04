#include <stdio.h>
#include "betaArm.h"

std::vector<double> curveRad, curveRadM1, curveRadM2, linRad5, linRad4, linRad3, linRad1; // betaRadVec;
betaGeometry betaBlockPts, betaCurvePts, betaLinearPts5, betaLinearPts4, betaLinearPts3; // betaArmPts;
int betaGeomID;

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
    {19.2-3.0-1.5, 0, 30} // 19.2 is flat end of head offset by 3mm (center of beta axis) 1.5 is machined radius
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
    // 2.5 // beta head width / 2
    2.61,
};



void initBetaArms(){
    Eigen::Vector3d origin, end1, end2, end3, blockOrigin, blockEnd;

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
    // angled cylinders
    origin << 0, 0, 0;
    end1 << 16.5 - 5.0/2.0, 0, 30;
    end2 << 16.5 - 4.0/2.0, 0, 30;
    end3 << 16.5 - 3.0/2.0, 0, 30;
    betaLinearPts5.push_back(origin);
    betaLinearPts5.push_back(end1);
    betaLinearPts4.push_back(origin);
    betaLinearPts4.push_back(end2);
    betaLinearPts3.push_back(origin);
    betaLinearPts3.push_back(end3);
    linRad5.push_back(2.5);
    linRad4.push_back(2.0);
    linRad3.push_back(1.5);
    linRad1.push_back(0.5);

    // block geometry
    blockOrigin << 0, 0, 30;
    blockEnd << 19.2 - 3.0 - 1.5, 0, 30; // 1.5 is radius
    betaBlockPts.push_back(blockOrigin);
    betaBlockPts.push_back(blockEnd);

    // default to P1 geometry wide block
    // setBetaGeom(6);

}

std::pair<betaGeometry, std::vector<double>> getBetaGeom(int betaGeom){
    // beta geom lookup list
    // 0: P1 gemoetry
    // 1: P1 geometry with collision radius shrunk by 0.5mm
    // 2: P1 geometry with collision radius shrunk by 1mm
    // 3: linear cylindrical arm with radius 5mm
    // 4: linear cylindrical arm with radius 4mm
    // 5: linear cylndrical arm with radius 3mm
    // 6: block geom, 5mm width
    // 7: block geom, 4mm width
    // 8: block geom, 3mm width
    // 9: block geom, 1mm width
    std::pair<betaGeometry, std::vector<double>> output;
    if (betaGeom==0){
        output.first = betaCurvePts;
        output.second = curveRad;
        // betaRadVec = curveRad;
        // betaArmPts = betaCurvePts;
    }
    else if (betaGeom==1){
        output.first = betaCurvePts;
        output.second = curveRadM1;
        // betaRadVec = curveRadM1;
        // betaArmPts = betaCurvePts;
    }
    else if (betaGeom==2){
        output.first = betaCurvePts;
        output.second = curveRadM2;
        // betaRadVec = curveRadM2;
        // betaArmPts = betaCurvePts;
    }
    else if (betaGeom==3){
        output.first = betaLinearPts5;
        output.second = linRad5;
        // betaRadVec = linRad5;
        // betaArmPts = betaLinearPts5;
    }
    else if (betaGeom==4){
        output.first = betaLinearPts4;
        output.second = linRad4;
        // betaRadVec = linRad4;
        // betaArmPts = betaLinearPts4;
    }
    else if (betaGeom==5){
        output.first = betaLinearPts3;
        output.second = linRad3;
        // betaRadVec = linRad3;
        // betaArmPts = betaLinearPts3;
    }
    else if (betaGeom==6){
        output.first = betaBlockPts;
        output.second = linRad5;
        // betaRadVec = linRad5;
        // betaArmPts = betaBlockPts;
    }
    else if (betaGeom==7){
        output.first = betaBlockPts;
        output.second = linRad4;
        // betaRadVec = linRad4;
        // betaArmPts = betaBlockPts;
    }
    else if (betaGeom==8){
        output.first = betaBlockPts;
        output.second = linRad3;
        // betaRadVec = linRad3;
        // betaArmPts = betaBlockPts;
    }
    else if (betaGeom==9){
        output.first = betaBlockPts;
        output.second = linRad1;
        // betaRadVec = linRad3;
        // betaArmPts = betaBlockPts;
    }
    else {
        throw std::runtime_error("invalid betaGeom!");
    }
    betaGeomID = betaGeom;
    return output;
}

