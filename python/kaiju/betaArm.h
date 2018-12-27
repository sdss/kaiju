#pragma once
#include <vector>
#include <array>
#include "eigen3/Eigen/Dense"

typedef std::vector<Eigen::Vector3d> betaGeometry;
// collision radii as a function of z
// these apply to segments (between points) in the CURVE_PTS
extern std::vector<double> curveRad, curveRadM1, curveRadM2, linRad5, linRad4, linRad3; //betaRadVec;
extern betaGeometry betaBlockPts, betaCurvePts, betaLinearPts5, betaLinearPts4, betaLinearPts3; //betaArmPts;
extern int betaGeomID; // describes the chosen beta arm geometry

void initBetaArms();
std::pair<betaGeometry, std::vector<double>> getBetaGeom(int betaGeom);
// void setBetaGeom(int betaGeom);

