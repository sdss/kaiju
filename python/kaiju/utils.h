# pragma once
#include <stdio.h>
#include <vector>
#include <array>
#include <eigen3/Eigen/Dense>

// #define SMALL_NUM   0.00000001 // anything that avoids division overflow
const double SMALL_NUM = 0.00000001; // anything that avoids division overflow

std::array<double, 2> sampleAnnulus(double rMin, double rMax);

Eigen::MatrixXd getHexPositions(int nDia, double pitch);

double linearInterpolate(std::vector<Eigen::Vector2d> & sparseXYPoints, double xValue);

double dist3D_Segment_to_Segment(
    Eigen::Vector3d S1_P0, Eigen::Vector3d S1_P1,
    Eigen::Vector3d S2_P0, Eigen::Vector3d S2_P1);

double PerpendicularDistance(const Eigen::Vector2d &pt, const Eigen::Vector2d &lineStart, const Eigen::Vector2d &lineEnd);

void RamerDouglasPeucker(const std::vector<Eigen::Vector2d> &pointList, double epsilon, std::vector<Eigen::Vector2d> &out);

double randomSample();

// double meanErrorRMD(
//     // return the mean error between the rmd simplified line
//     // and the original line, we'll use this to globally
//     // shift the simplified line to one side
//     const std::vector<Eigen::Vector2d> &rmdInterpPoints,
//     const std::vector<Eigen::Vector2d> &pathGenPoints,
//     )