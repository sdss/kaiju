# pragma once
#include <stdio.h>
#include <vector>
#include <array>
// #include <Eigen/Dense>
#include "coordio.h"

// #define SMALL_NUM   0.00000001 // anything that avoids division overflow
const double SMALL_NUM = 0.00000001; // anything that avoids division overflow

std::array<double, 2> sampleAnnulus(double rMin, double rMax);

// Eigen::MatrixXd getHexPositions(int nDia, double pitch);

double linearInterpolate(std::vector<vec2> & sparseXYPoints, double xValue);

double dist3D_Segment_to_Segment(
    vec3 S1_P0, vec3 S1_P1,
    vec3 S2_P0, vec3 S2_P1);

double dist3D_Point_to_Segment( vec3 Point, vec3 Seg_P0, vec3 Seg_P1);

double PerpendicularDistance(const vec2 &pt, const vec2 &lineStart, const vec2 &lineEnd);

void RamerDouglasPeucker(const std::vector<vec2> &pointList, double epsilon, std::vector<vec2> &out);

double randomSample();

// double meanErrorRMD(
//     // return the mean error between the rmd simplified line
//     // and the original line, we'll use this to globally
//     // shift the simplified line to one side
//     const std::vector<vec2> &rmdInterpPoints,
//     const std::vector<vec2> &pathGenPoints,
//     )