#include "utils.h"

double randomSample(){
    // return between 0 and 1
    return static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
}

std::array<double, 2> sampleAnnulus(double rMin, double rMax){
    // random anululs sampling:
    // https://ridlow.wordpress.com/2014/10/22/uniform-random-points-in-disk-annulus-ring-cylinder-and-sphere/
    double rPick = sqrt((rMax*rMax - rMin*rMin)*randomSample() + rMin*rMin);
    double thetaPick = randomSample() * 2 * M_PI;
    std::array<double, 2> outArr;
    outArr[0] = rPick * cos(thetaPick);
    outArr[1] = rPick * sin(thetaPick);
    return outArr;
}

// https://internal.sdss.org/trac/as4/wiki/FPSLayout
Eigen::MatrixXd getHexPositions(int nDia, double pitch){
    // returns a 2d array populated with xy positions
    // for a hex packed grid
    // nDia must be odd (not caught)
    int nHex = 0.25*(3*nDia*nDia + 1);
    int nEdge = 0.5*(nDia + 1);
    Eigen::MatrixXd A(nHex, 2);
    double vertShift = sin(60*M_PI/180.0)*pitch;
    double horizShift = cos(60*M_PI/180.0)*pitch;
    int hexInd = 0;
    // start a xStart such that the center of the
    // hex grid is at 0,0
    double xStart = -1*pitch*(nDia - 1.0)/2.0;
    double nextX = xStart;
    double nextY = 0;

    // first fill in equator
    // 0,0 is center
    for (int ii = 0; ii < nDia; ii++){
        A(hexInd,0) = nextX;
        A(hexInd, 1) = nextY;
        nextX += pitch;
        hexInd++;
    }

    // loop over top and bottom rows
    for (int row = 1; row < nEdge; row++){
        nextY = row * vertShift;
        nextX = xStart + row * horizShift;
        for (int jj = 0; jj < nDia - row; jj++){
            A(hexInd, 0) = nextX;
            A(hexInd, 1) = nextY;
            hexInd++;
            A(hexInd, 0) = nextX;
            A(hexInd, 1) = -1*nextY;
            hexInd++;
            nextX += pitch;
        }
    }

    return A;
}

// create a linear interpolater
double linearInterpolate(std::vector<Eigen::Vector2d> & sparseXYPoints, double xValue){
    Eigen::Vector2d pt1, pt0;
    double yValue;
    int nPoints = sparseXYPoints.size();
    // check that x is in range
    if (xValue < sparseXYPoints[0](0) || xValue > sparseXYPoints[nPoints-1](0)){
        throw std::runtime_error("x outside interpolation range");
    }
    // check if x == last point
    if (xValue == sparseXYPoints[nPoints-1](0)){
        yValue = sparseXYPoints[nPoints-1](1);
    }
    for (int ii = 0; ii < nPoints-1; ii++){
        pt1 = sparseXYPoints[ii+1];
        pt0 = sparseXYPoints[ii];
        if (xValue < pt1(0)){
            yValue = pt0(1) + (pt1(1)-pt0(1)) / (pt1(0) - pt0(0)) * (xValue - pt0(0));
            break;
        }
    }
    return yValue;
}

// http://geomalgorithms.com/a07-_distance.html#dist3D_Segment_to_Segment()
// dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
//    Input:  two 3D line segments S1 and S2
//    Return: the shortest distance between S1 and S2
double dist3D_Segment_to_Segment(
    // return distance squared
    Eigen::Vector3d S1_P0, Eigen::Vector3d S1_P1,
    Eigen::Vector3d S2_P0, Eigen::Vector3d S2_P1)
{
    Eigen::Vector3d   u = S1_P1 - S1_P0;
    Eigen::Vector3d   v = S2_P1 - S2_P0;
    Eigen::Vector3d   w = S1_P0 - S2_P0;
    double    a = u.dot(u);         // always >= 0
    double    b = u.dot(v);
    double    c = v.dot(v);         // always >= 0
    double    d = u.dot(w);
    double    e = v.dot(w);
    double    D = a*c - b*b;        // always >= 0
    double    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    double    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM) { // the lines are almost parallel
        sN = 0.0;         // force using point P0 on segment S1
        sD = 1.0;         // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                 // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d +  b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
    tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

    // get the difference of the two closest points
    Eigen::Vector3d   dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

    return dP.dot(dP);   // return the closest distance squared
}


// Ramer-Douglas-Peucker for segmentizing paths!
// https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
// https://gist.github.com/TimSC/0813573d77734bcb6f2cd2cf6cc7aa51
// std::vector<Eigen::Vector2d>
double PerpendicularDistance(const Eigen::Vector2d &pt, const Eigen::Vector2d &lineStart, const Eigen::Vector2d &lineEnd)
{
    // copied from dude's github,
    // could be made to use eigen stuff for linalg/norms
    double dx = lineEnd(0) - lineStart(0);
    double dy = lineEnd(1) - lineStart(1);


    //Normalise
    double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
    if(mag > 0.0)
    {
        dx /= mag; dy /= mag;
    }

    double pvx = pt(0) - lineStart(0);
    double pvy = pt(1) - lineStart(1);

    //Get dot product (project pv onto normalized direction)
    double pvdot = dx * pvx + dy * pvy;

    //Scale line direction vector
    double dsx = pvdot * dx;
    double dsy = pvdot * dy;

    //Subtract this from pv
    double ax = pvx - dsx;
    double ay = pvy - dsy;

    return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}

void RamerDouglasPeucker(const std::vector<Eigen::Vector2d> &pointList, double epsilon, std::vector<Eigen::Vector2d> &out)
{   // biasDir is to modify the internal (not end) points by epsilon in one directon to make sure we are on one
    // side of the curve
    if(pointList.size()<2)
        throw std::runtime_error("Not enough points to simplify");

    // Find the point with the maximum distance from line between start and end
    double dmax = 0.0;
    size_t index = 0;
    size_t end = pointList.size()-1;
    for(size_t i = 1; i < end; i++)
    {
        double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
        if (d > dmax)
        {
            index = i;
            dmax = d;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if(dmax > epsilon)
    {
        // Recursive call
        std::vector<Eigen::Vector2d> recResults1;
        std::vector<Eigen::Vector2d> recResults2;
        std::vector<Eigen::Vector2d> firstLine(pointList.begin(), pointList.begin()+index+1);
        std::vector<Eigen::Vector2d> lastLine(pointList.begin()+index, pointList.end());
        RamerDouglasPeucker(firstLine, epsilon, recResults1);
        RamerDouglasPeucker(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end()-1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
        if(out.size()<2)
            throw std::runtime_error("Problem assembling output");
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList[0]);
        out.push_back(pointList[end]);
    }
}

// double meanErrorRMD(
//     // return the y value for which to shift points
//     // such that we have only a 1 sided error
//     const std::vector<Eigen::Vector2d> &rmdInterpPoints,
//     const std::vector<Eigen::Vector2d> &pathGenPoints,
//     )
// {
//     // compute mean error
//     int size = rmdInterpPoints.size();
//     double mean = 0
//     for (ii=0; ii<size; ii++){
//         mean += std::abs(rmdInterpPoints[ii] - pathGenPoints[ii]);
//     }
//     mean = mean / size;
//     return mean;
// }



