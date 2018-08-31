// Copyright 2001 softSurfer, 2012 Dan Sunday
// This code may be freely used, distributed and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.


// Assume that classes are already given for the objects:
//    Point and Vector with
//        coordinates {float x, y, z;}
//        operators for:
//            Point   = Point ± Vector
//            Vector =  Point - Point
//            Vector =  Vector ± Vector
//            Vector =  Scalar * Vector
//    Line and Segment with defining points {Point  P0, P1;}
//    Track with initial position and velocity vector
//            {Point P0;  Vector v;}
//===================================================================

#include <iostream>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <cmath>

#define SMALL_NUM   0.00000001 // anything that avoids division overflow
// dot product (3D) which allows vector operations in arguments
#define dot(u,v)   ((u).x * (v).x + (u).y * (v).y + (u).z * (v).z)
#define norm(v)    sqrt(dot(v,v))  // norm = length of  vector
#define d(u,v)     norm(u-v)        // distance = norm of difference
#define abs(x)     ((x) >= 0 ? (x) : -(x))   //  absolute value

class Point;

class Vector {
public:
    float x, y, z;
    Vector();
    Vector(float, float, float);
    Vector operator+(const Vector&);
    Vector operator-(const Vector&);
    Vector operator*(float);
};

Vector::Vector() : x(0), y(0), z(0) {}

Vector::Vector(float xx, float yy, float zz){
    x = xx;
    y = yy;
    z = zz;
}

Vector Vector::operator+(const Vector& v){
    return Vector(x+v.x, y+v.y, z+v.z);
}

Vector Vector::operator-(const Vector& v){
    return Vector(x-v.x, y-v.y, z-v.z);
}


Vector Vector::operator*(float scalar){
    return Vector(x*scalar, y*scalar, z*scalar);
}


class Point {
public:
    float x,y,z;
    Point();
    Point(float, float, float);
    Vector operator-(const Point&);
    Point operator+(const Vector&);
    Point operator-(const Vector&);
};

Point::Point() : x(0), y(0), z(0){}

Point::Point(float xx, float yy, float zz){
    x = xx;
    y = yy;
    z = zz;
}

Vector Point::operator-(const Point& p){
    return Vector(x-p.x, y-p.y, z-p.z);
}

Point Point::operator+(const Vector& v){
    return Point(x+v.x, y+v.y, z+v.z);
}

Point Point::operator-(const Vector& v){
    return Point(x-v.x, y-v.y, z-v.z);
}


class Segment{
public:
    Point P0, P1;
    Segment();
    Segment(Point, Point);
};

Segment::Segment(){
    Point P0;
    Point P1;
}

Segment::Segment(Point pp0, Point pp1){
    P0 = pp0;
    P1 = pp1;
}

// dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
//    Input:  two 3D line segments S1 and S2
//    Return: the shortest distance between S1 and S2
float
dist3D_Segment_to_Segment( Segment S1, Segment S2)
{
    Vector   u = S1.P1 - S1.P0;
    Vector   v = S2.P1 - S2.P0;
    Vector   w = S1.P0 - S2.P0;
    float    a = dot(u,u);         // always >= 0
    float    b = dot(u,v);
    float    c = dot(v,v);         // always >= 0
    float    d = dot(u,w);
    float    e = dot(v,w);
    float    D = a*c - b*b;        // always >= 0
    float    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    float    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

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
    // Vector   dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)
    Vector   dP = w + (u * sc) - (v * tc);  // =  S1(sc) - S2(tc)

    return norm(dP);   // return the closest distance
}

int main(){
    Point P1 = Point(1,2,0);
    Point P2 = Point(5,2,0);
    Point P3 = Point(0,0,0);
    Point P4 = Point(-4,8,0);
    Segment Seg1 = Segment(P1, P2);
    Segment Seg2 = Segment(P3, P4);
    std::cout << "dist between segs " << dist3D_Segment_to_Segment(Seg1, Seg2) << std::endl;
}
