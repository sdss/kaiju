#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <iostream>
#include <time.h>       /* time */
#include <thread>
#include "robotGrid.h"
#include "utils.h"



void doOne(){
    double angStep = 0.01;
    double colBuff = 0;
    double eps = 2;
    int seed = 0;
    bool hasApogee = true;
    RobotGrid rg(angStep, colBuff, eps, seed);
    auto hexPos = getHexPositions(51, 22.4);
    // std::cout << "hex " << hexPos.rowwise() << std::endl;
    for (int ii=0; ii<hexPos.rows(); ii++){
        rg.addRobot(ii, hexPos(ii,0), hexPos(ii,1), hasApogee);
    }
    rg.initGrid();
}


int main(){
    int maxIter = 2000;
    int ii = 0;
    while(ii < maxIter){
        doOne();
        ii++;
    }
    return 0;
}


