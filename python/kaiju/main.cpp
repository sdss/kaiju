#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <iostream>
#include <time.h>       /* time */
#include <thread>
#include "robotGrid.h"
#include "betaArm.h"


RobotGrid doOne(){
    RobotGrid rg (25, 1);
    // std::cout << "ncollisions before swaps " << rg.getNCollisions() << std::endl;
    rg.optimizeTargets();
    // std::cout << "ncollisions after swaps " << rg.getNCollisions() << std::endl;
    rg.decollide();
    rg.pathGen();
    return rg;
}

void doOneThread(int threadNum){
    int ang_step = 1;
    int maxIter = 400;
    int seed = threadNum * maxIter;
    char buffer[50];
    for (int ii = 0; ii<maxIter; ii++){
        // std::cout << "seed " << seed << std::endl;
        srand(seed);
        RobotGrid rg (25, ang_step);
        rg.optimizeTargets();
        rg.decollide();
        rg.pathGen();

        // if (!rg.didFail){
        //     sprintf(buffer, "success_%04d.txt", seed);
        //     rg.toFile(buffer);
        // }
        sprintf(buffer, "stats_%04d_%02d.txt", seed, betaGeomID);
        rg.printStats(buffer);
        seed++;
    }

}

// int main(){
//     initBetaArms();
//     int maxIter = 1000;
//     char buffer[50];
//     for (int ii = 0; ii<maxIter; ii++){
//         std::cout << "seed " << ii << std::endl;
//         srand(ii);
//         RobotGrid rg (25, 1,);
//         rg.decollide();
//         rg.pathGen();
//         sprintf(buffer, "end_%04d.txt", ii);
//         rg.toFile(buffer);
//     }
// }

// int main() // try geometry stats
// {
//     initBetaArms();
//     int geom_id;
//     int nThreads = 10;
//     std::thread t[10];
//     for (geom_id=0; geom_id<9; geom_id++){
//         setBetaGeom(geom_id);
//         for (int i = 0; i<nThreads; ++i){
//             t[i] = std::thread(doOneThread, i);
//         }
//         for (int i=0; i<nThreads; ++i){
//             t[i].join();
//         }
//     }
// }

// int main()
// {
//     initBetaArms();
//     int nThreads = 10;
//     std::thread t[10];
//     clock_t tStart;
//     clock_t tEnd;
//     tStart = clock();
//     for (int i = 0; i<nThreads; ++i){
//         t[i] = std::thread(doOneThread, i);
//     }
//     for (int i=0; i<nThreads; ++i){
//         t[i].join();
//     }
//     tEnd = clock();
//     std::cout << "time took: " << (double)(tEnd - tStart)/CLOCKS_PER_SEC << std::endl;
// }

// int main()
// {
//     initBetaArms();
//     std::cout << "max steps " << 1, << std::endl;
//     // run 500, print out failed grids
//     int nFails = 0;
//     int maxTries = 500;
//     char buffer[50];
//     for (int ii=0; ii<maxTries; ii++){
//         srand(ii);
//         std::cout << "trial " << ii << std::endl;
//         RobotGrid rg = doOne();
//         if(rg.didFail){
//             sprintf(buffer, "fail_%d.txt", ii);
//             rg.toFile(buffer);
//             nFails++;
//         }
//     }
//     std::cout << "nFails " << nFails << std::endl;

// }

int main(int argc,char *argv[])
{
    double collisionBuffer = 0;
    double ang_step = 1;
    if( argc > 1 ) {
        collisionBuffer = atof(argv[1]);  // alternative strtod
    }
    if( argc > 2 ) {
        ang_step = atof(argv[2]);  // alternative strtod
    }
    // single run print out robot paths
    initBetaArms();
    srand(0);
    int print_every =  (int)(1.0 / ang_step);
    std::cout << "print every: " << print_every << std::endl;
    clock_t tStart;
    clock_t tEnd;
    tStart = clock();
    RobotGrid rg (25, ang_step, print_every, collisionBuffer);
    std::cout << "ncollisions before swaps " << rg.getNCollisions() << std::endl;
    rg.optimizeTargets();
    std::cout << "ncollisions after swaps " << rg.getNCollisions() << std::endl;
    rg.decollide();
    rg.pathGen();
    std::cout << "pathGen fail " << rg.didFail << std::endl;
    rg.setCollisionBuffer(0);
    rg.smoothPaths();
    tEnd = clock();
    std::cout << "time took: " << (double)(tEnd - tStart)/CLOCKS_PER_SEC << std::endl;
    for (Robot &robot : rg.allRobots){
        robot.pathToFile();
        // std::cout << " 1 " << robot.id << std::endl;
        robot.smoothPathToFile();
        // std::cout << " 2 " << robot.id << std::endl;
        robot.ismoothPathToFile();
        // std::cout << " 3 " << robot.id << std::endl;
    }
    rg.verifySmoothed();
}

