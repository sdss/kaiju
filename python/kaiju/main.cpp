#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <iostream>
#include <time.h>       /* time */
#include <thread>
#include "robotGrid.h"


RobotGrid doOne(){
    RobotGrid rg (25, maxPathStepsGlob);
    rg.decollide();
    rg.pathGen();
    return rg;
}

void doOneThread(int threadNum){
    int maxIter = 200;
    int seed = threadNum * maxIter;
    char buffer[50];
    for (int ii = 0; ii<maxIter; ii++){
        std::cout << "seed " << seed << std::endl;
        srand(seed);
        RobotGrid rg (25, maxPathStepsGlob);
        rg.decollide();
        rg.pathGen();
        if (!rg.didFail){
            sprintf(buffer, "success_%04d.txt", seed);
            rg.toFile(buffer);
        }
        seed++;
    }

}

// int main(){
//     int maxIter = 1000;
//     char buffer[50];
//     for (int ii = 0; ii<maxIter; ii++){
//         std::cout << "seed " << ii << std::endl;
//         srand(ii);
//         RobotGrid rg (25, maxPathStepsGlob);
//         rg.decollide();
//         rg.pathGen();
//         sprintf(buffer, "end_%04d.txt", ii);
//         rg.toFile(buffer);
//     }
// }

// int main()
// {
//     int nThreads = 10;
//     std::thread t[10];
//     clock_t tStart;
//     clock_t tEnd;
//     tStart = clock();
//     for (int i = 0; i<10; ++i){
//         t[i] = std::thread(doOneThread, i);
//     }
//     for (int i=0; i<10; ++i){
//         t[i].join();
//     }
//     tEnd = clock();
//     std::cout << "time took: " << (double)(tEnd - tStart)/CLOCKS_PER_SEC << std::endl;
// }

// int main()
// {
//     std::cout << "max steps " << maxPathStepsGlob << std::endl;
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

int main()
{
    // single run print out robot paths
    srand(0);
    clock_t tStart;
    clock_t tEnd;
    tStart = clock();
    RobotGrid rg = doOne();
    // rg.smoothPaths();
    tEnd = clock();
    std::cout << "time took: " << (double)(tEnd - tStart)/CLOCKS_PER_SEC << std::endl;
    for (Robot &robot : rg.allRobots){
        robot.pathToFile();
        // std::cout << " 1 " << robot.id << std::endl;
        // robot.smoothPathToFile();
        // std::cout << " 2 " << robot.id << std::endl;
        // robot.ismoothPathToFile();
        // std::cout << " 3 " << robot.id << std::endl;
    }
    // rg.verifySmoothed();
}


