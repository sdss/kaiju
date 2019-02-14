# import matplotlib.pyplot as plt
import numpy
import cKaiju
import time
from multiprocessing import Pool
import cPickle
global r_phys

# from interface docs
# speed is 5rpm at output of arm
# output acceleration 10

betaGeom = 9 # 1mm widefin!
printEvery = 0
stepSize = 0.03 # degrees
nDia = 25
# r_phys = 2
b_step = 0.02
b_smooth = 0.04
epsilon = stepSize * 2.2

# max speed = 30 deg per sec
# dt / step = stepSize /
# 0.03 degrees / dt = 30
deg_per_sec = 30
sec_per_step = stepSize / deg_per_sec

def doOne(seed):
    global r_phys
    collisionBuffer = r_phys + b_step + b_smooth
    t0 = time.time()
    rg = cKaiju.RobotGrid(nDia, stepSize, betaGeom, printEvery, collisionBuffer, epsilon, seed)
    rg.decollide()
    rg.pathGen()

    rg.setCollisionBuffer(collisionBuffer - b_smooth)
    rg.smoothPaths()
    rg.verifySmoothed()
    t_run = time.time() - t0

    # stats .....
    maxRDPSteps = 0
    # get number of deadlocked robots
    nDeadlocked = 0
    nReplaced = 0 # decollided
    nReplacements = 0
    maxAccel = 0

    # get max steps sent to robot
    lastAlphas = []
    for robot in rg.allRobots:
        smoothAlphaPath = numpy.asarray(robot.smoothAlphaPath)
        smoothBetaPath = numpy.asarray(robot.smoothBetaPath)
        alphaPts = len(smoothAlphaPath)
        betaPts = len(smoothBetaPath)
        m = numpy.max([alphaPts, betaPts])
        if m > maxRDPSteps:
            maxRDPSteps = m
        lastBeta = smoothBetaPath[-1,-1]
        lastAlpha = smoothAlphaPath[-1,-1]
        lastAlphas.append(lastAlpha)
        if not (lastBeta == 180):
            # print("len alpha path beta path", len(robot.alphaPath), len(robot.betaPath))
            # print("deadlock", ii, lastAlpha, lastBeta)
            nDeadlocked += 1
        if robot.nDecollide > 0:
            nReplaced += 1
            nReplacements += robot.nDecollide


    #     alphaDiff = numpy.diff(smoothAlphaPath, axis=0)
    #     betaDiff = numpy.diff(smoothBetaPath, axis=0)
    #     alphaVel = alphaDiff[:, 1] / (alphaDiff[:, 0] * sec_per_step)
    #     betaVel = betaDiff[:, 1] / (betaDiff[:, 0] * sec_per_step)

    #     alphaAcc = numpy.diff(alphaVel) #/ sec_per_step
    #     betaAcc = numpy.diff(betaVel) #/  sec_per_step

    #     print("shapes", alphaAcc.shape, betaAcc.shape)
    #     ma = -1
    #     mb = -1
    #     if len(alphaAcc) > 0:
    #         ma = numpy.max(numpy.abs(alphaAcc))
    #         if ma > 29.5:
    #             ma = -1
    #     if len(betaAcc) > 0:
    #         mb = numpy.max(numpy.abs(betaAcc))
    #         if mb > 29.5:
    #             mb = -1
    #     _maxAccel = numpy.max([ma, mb])

    #     if _maxAccel > maxAccel:
    #         maxAccel = _maxAccel

    # print ("max accel", ma)


    return [t_run, maxRDPSteps, rg.smoothCollisions, rg.nSteps, nDeadlocked, nReplaced, nReplacements, rg.didFail, lastAlphas]


if __name__ == "__main__":
    global r_phys
    cKaiju.initBetaArms()
    seeds = range(500)
    rPhysList = [
        2.5,
        2.4,
        2.3,
        2.2,
        2.1,
        2.0,
        1.9,
        1.8,
        1.7,
        1.6,
        1.5,
        1.4,
        1.3,
        1.2,
        1.1,
        1.0,
        0.9,
        0.8,
        0.7,
        0.6,
        0.5,
        0.4,
        0.3,
        0.2,
        0.1,
        0.0
    ]
    for r in rPhysList:
        r_phys = r
        p = Pool(12)
        t = time.time()
        outList = p.map(doOne, seeds)
        print("r_phys", r_phys, "took ", time.time()-t)
        outFile = open('r_phys_%.2f.pkl'%r_phys, 'wb')
        cPickle.dump(outList, outFile)
        outFile.close()
        # p.join()
        p.close()


    # betaGeom = 9 # 1mm widefin!
    # printEvery = 0
    # stepSize = 0.03
    # nDia = 25
    # r_phys = 2
    # b_step = 0.02
    # b_smooth = 0.02
    # epsilon = stepSize * 2.2
    # collisionBuffer = r_phys + b_step + b_smooth
    # seed = 0
    # # epsilon = 5 * stepSize
    # rg = cKaiju.RobotGrid(nDia, stepSize, betaGeom, printEvery, collisionBuffer, epsilon, seed)
    # # rg.optimizeTargets()
    # rg.decollide()
    # t0 = time.time()
    # rg.pathGen()
    # print("path gen time: %.2f"%(time.time()-t0))
    # rg.setCollisionBuffer(collisionBuffer - b_smooth)
    # rg.smoothPaths()
    # t0 = time.time()
    # rg.verifySmoothed()
    # print("smooth verification: %.2f"%(time.time()-t0))
    # smoothPts = []
    # roboInd = []
    # for ii, robot in enumerate(rg.allRobots):
    #     alphaPts = len(robot.smoothAlphaPath)
    #     betaPts = len(robot.smoothBetaPath)
    #     maxSteps = numpy.max([alphaPts, betaPts])
    #     if maxSteps > 4:
    #         smoothPts.append(maxSteps)
    #     if maxSteps > 200:
    #         plt.figure()
    #         # print("number of smooth points: %i"%len(rg.))
    #         sAlpha = numpy.asarray(robot.smoothAlphaPath)
    #         sBeta = numpy.asarray(robot.smoothBetaPath)
    #         plt.plot(sAlpha[:,0], sAlpha[:,1], "-o")
    #         plt.plot(sBeta[:,0], sBeta[:,1], "-o")

    # plt.figure()
    # plt.hist(smoothPts, bins=1000)

    # # for ii in roboInd:
    # #     plt.figure()
    # #     # print("number of smooth points: %i"%len(rg.))
    # #     g = rg.allRobots[ii]
    # #     print g
    # #     steps = [x[0] for x in g.smoothAlphaPath]
    # #     alpha = [x[1] for x in g.smoothAlphaPath]
    # #     plt.plot(steps, alpha)

    # #     steps = [x[0] for x in g.smoothBetaPath]
    # #     beta = [x[1] for x in g.smoothBetaPath]
    # #     plt.plot(steps, beta)


    # plt.show()
    # # import pdb; pdb.set_trace()

