import pytest
import numpy
import time
import pandas

from kaiju.robotGrid import RobotGrid
from kaiju import utils

# nDia = 15
# angStep = 1
# collisionBuffer = 1.8 #2
# epsilon = angStep * 2
hasApogee = True
ang_step = 0.1
smooth_points = 21
collision_shrink = 0.08
epsilon_factor = 2
default_path_generator = "mdp"
hex_size = 35
greed = 0.7
phobia = 0.6
seed = 3
collision_buffer = 2
epsilon = ang_step * epsilon_factor


def test_2d(plot=False):
    initData = pandas.read_csv("data/test_2d_1.csv")
    rg = RobotGrid(ang_step, seed=seed)
    for ii, row in initData.iterrows():
        robotID = int(row.robotID)
        rg.addRobot(
            robotID,
            str(robotID),
            [row.xPos, row.yPos, 0.0],
            hasApogee
        )
        robot = rg.robotDict[robotID]
        robot.setDestinationAlphaBeta(0,180)
        robot.setAlphaBeta(row.alpha, row.beta)

    rg.setCollisionBuffer(collision_buffer)
    rg.initGrid()
    utils.plotOne(0, rg, figname="pathGenInitial.png", isSequence=False)
    assert rg.getNCollisions() == 0
    tstart = time.time()
    rg.pathGenMDP2(greed, phobia)
    print("pathgen took", time.time()-tstart)
    print("deadlocks", rg.deadlockedRobots())
    # import pdb; pdb.set_trace()

    # import pdb; pdb.set_trace()
    # xPos, yPos = utils.hexFromDia(hex_size, pitch=22.4)
    # print("using", len(xPos), "robots")

    # rg = RobotGrid(angStep, seed=seed)
    # for robotID, (x, y) in enumerate(zip(xPos, yPos)):
    #     rg.addRobot(robotID, str(robotID), [x, y, 0], hasApogee)
    #     rg.robotDict[robotID].setDestinationAlphaBeta(0, 180)
    # rg.setCollisionBuffer(collisionBuffer)
    # rg.initGrid()
    # for rID in rg.robotDict:
    #     robot = rg.getRobot(rID)
    #     robot.setXYUniform()
    # assert rg.getNCollisions() > 10
    # if plot:
    #     utils.plotOne(0, rg, figname="pathGenInitial.png", isSequence=False)
    # rg.decollideGrid()

    # robotIDs = numpy.arange(len(xPos))
    # alpha = []
    # beta = []
    # for robotID in robotIDs:
    #     robot = rg.robotDict[robotID]
    #     alpha.append(robot.alpha)
    #     beta.append(robot.beta)

    # df = pandas.DataFrame(
    #     {
    #         "robotID": robotIDs,
    #         "xPos": xPos,
    #         "yPos": yPos,
    #         "alpha": alpha,
    #         "beta": beta
    #     }
    # )
    # df.to_csv("data/test_2d_1.csv", index=False)

    # import pdb; pdb.set_trace()

    # assert rg.getNCollisions() == 0
    # if plot:
    #     utils.plotOne(0, rg, figname="pathGenDecollided.png", isSequence=False)
    # rg.pathGenGreedy()
    # # print("rg alg type", rg.algType, type(rg.algType), str(rg.algType))
    # # sd = rg.robotGridSummaryDict()
    # # for d in sd["robotDict"].values():
    # #     print(d["id"], d["alpha"], d["beta"])
    # # print("deadlocks", rg.deadlockedRobots())
    # assert not rg.didFail
    # rg.smoothPaths(smoothPts)
    # rg.simplifyPaths()
    # # rg.verifySmoothed()
    # # assert rg.smoothCollisions > 100
    # # print(rg.smoothCollisions)
    # rg.shrinkCollisionBuffer(collisionShrink)
    # # rg.verifySmoothed()
    # # assert rg.smoothCollisions == 0
    # print(rg.smoothCollisions)

    # if plot:
    #     utils.plotOne(0, rg, figname="donepaht.png", isSequence=False)
    #     utils.plotPaths(rg, downsample=downsample, filename="pathGen.mp4")


if __name__ == "__main__":
    test_2d(plot=True)


