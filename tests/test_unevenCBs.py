import pytest
import numpy
from kaiju import RobotGrid, RobotGridCalib
from kaiju import utils
import time
numpy.random.seed(0)


def test_unevenCBs(plot=False):
    hasApogee = True
    greed = 0.8
    phobia = 0.2

    xPos, yPos = utils.hexFromDia(17, pitch=22.4)
    seed = 1
    # cb = 2.5
    cs = 0.04
    step = 0.1          # degrees per step in kaiju's rough path
    smoothPts = 10           # width of velocity smoothing window
    eps = step * 2.2
    # downsample = int(numpy.floor(50 / step))
    rg = RobotGrid(
        stepSize=step,
        epsilon=eps, seed=seed
    )

    for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        rg.addRobot(robotID, str(robotID), [x, y, 0], hasApogee, collisionBuffer=numpy.random.uniform(1.5, 3.5))
    rg.initGrid()
    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        robot.setXYUniform()
    assert rg.getNCollisions() > 10

    rg.decollideGrid()

    for robot in rg.robotDict.values():
        robot.setDestinationAlphaBeta(0, 180)
    assert rg.getNCollisions() == 0
    tstart = time.time()
    rg.pathGenMDP(greed, phobia)
    print("pathgen took", time.time()-tstart)
    rg.smoothPaths(smoothPts)
    rg.simplifyPaths()
    rg.shrinkCollisionBuffer(cs)
    rg.verifySmoothed()


    assert rg.smoothCollisions == 0
    print("n smooth collisions", rg.smoothCollisions)

    if plot:
        for r in rg.robotDict.values():
            utils.plotTraj(r, "unevenCBs", dpi=250)
        utils.plotPaths(rg, downsample=3, filename="unevenCBs.mp4")


if __name__ == "__main__":
    test_unevenCBs(plot=True)
