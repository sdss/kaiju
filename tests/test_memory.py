
from kaiju.robotGrid import RobotGridNominal
from kaiju import utils
import psutil
import os
import time


nDia = 15
angStep = 1
collisionBuffer = 2
epsilon = angStep * 2
hasApogee = True
GBperByte = 1e-9 / 5

nGrids = int(300 / 5)

def test_memory():
    # pick a big grid, run a bunch of grids
    # watch memory...
    # xPos, yPos = utils.hexFromDia(19, pitch=22.4)
    for seed in range(nGrids):
        rg = RobotGridNominal(angStep, seed)
        rg.setCollisionBuffer(collisionBuffer)
        # for robotID, (x, y) in enumerate(zip(xPos, yPos)):
        #     rg.addRobot(robotID, str(robotID), [x, y,  hasApogee)
        #     # rg.robotDict[robotID].setTargetAlphaBeta(0, 180)
        # rg.initGrid()

        for r in rg.robotDict.values():
            r.setXYUniform()
            r.setDestinationAlphaBeta(10,170)
        rg.decollideGrid()
        rg.pathGenGreedy()
    process = psutil.Process(os.getpid())
    GBUsed = process.memory_info().rss * GBperByte
    # print("GBUSED", GBUsed)
    assert GBUsed < 0.3


if __name__ == "__main__":
    tstart = time.time()
    test_memory()
    print("took %.2f mins"%((time.time()-tstart)/60.))