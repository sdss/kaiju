from kaiju.cKaiju import RobotGrid
from kaiju import utils
import psutil
import os


nDia = 15
angStep = 1
collisionBuffer = 2
epsilon = angStep * 2
hasApogee = True
GBperByte = 1e-9

def test_memory():
    # pick a big grid, run a bunch of grids
    # watch memory...
    xPos, yPos = utils.hexFromDia(19, pitch=22.4)
    for seed in range(300):
        rg = RobotGrid(angStep, collisionBuffer, epsilon, seed)
        for robotID, (x, y) in enumerate(zip(xPos, yPos)):
            rg.addRobot(robotID, x, y, hasApogee)
            # rg.robotDict[robotID].setTargetAlphaBeta(0, 180)
        rg.initGrid()

        for r in rg.robotDict.values():
            r.setXYUniform()
            r.setDestinationAlphaBeta(10,170)
        rg.decollideGrid()
        rg.pathGenGreedy()
    process = psutil.Process(os.getpid())
    GBUsed = process.memory_info().rss * GBperByte
    assert GBUsed < 0.2


if __name__ == "__main__":
    test_memory()
