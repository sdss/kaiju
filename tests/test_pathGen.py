from kaiju import RobotGrid
from kaiju.utils import plotPaths, hexFromDia

def test_pathGen():
    stepSize = 1 # degrees
    nDia = 25
    # r_phys = 2
    b_step = 0.02
    b_smooth = 0.04
    epsilon = stepSize * 2.2
    collisionBuffer = 0.1

    rg = RobotGrid(stepSize, collisionBuffer, epsilon, 0)
    xPos, yPos = hexFromDia(nDia, pitch=22.4)
    for ii, (xp,yp) in enumerate(zip(xPos,yPos)):
        rg.addRobot(ii, xp, yp)
    rg.initGrid()
    nFails = 0
    for i in range(100):
        print("iter", i)
        for ii in range(rg.nRobots):
            r = rg.getRobot(ii)
            r.setXYUniform()
        rg.decollide()
        rg.pathGen()
        if rg.didFail:
            plotPaths(rg)
            break
            nFails += 1

    print("nFails", nFails)
    assert nFails == 0


    # rg = RobotGrid(nDia, stepSize, collisionBuffer, epsilon, 0)
    # rg.decollide()
    # rg.pathGen()
    # rg.smoothPaths()
    # rg.verifySmoothed()
    # plotPaths(rg)
    # print("interp collisions", rg.smoothCollisions)


test_pathGen()
