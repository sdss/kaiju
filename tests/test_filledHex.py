from kaiju.utils import robotGridFromFilledHex, plotPaths, plotOne

def test_filledHex():
    stepSize = 1
    collisionBuffer = 0.1
    rg = robotGridFromFilledHex(stepSize, collisionBuffer)
    rg.decollide()
    rg.pathGen()
    plotPaths(rg)

def test_initialPlacements():

    for i in range(1):
        stepSize = 1
        collisionBuffer = 0.1
        rg = robotGridFromFilledHex(stepSize, collisionBuffer, i)
        rg.decollide()
        rg.pathGen()
        plotOne(0, rg, "fig_%i.png"%i)

test_initialPlacements()