from kaiju.utils import robotGridFromFilledHex
import numpy
import numpy.random
import matplotlib.pyplot as plt


nTargs = 100000

def test_apTargetList(fiberID=1):
    stepSize = 1
    collisionBuffer = 0.1
    rg = robotGridFromFilledHex(stepSize, collisionBuffer, 0)
    # targetList must not be numpy types...for now
    targetList = []
    for tID in range(nTargs):
        x = numpy.random.random()*700-350
        y = numpy.random.random()*700-350
        targetList.append([tID, x, y, 1, fiberID])
    rg.setTargetList(targetList)
    fig = plt.figure(figsize=(10,10))
    goodX = []
    goodY = []
    badX = []
    badY = []
    for targ in rg.targetList:
        if len(targ.validRobots) == 0:
            badX.append(targ.x)
            badY.append(targ.y)
        else:
            goodX.append(targ.x)
            goodY.append(targ.y)

    plt.plot(goodX, goodY, ".", color="green", markersize=5, alpha=0.5)
    plt.plot(badX, badY, ".", color="orange", markersize=5, alpha=0.5)
    if fiberID == 1:
        plt.title("APOGEE TARGETS")
    else:
        plt.title("BOSS TARGETS")
    plt.savefig("fig_%i.png"%fiberID)

test_apTargetList(1)
test_apTargetList(2)