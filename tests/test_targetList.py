from kaiju.utils import robotGridFromFilledHex, plotOne
import numpy
import numpy.random
import matplotlib.pyplot as plt


numpy.random.seed(0)

stepSize = 1
collisionBuffer = 0.1
radius = numpy.linalg.norm([-145.6000, -252.1866]) + 22.4
minSep = 3 #mm

def randomXY():
    #  sample disk
    r = numpy.sqrt(numpy.random.random())*radius
    # print("r", r)
    theta = numpy.random.random()*2*numpy.pi
    x = r*numpy.cos(theta)
    y = r*numpy.sin(theta)
    return x,y

def generateTargetList(nTargs):
    targetList = []
    for tID in range(nTargs):
        x, y = randomXY()
        # enforce a minimum separation
        isOK = True
        for existTarg in targetList:
            if numpy.linalg.norm([x-existTarg[1], y-existTarg[2]]) < minSep:
                isOK = False
                break
        if not isOK:
            continue
        priority = numpy.random.choice(range(5))
        fiberID = numpy.random.choice([1,2])
        targetList.append([tID, x, y, priority, fiberID])
    print("len target list", len(targetList))
    return targetList

def test_apTargetList(fiberID=1):
    # nTargs = 1000000
    nTargs = 600
    rg = robotGridFromFilledHex(stepSize, collisionBuffer, 0)
    # targetList must not be numpy types...for now
    targetList = []
    for tID in range(nTargs):
        x = numpy.random.random()*700-350
        y = numpy.random.random()*700-350
        priority = numpy.random.choice(range(5))
        targetList.append([tID, x, y, priority, fiberID])
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

    plt.plot(goodX, goodY, ".", color="green", markersize=5, alpha=0.9)
    plt.plot(badX, badY, ".", color="orange", markersize=5, alpha=0.9)
    if fiberID == 1:
        plt.title("APOGEE TARGETS")
    else:
        plt.title("BOSS TARGETS")
    plt.savefig("fig_%i.png"%fiberID)

def test_assignability():

    targOps = [500, 500, 600, 700, 800, 900, 1000, 2000, 3000, 4000]
    rg = robotGridFromFilledHex(stepSize, collisionBuffer, 0)

    for nTargs in targOps:
        targetList = generateTargetList(nTargs)
        rg.setTargetList(targetList)
        print("badRobos %i badTargs %i"%(len(rg.targetlessRobots()), len(rg.unreachableTargets())))


def test_one():
    nTargs = 3000
    rg = robotGridFromFilledHex(stepSize, collisionBuffer, 0)
    targetList = generateTargetList(nTargs)
    rg.setTargetList(targetList)
    rg.greedyAssign()
    print("collisions before pair swap", rg.getNCollisions())
    plotOne(0, robotGrid=rg, figname="n_1.png", isSequence=False)
    rg.pairwiseSwap()
    print("collisions after pair swap", rg.getNCollisions())
    plotOne(0, robotGrid=rg, figname="n_2.png", isSequence=False)
    rg.decollide()
    plotOne(0, robotGrid=rg, figname="n_3.png", isSequence=False)
    ats = rg.assignedTargets()
    targIDs = [at.id for at in ats]
    print("targetless robots", len(rg.targetlessRobots()))
    for r in rg.targetlessRobots():
        print("id, x, y, isassigned", r.id, r.xPos, r.yPos, r.isAssigned())


# test_apTargetList(1)
# test_apTargetList(2)
test_one()
