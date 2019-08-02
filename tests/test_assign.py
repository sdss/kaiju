import numpy

from kaiju.utils import robotGridFromFilledHex, plotOne

numpy.random.seed(0)
nTargs = 10000

def getRandomTargetList(nTargs):
    # return a random targetlist picked from a uniform
    # disk inscribing the hex array
    # many targets will fall outsize the hex
    # design
    # columns are: id, x, y, priority, fiberID
    # priority is between 0-4 (any integer is fine)
    # fiberID is 1 or 2 corresponding to apogee or boss respectively

    # radius containing the grid
    bottomLeftRobot = [-145.6000, -252.1866]
    gridRadius = numpy.linalg.norm(bottomLeftRobot) + 22.4

    ## generate a uniform sample in xy on the disk
    r = numpy.sqrt(numpy.random.random(nTargs))*gridRadius
    theta = numpy.random.random(nTargs)*2*numpy.pi
    x = r * numpy.cos(theta)
    y = r * numpy.sin(theta)
    ids = numpy.arange(nTargs)
    # generate random priorities for targets (0-4), 4 being high priority
    priorities = numpy.random.choice(range(5), nTargs)
    # generate random spectrograph assignments 1=apogee 2=boss
    fiberIDs = numpy.random.choice([1,2], nTargs)

    # create the numpy target list to feed into the robotGrid
    # an n x 5 array
    targetList = numpy.vstack((ids, x, y, priorities, fiberIDs)).T
    return targetList

targetList = getRandomTargetList(nTargs)
# get the robot grid and give it the target list
stepSize = 1
collisionBuffer = 0.1
seed = 0
rg = robotGridFromFilledHex(stepSize, collisionBuffer, seed)
rg.setTargetList(targetList)

# see if any robots do not have target options
targetlessRobots = rg.targetlessRobots()
print("%i robots have no viable targets"%len(targetlessRobots))

# determine the targets that will never be reached
# due to fiducial collisions or out of range
unreachableTargets = rg.unreachableTargets()
print("%i targets are unreachable"%len(unreachableTargets))


badIDs = [t.id for t in unreachableTargets]
# maybe a better way to avoid loop here?
filterInds = []
for ind, row in enumerate(targetList):
    if int(row[0]) in badIDs: # casting to int makes things go faster?
        filterInds.append(False) # don't keep
    else:
        filterInds.append(True) # keep

# delete the unreachable targets from the targetList
reachableTargetList = targetList[filterInds]
print("%i reachable targets"%len(reachableTargetList))

# reset the grid to contain only reachable targets
rg.setTargetList(reachableTargetList)

# get robot->taret mapping.  Robots are available by their index
# in rg.allRobots
robot2targ = []
for robot in rg.allRobots:
    # be carefull when iterating over allRobots, you're getting
    # a copy.  IF you want to modify a robot use getRobot(robotInd)
    targIDList = [t.id for t in robot.targetList]
    robot2targ.append(targIDList)

robotInd = 5

print("robot %i avalable target ids %s"%(robotInd, str(robot2targ[robotInd])))

# get target-> robot mapping
target2robot = {}
for target in rg.targetList:
    target2robot[target.id] = [r.id for r in target.validRobots]

# look at a random targetID see what robots it can go to
targetID = reachableTargetList[6,0]

print("targetID %i valid robot inds %s"%(targetID, str(target2robot[targetID])))

# ask kaiju to pick high priority targets from the list
print("greedy assigning targets from targetList")
rg.greedyAssign()
print("after greedy assign, %i robots are collided"%(rg.getNCollisions()))
# plot the collided array
plotOne(0, robotGrid=rg, figname="greedyAssign.png", isSequence=False)

# find first collided robot and reassign it until it doesn't collide
for robotInd, robot in enumerate(rg.allRobots):
    if robot.isCollided():
        reassignRobot = robotInd
        break

# get available target ids for collided robot
availableTargIDs = [t.id for t in rg.allRobots[reassignRobot].targetList]
for targID in availableTargIDs:
    try:
        rg.assignRobot2Target(reassignRobot, targID)
    except:
        print("targID %i not valid (probably taken by another robot)"%targID)
    robot = rg.allRobots[reassignRobot]
    if not robot.isCollided():
        print("assigning targetID %i to robot %i resolved the collision"%(targID, reassignRobot))
        break
# plot the grid again
plotOne(0, robotGrid=rg, figname="byHandReplace.png", isSequence=False)

# ask kaiju to try pairwise swapping to reduce collisions in grid
# but keeping all assigned targets
print("collisions before pairwise swap: %i"%(rg.getNCollisions()))
rg.pairwiseSwap()
print("collisions after pairwise swap: %i"%(rg.getNCollisions()))
# plot the result of the pairwise swap
plotOne(0, robotGrid=rg, figname="pairwiseSwap.png", isSequence=False)

# finally ask kaiju to throw away targets until the grid is decollided
# lowest priority targets will be thrown out first
rg.decollide()
#plot the result of the decollision
plotOne(0, robotGrid=rg, figname="decollide.png", isSequence=False)

# collect the acquired target ids
# and unassigned robots
assignedTargets = rg.assignedTargets()
unassignedRobots = rg.unassignedRobots()

print("got %i targets" % len(assignedTargets))
print("%i robots thrown away" % len(unassignedRobots))

