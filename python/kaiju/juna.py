import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import PyGuide
import numpy
from astropy.io import fits
import cKaiju
from shapely.geometry import LineString
from descartes import PolygonPatch
from multiprocessing import Pool
import functools

starPng = "/Users/csayres/Downloads/sdssColor.png"
img = plt.imread(starPng)

imgSize = 600
imgStartX = 450
imgStartY = 450
img = img[imgStartX:imgSize+imgStartX, imgStartY:imgSize+imgStartY,:]

plt.figure(figsize=(10,10))
plt.imshow(img)
plt.xlim([0,imgSize])
plt.ylim([0, imgSize])
plt.axis('off')
plt.savefig("color.png")

# import pdb; pdb.set_trace()

gray = img[:,:,2]

gray = numpy.sum(img, axis=2)
# plt.figure()
# plt.hist(gray.flatten())
# plt.show()

# import pdb; pdb.set_trace()

CCDInfo = PyGuide.CCDInfo(
    bias = 1,    # image bias, in ADU
    readNoise = 1, # read noise, in e-
    ccdGain = 2,  # inverse ccd gain, in e-/ADU
)

centroids, stats = PyGuide.findStars(gray, None, None, CCDInfo)
# import pdb; pdb.set_trace()
centroids = centroids[10:-10]
for centroid in centroids:
    centroid.xyCtr[0] = centroid.xyCtr[0] - 0.5
    centroid.xyCtr[1] = centroid.xyCtr[1] - 0.5
    centroid.got = False

plt.figure(figsize=(10,10))
plt.xlim([0,imgSize])
plt.ylim([0, imgSize])
plt.axis('off')
plt.imshow(gray, cmap='gray_r', vmax=1.75)
plt.savefig("gray.png")

for cent in centroids:
    x,y = cent.xyCtr
    plt.scatter(x,y,edgecolors='red', facecolors='red', marker="+", linewidths=2)
plt.savefig("gray_targ.png")

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
r_phys = 2
collisionBuffer = 1.5
offset = imgSize / 2.

cKaiju.initBetaArms()
rg = cKaiju.RobotGrid(nDia, stepSize, betaGeom, printEvery, collisionBuffer, epsilon, 0)
print("setting alpha beta")

# nRobots = len(rg.allRobots)

# for ii in range(rg.nRobots):
#     #
#     # robot.setAlphaBeta(180,180)
#     robot = rg.getRobot(ii)
#     print("pybetaBefore", robot.betaOrientation)
#     robot.setAlphaBetaRand();
#     print("pybetaAfter", robot.betaOrientation)
#     before = robot.betaOrientation
#     # print()
#     # print()
#     # print()

# print("trying again?")
# for ii in range(rg.nRobots):
#     robot = rg.getRobot(ii)
#     print("zizz ", robot.betaOrientation)

for ii in range(rg.nRobots):
    robot = rg.getRobot(ii)
    for cent in centroids:
        if cent.got:
            continue
        cx, cy = cent.xyCtr
        xGlobal = cx - offset
        yGlobal = cy - offset
        # print("got here")
        check = robot.checkFiberXYGlobal(xGlobal, yGlobal)
        # print('check', check)
        if check:
            robot.setFiberXY(xGlobal, yGlobal)
            if not robot.isCollided():
                cent.got = True
                break
            robot.decollide()

print("loop done")
rg.decollide()
rg.pathGen()
print('path gen done')


# gross way to hold things
robotList = []
for ii in range(rg.nRobots):
    robot = rg.getRobot(ii)
    # only keep positions (not time), and downsample
    alphaX = numpy.asarray(robot.roughAlphaX)[:,1]
    alphaY = numpy.asarray(robot.roughAlphaY)[:,1]
    betaX = numpy.asarray(robot.roughBetaX)[:,1]
    betaY = numpy.asarray(robot.roughBetaY)[:,1]

    alphaXds = numpy.hstack((alphaX[0:-1:50], alphaX[-1]))
    alphaYds = numpy.hstack((alphaY[0:-1:50], alphaY[-1]))
    betaXds = numpy.hstack((betaX[0:-1:50], betaX[-1]))
    betaYds = numpy.hstack((betaY[0:-1:50], betaY[-1]))
    robotList.append([alphaXds, alphaYds, betaXds, betaYds, robot.xPos, robot.yPos])


def plotStep(step):
    plt.figure(figsize=(10,10))
    plt.xlim([0, imgSize])
    plt.ylim([0, imgSize])
    plt.axis('off')
    plt.imshow(gray, cmap='gray_r', vmax=1.75)
    ax = plt.gca()

    for r in robotList:
        alphaX = r[0][step] + offset
        alphaY = r[1][step] + offset
        betaX = r[2][step] + offset
        betaY = r[3][step] + offset
        x = r[4] + offset
        y = r[5] + offset
        plt.plot([x, alphaX], [y, alphaY], color='black', linewidth=2)

        topCollideLine = LineString(
            [(alphaX, alphaY), (betaX, betaY)]
        ).buffer(collisionBuffer, cap_style=1)
        topcolor = 'blue'
        patch = PolygonPatch(topCollideLine, fc=topcolor, ec=topcolor, alpha=0.5, zorder=10)
        ax.add_patch(patch)

    for cent in centroids:
        x,y = cent.xyCtr
        if step == 0 and cent.got:
            color = 'green'
            marker = 'o'
            facecolor = "none"
        else:
            color = "red"
            facecolor = "red"
            marker = '+'
        plt.scatter(x,y,edgecolors=color, facecolors=facecolor, marker=marker, linewidths=2)


    plt.savefig("step_%04d.png"%step)
    plt.close()

stepArray = range(len(robotList[0][0]))
# pltPartial = functools.partial(plotStep, robotList)
p = Pool(12)
p.map(plotStep, stepArray)
p.close()
# plt.show()
# ffmpeg -r 10 -f image2 -i step_%04d.png -pix_fmt yuv420p robotMovie.mp4

