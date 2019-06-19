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
img = img[imgStartX:imgSize+imgStartX-1, imgStartY:imgSize+imgStartY-1,:]

img = img*2


plt.figure(figsize=(10,10))
plt.imshow(img)
plt.xlim([0,imgSize])
plt.ylim([0, imgSize])
plt.axis('off')
plt.savefig("color.png", dpi=250)


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
plt.imshow(img)#, vmax=1.75)#, cmap='gray_r')
plt.savefig("gray.png", dpi=250)

for cent in centroids:
    x,y = cent.xyCtr
    plt.scatter(x,y,edgecolors='red', facecolors='red', marker="+", linewidths=2)
plt.savefig("gray_targ.png", dpi=250)

betaGeom = 9 # 1mm widefin!
printEvery = 0
stepSize = 0.03 # degrees
nDia = 25
# r_phys = 2
b_step = 0.02
b_smooth = 0.04
epsilon = stepSize * 2.2

collisionBuffer = 1.5
offset = imgSize / 2.

cKaiju.initBetaArms()
rg = cKaiju.RobotGrid(nDia, stepSize, betaGeom, printEvery, collisionBuffer, epsilon, 0)
print("setting alpha beta")

for robot in rg.allRobots:
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
rg.pathGen()
print('path gen done')


# gross way to hold things
robotList = []
for ri in range(rg.nRobots):
    robot = rg.getRobot(ri)
    # only keep positions (not time), and downsample
    alphaX = numpy.asarray(robot.roughAlphaX)[:,1]
    alphaY = numpy.asarray(robot.roughAlphaY)[:,1]
    betaX = numpy.asarray(robot.roughBetaX)[:,1]
    betaY = numpy.asarray(robot.roughBetaY)[:,1]

    alphaXds = list(numpy.hstack((alphaX[0:-1:50], alphaX[-1])))
    alphaYds = list(numpy.hstack((alphaY[0:-1:50], alphaY[-1])))
    betaXds = list(numpy.hstack((betaX[0:-1:50], betaX[-1])))
    betaYds = list(numpy.hstack((betaY[0:-1:50], betaY[-1])))
    robotList.append([alphaXds, alphaYds, betaXds, betaYds, robot.xPos, robot.yPos])

# reverse robot list
for r in robotList:
    for a in r[:-2]:
        a.reverse()

figOffset = 0

def plotStep(figOffset, step):
    plt.figure(figsize=(10,10))
    plt.xlim([0, imgSize])
    plt.ylim([0, imgSize])
    plt.axis('off')
    plt.imshow(img)#, vmax=1.75)#, cmap='gray_r')
    ax = plt.gca()

    for r in robotList:
        alphaX = r[0][step] + offset
        alphaY = r[1][step] + offset
        betaX = r[2][step] + offset
        betaY = r[3][step] + offset
        x = r[4] + offset
        y = r[5] + offset
        plt.plot([x, alphaX], [y, alphaY], color='white', linewidth=2, alpha=0.5)

        topCollideLine = LineString(
            [(alphaX, alphaY), (betaX, betaY)]
        ).buffer(collisionBuffer, cap_style=1)
        topcolor = 'white'
        edgecolor = 'yellow'
        patch = PolygonPatch(topCollideLine, fc=topcolor, ec=edgecolor, alpha=0.5, zorder=10)
        ax.add_patch(patch)

    for cent in centroids:
        x,y = cent.xyCtr
        if cent.got:
            if figOffset > 0:
                color = 'green'
                marker = 'o'
                facecolor = "none"
            else:
                color = "gold"
                facecolor = "gold"
                marker = '+'
            plt.scatter(x,y,edgecolors=color, facecolors=facecolor, marker=marker, linewidths=2)


    plt.savefig("step_%04d.png"%(step+figOffset), dpi=250)
    plt.close()



stepArray = range(len(robotList[0][0]))
# pltPartial = functools.partial(plotStep, robotList)
ps = functools.partial(plotStep, 0)

p = Pool(12)
p.map(ps, stepArray)
p.close()

# figOffset += len(stepArray)
for expStep in range(20):
    figOffset += 1
    plotStep(figOffset, stepArray[-1])

# reverse robot list
figOffset += 1 + stepArray[-1]
ps = functools.partial(plotStep, figOffset)
for r in robotList:
    for a in r[:-2]:
        a.reverse()

p = Pool(12)
p.map(ps, stepArray)
p.close()


# plt.show()
# ffmpeg -r 10 -f image2 -i step_%04d.png -pix_fmt yuv420p robotMovie.mp4

