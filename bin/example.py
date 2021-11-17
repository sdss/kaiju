import matplotlib
import matplotlib.pyplot as plt
import numpy
from kaiju import cKaiju, utils
from shapely.geometry import LineString
from descartes import PolygonPatch
from multiprocessing import Pool, cpu_count
from subprocess import Popen
import glob
import os

matplotlib.use('Agg')

stepSize = 1 # degrees
nDia = 15
# r_phys = 2
b_step = 0.02
b_smooth = 0.04
epsilon = stepSize * 2.2

internalBuffer = 1.5
collisionBuffer = 0.25

rg = cKaiju.RobotGrid(stepSize, epsilon, 0)
rg.setCollisionBuffer(collisionBuffer)
xPos, yPos = utils.hexFromDia(nDia, pitch=22.4)
for ii, (xp,yp) in enumerate(zip(xPos,yPos)):
    rg.addRobot(ii, xp, yp)
rg.initGrid()
rg.decollide()
rg.pathGen()

steps = range(rg.nSteps)

def plotOne(step):
    plt.figure(figsize=(10,10))
    ax = plt.gca()
    for robot in rg.allRobots:
        alphaX = robot.roughAlphaX[step][1]
        alphaY = robot.roughAlphaY[step][1]
        betaX = robot.roughBetaX[step][1]
        betaY = robot.roughBetaY[step][1]
        plt.plot([robot.xPos, alphaX], [robot.yPos, alphaY], color='black', linewidth=2, alpha=0.5)

        topCollideLine = LineString(
            [(alphaX, alphaY), (betaX, betaY)]
        ).buffer(internalBuffer, cap_style=1)
        topcolor = 'blue'
        edgecolor = 'black'
        patch = PolygonPatch(topCollideLine, fc=topcolor, ec=edgecolor, alpha=0.5, zorder=10)
        ax.add_patch(patch)
    print("fig", step)
    plt.savefig("step_%04d.png"%(step), dpi=250)
    plt.close()

p = Pool(cpu_count())
p.map(plotOne, steps)

fps = 10 # frames per second
args = ['ffmpeg', '-r', '%i'%fps, '-f', 'image2', '-i', 'step_%04d.png',
        '-pix_fmt', 'yuv420p', 'example.mp4']

movie = Popen(args)
movie.wait()

# clean up imgs
imgs = glob.glob("step*.png")
for img in imgs:
    os.remove(img)



