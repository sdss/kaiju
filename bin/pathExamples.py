from kaiju import cKaiju
import matplotlib.pyplot as plt
import numpy

collisionBuffer = 0.25
stepSize = 0.03 # degrees
nDia = 25
epsilon = stepSize * 2.2
b_smooth = 0.04
# max speed = 30 deg per sec
# dt / step = stepSize /
# 0.03 degrees / dt = 30
deg_per_sec = 30
sec_per_step = stepSize / deg_per_sec

totalRobots = 0
maxPts = []

for seed in range(15):
    rg = cKaiju.RobotGrid(nDia, stepSize, epsilon, seed)
    rg.setCollisionBuffer(collisionBuffer)

    totalRobots += rg.nRobots
    rg.decollide()
    rg.pathGen()

    rg.shrinkCollisionBuffer(b_smooth)
    rg.smoothPaths()
    rg.verifySmoothed()

    print("interp collisions", rg.smoothCollisions)
    if rg.smoothCollisions > 0:
        print("skipping grid")
        continue

    for r in rg.allRobots:
        roughAlpha = numpy.asarray(r.alphaPath)
        roughBeta = numpy.asarray(r.betaPath)
        sa = numpy.asarray(r.smoothAlphaPath)
        sb = numpy.asarray(r.smoothBetaPath)
        if len(sb) < 20:
            continue
        maxPts.append(numpy.max([len(sb), len(sa)]))
        with open("robot%i_seed%i.txt"%(r.id, seed), "w") as f:
            alphaStepStr = ",".join(["%i"%s for s in roughAlpha[:,0]])
            alphaDegStr = ",".join(["%.8f"%s for s in roughAlpha[:,1]])
            betaStepStr = ",".join(["%i"%s for s in roughBeta[:,0]])
            betaDegStr = ",".join(["%.8f"%s for s in roughBeta[:,1]])

            s_alphaStepStr = ",".join(["%i"%s for s in sa[:,0]])
            s_alphaDegStr = ",".join(["%.8f"%s for s in sa[:,1]])
            s_betaStepStr = ",".join(["%i"%s for s in sb[:,0]])
            s_betaDegStr = ",".join(["%.8f"%s for s in sb[:,1]])

            f.write("roughAlphaStep: %s\n"%alphaStepStr)
            f.write("roughAlphaDeg: %s\n"%alphaDegStr)
            f.write("roughBetaStep: %s\n"%betaStepStr)
            f.write("roughBetaDeg: %s\n"%betaDegStr)

            f.write("smoothAlphaStep: %s\n"%s_alphaStepStr)
            f.write("smoothAlphaDeg: %s\n"%s_alphaDegStr)
            f.write("smoothBetaStep: %s\n"%s_betaStepStr)
            f.write("smoothBetaDeg: %s\n"%s_betaDegStr)

print("percent robots > 20 points: ", len(maxPts) / float(totalRobots))
plt.hist(maxPts)
plt.xlabel("path points > 20")
plt.savefig("pathlen.png")


