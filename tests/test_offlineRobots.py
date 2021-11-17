from kaiju.robotGrid import RobotGridCalib
from kaiju import utils
import numpy

angStep = 0.05
collisionBuffer = 2.3
epsilon = 2.2*angStep
hasApogee = True


def test_offlineRobots(plot=False):
    # xPos, yPos = utils.hexFromDia(35, pitch=22.4)
    seed = 6
    rg = RobotGridCalib(
        stepSize=angStep,
        epsilon=epsilon, seed=seed
    )
    rg.setCollisionBuffer(collisionBuffer)


    for rID in rg.robotDict:
        robot = rg.getRobot(rID)
        if rID == 1255:
            robot.setAlphaBeta(305, 238.29)
            robot.isOffline = True
        elif rID == 717:
            robot.setAlphaBeta(0,180)
            robot.isOffline = True
        elif rID == 1367:
            robot.setAlphaBeta(0, 164.88)
            robot.isOffline = True
        elif rID == 398:
            robot.setAlphaBeta(0, 152.45)
            robot.isOffline = True
        else:
            robot.setXYUniform()

    rg.decollideGrid()

    # for robot in rg.robotDict.values():
    #     if numpy.random.uniform() > 0.998:

    #         robot.isOffline = True

    for robot in rg.robotDict.values():
        robot.setDestinationAlphaBeta(0, 180)

    assert rg.getNCollisions() == 0
    rg.pathGenMDP(0.8, 0.2)
    print("didfai", rg.didFail)
    if plot:
        utils.plotPaths(rg, downsample=int(numpy.floor(10 / angStep)), filename="offline.mp4")

if __name__ == "__main__":
    test_offlineRobots(plot=True)