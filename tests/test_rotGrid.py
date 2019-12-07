import numpy

from kaiju import RobotGrid, utils



def test_rotGrid(plot=False):
    nDia = 15
    for rotAng in numpy.linspace(0,30,10):
        xPos, yPos = utils.hexFromDia(nDia, pitch=22.4, rotAngle=rotAng)
        rg = RobotGrid()
        for robotID, (x, y) in enumerate(zip(xPos,yPos)):
            rg.addRobot(robotID, x, y, hasApogee=True)
        rg.initGrid()
        for robot in rg.robotDict.values():
            robot.setAlphaBeta(0,180)

        if plot:
            utils.plotOne(0, rg, figname="rotGrid_%.2f.png"%rotAng, isSequence=False)

def test_filledRotGrid(plot=False):
    for rotAng in numpy.linspace(0,30,10):
        rg = utils.robotGridFromFilledHex(rotAngle=rotAng)
        for robot in rg.robotDict.values():
            robot.setAlphaBeta(0,180)
        if plot:
            utils.plotOne(0, rg, figname="rotFilledGrid_%.2f.png"%rotAng, isSequence=False)


if __name__ == "__main__":
    test_rotGrid(plot=True)
    test_filledRotGrid(plot=True)
