import numpy

from kaiju import RobotGrid
from kaiju import utils



def test_rotGrid(plot=False):
    nDia = 15
    for ii, rotAng in enumerate(numpy.linspace(0,90,50)):
        xPos, yPos = utils.hexFromDia(nDia, pitch=22.4, rotAngle=rotAng)
        rg = RobotGrid()
        for robotID, (x, y) in enumerate(zip(xPos,yPos)):
            rg.addRobot(robotID, str(robotID), [x, y, 0], hasApogee=True)
        rg.initGrid()
        for robot in rg.robotDict.values():
            robot.setAlphaBeta(0,180)

        if plot:
            utils.plotOne(0, rg, figname="rotGrid_%s.png"%("%i"%ii).zfill(4), isSequence=False)

# def test_filledRotGrid(plot=False):
#     for ii, rotAng in enumerate(numpy.linspace(0,90,50)):
#         rg = utils.robotGridFromFilledHex(rotAngle=rotAng)
#         for robot in rg.robotDict.values():
#             robot.setAlphaBeta(0,180)
#         if plot:
#             utils.plotOne(0, rg, figname="rotFilledGrid_%s.png"%("%i"%ii).zfill(4), isSequence=False)


if __name__ == "__main__":
    test_rotGrid(plot=True)
    # test_filledRotGrid(plot=True)
