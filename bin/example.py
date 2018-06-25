from kaiju.robot import RobotGrid, throwAway

# number of robots along 'equator' of hexagon
nRobotDia = 11
# minimum separation between fibers in mm
minSeparation = 8
robotGrid = RobotGrid(nRobotDia, minSeparation)

# throw away (replace) any colliding robots until there
# are no collisions left in the grid.  This ensures that
# the final state is non-colliding
throwAway(robotGrid, markAsTossed=False)

# set up some plot limits
# change these if you add more robots to the grid
xlim = [-150,150]
ylim = [-150,150]
robotGrid.xlim = xlim
robotGrid.ylim = ylim
# begin loop to move all robots towards a fold
maxIter = 500
ii = 0
for ii in range(1,maxIter+1):
    print("step %i"%ii)
    for robot in robotGrid.robotList:
        robot.stepTowardFold()
    # save snap shot of grid on each iteration
    robotGrid.plotNext()
    if not False in [robot.alphaBeta[1]==180 for robot in robotGrid.robotList]:
        print("Routine finished successfully!")
        break
if ii == maxIter:
    print("Max iteration reached")


# write pngs to movie with ffmpeg:
# ffmpeg -r 10 -f image2 -i fig%04d.png robotMovie.mp4