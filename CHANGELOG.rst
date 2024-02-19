.. _kaiju-changelog:

Changelog
=========

* **1.4.0** Unpin ``simplification`` and ``Shapely`` and allow them to use the latest versions. Added a workflow to build wheels for macOS and Linux on Python 3.9+. This should make Kaiju compatible with 3.11 and 3.12.

* **1.3.1** Require ``shapely<2.0.0`` since that version seems to break ``descartes`` and some of the plotting.

* **1.3.0** Introduce MDP2 path generation strategy, tries iteratively to
solve paths.

* **1.2.3** Rework smoothing strategy.

* **1.2.2** Add APO/LCO specific versions for a ``robotGrid``.

* **1.2.1** Download libcoordio source files when building.

* **1.2.0** Implement dither method for robots. Add options to ``pathGenGreedy`` to exit early in case of deadlock and to allow path generation to start from an collided state. Add an option to explode a single robot at a time. Add GFA avoidance.  Modify API for robotGrid.isCollidedWithAssigned, and RobotGrid.wouldCollideWithAssigned, now returning info about GFAs.  Move collsion buffer out of RobotGrid to Robot (allows for unique collision buffer for each robot, fiducal, GFA).

* **1.1.0**  Add coordio dependency, implement handling for calibration offsets.

* **1.0.0**  Using Rick's new designReference file specifying locations of robots and fiducials.  Alpha zero modified to point the correct direction with respect to the hexagonal grid.

* **0.4.0**  Begining of version history.  Supporting varible sized hex grids, variable shaped beta arms, and no fiducials.  Skeleton docs building.  Pip/sdss installable.  07/15/2019.
