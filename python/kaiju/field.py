# encoding: utf-8
#
# @Author: Tom Dwelly
# @Date: 09-07-2020
# @Filename: bhm_targeting
# @License: BSD 3-Clause

# adapted from  robostrategy/python/robostrategy/field.py
# remove/simplify cadence stuff - we just want to test a single configuration at a time

import os
import sys
import json
import numpy as np

import kaiju
import kaiju.robotGrid
import fitsio
import matplotlib.pyplot as plt
import scipy.optimize as optimize

#import roboscheduler.cadence as cadence
import kaiju
import kaiju.robotGrid



__all__ = ['Field']

"""Field module class.

Dependencies:

 numpy
 fitsio
 matplotlib
 roboscheduler
 kaiju

"""


class Field(object):
    """Field class

    Parameters:
    ----------

    racen : np.float64
        boresight RA, J2000 deg

    deccen : np.float64
        boresight Dec, J2000 deg

    observatory : str
        observatory field observed from, 'apo' or 'lco' (default 'apo')

    fps_layout : str
        name of FPS layout to assume (default 'filled_hex')

    db : boolean
        whether to use database when setting up Robot instance (default True)

    Attributes:
    ----------

    racen : np.float64
        boresight RA, J2000 deg

    deccen : np.float64
        boresight Dec, J2000 deg

    observatory : str
        observatory field observed from ('apo' or 'lco')

    field_cadence : int, np.int32
        name of field cadence (as given in cadencelist)

    robotgrids : list
        instances of RobotGrid, one per exposure

    cadencelist : CadenceList class
        instance of CadenceList (singleton)

    ntarget : int or np.int32
        number of targets

    target_array : ndarray
        ndarray with target info, exact format varies

    target_ra : ndarray of np.float64
        RA of targets, J2000 deg

    target_dec : ndarray of np.float64
        Dec of targets, J2000 deg

    target_x : ndarray of np.float64
        x positions of targets, mm

    target_y : ndarray of np.float64
        y positions of targets, mm

    target_within : ndarray of np.int32
        1 if target is within the robot hexagon, 0 otherwise

    target_priority : ndarray of np.int32
        priorities of targets (lower is considered first)

    target_program : ndarray of strings
        program of targets

    target_category : ndarray of strings
        category of targets ('SKY', 'STANDARD', 'SCIENCE')

    target_pk : ndarray of np.int64
        unique primary key for each target

    target_cadence : ndarray of np.int32
        cadences of targets

    target_incadence : ndarray of np.bool
        whether each target is allowed in the field cadence (set by assign())

    target_assigned : ndarray of np.int32
        (ntarget) array of 0 or 1, indicating whether target is assigned

    target_assignments : ndarray of np.int32
        (ntarget, nexposure) array of positionerid for each target

    assignment : ndarray of np.int32
        (npositioner, nexposure) array of target indices

    greedy_limit : int or np.int32
        number of exposures above which assign() uses greedy algorithm

    Methods:
    -------

    targets_fromarray() : read targets from an ndarray
    targets_fromfits() : read targets from a FITS file
    targets_toarray() : write targets to an ndarray
    tofits() : write targets (and assignments) to a FITS file
    assign() : assign targets to robots for cadence
    plot() : plot assignments of robots to targets

    Notes:
    -----

    assignments gives a direct index into the target_* arrays, or -1
    for unassigned positioner-exposures. It does not contain
    target_pk.
"""
    def __init__(self,
                 racen=None,
                 deccen=None,
                 pa=0.,
                 db=True,
                 observatory='apo',
                 nsky_apogee=20,
                 nsky_boss=50,
                 nstandard_apogee=20,
                 nstandard_boss=20,
                 collisionBuffer=2.0,
    ):
        self.stepSize = 1  # for kaiju
        assert (collisionBuffer > 0.0 and collisionBuffer < 10.0 )
        self.collisionBuffer = collisionBuffer  # for kaiju
        self.mastergrid = self._robotGrid()
        self.robotID2indx = dict()
        self.indx2RobotID = dict()
        for i, k in enumerate(self.mastergrid.robotDict):
            self.robotID2indx[k] = i
            self.indx2RobotID[i] = k
        self.robotgrids = []
        self.stepSize = 1
        self.racen = racen
        self.deccen = deccen
        self.pa = pa  # assume deg E of N
        self.observatory = observatory
        self.set_vertices()
        #self.cadencelist = cadence.CadenceList()
        self.set_field_cadence('single')
        self.assignments = None
        self.target_assigned = None
        self.target_assignments = None
        self.target_incadence = None
        self.greedy_limit = 100
        self.nsky_apogee = nsky_apogee
        self.nstandard_apogee = nstandard_apogee
        self.nsky_boss = nsky_boss
        self.nstandard_boss = nstandard_boss
        return

    def _arrayify(self, quantity=None, dtype=np.float64):
        """Cast quantity as ndarray of numpy.float64"""
        try:
            length = len(quantity)
        except TypeError:
            length = 1
        return np.zeros(length, dtype=dtype) + quantity

    def _robotGrid(self):
        """Return a RobotGrid instance"""
        rg = kaiju.robotGrid.RobotGridFilledHex(collisionBuffer=self.collisionBuffer)
        for k in rg.robotDict.keys():
            rg.robotDict[k].setAlphaBeta(0., 180.)
        return(rg)

    def set_vertices(self):
        maxReach = self.mastergrid.robotDict[1].getMaxReach()
        xPos = np.array([self.mastergrid.robotDict[r].xPos
                         for r in self.mastergrid.robotDict])
        yPos = np.array([self.mastergrid.robotDict[r].yPos
                         for r in self.mastergrid.robotDict])
        rPos = np.sqrt(xPos**2 + yPos**2)
        ivert = np.argsort(rPos)[-6:]
        xVert = np.zeros(6, dtype=np.float64)
        yVert = np.zeros(6, dtype=np.float64)
        for i, cvert in enumerate(ivert):
            rOut = rPos[cvert] + maxReach
            xVert[i] = xPos[cvert] * rOut / rPos[cvert]
            yVert[i] = yPos[cvert] * rOut / rPos[cvert]
        thVert = np.arctan2(yVert, xVert)
        isort = np.argsort(thVert)
        self.xVert = np.zeros(7, dtype=np.float64)
        self.yVert = np.zeros(7, dtype=np.float64)
        self.xVert[0:6] = xVert[isort]
        self.yVert[0:6] = yVert[isort]
        self.xVert[6] = self.xVert[0]
        self.yVert[6] = self.yVert[0]
        self.raVert, self.decVert = self.xy2radec(self.xVert, self.yVert)
        return

    def set_target_assignments(self):
        """Convert assignments array to per-target basis

        Notes:
        ------

        Sets attributes target_assignment and target_assigned based on
        the assignments attribute values.
"""
        if(self.assignments is None):
            return

        nexp = self.nexposures
        self.target_assignments = np.zeros((self.ntarget, nexp),
                                           dtype=np.int32) - 1
        self.target_assigned = np.zeros(self.ntarget, dtype=np.int32)
        for iexp in np.arange(nexp, dtype=np.int32):
            rg = self.mastergrid
            for robotID in rg.robotDict:
                irobot = self.robotID2indx[robotID]
                curr_assignment = self.assignments[irobot, iexp]
                if(curr_assignment >= 0):
                    tindx = self.targetID2indx[curr_assignment]
                    self.target_assigned[tindx] = 1
                    self.target_assignments[tindx, iexp] = robotID

        return

    def radec2xy(self, ra=None, dec=None):
        # Yikes!
        if(self.observatory == 'apo'):
            scale = 218.
        if(self.observatory == 'lco'):
            scale = 329.

        # From Meeus Ch. 17
        deccen_rad = self.deccen * np.pi / 180.
        racen_rad = self.racen * np.pi / 180.
        dec_rad = dec * np.pi / 180.
        ra_rad = ra * np.pi / 180.
        x = (np.cos(deccen_rad) * np.sin(dec_rad) -
             np.sin(deccen_rad) * np.cos(dec_rad) *
             np.cos(ra_rad - racen_rad))
        y = np.cos(dec_rad) * np.sin(ra_rad - racen_rad)
        z = (np.sin(deccen_rad) * np.sin(dec_rad) +
             np.cos(deccen_rad) * np.cos(dec_rad) *
             np.cos(ra_rad - racen_rad))
        d_rad = np.arctan2(np.sqrt(x**2 + y**2), z)

        pay = np.sin(ra_rad - racen_rad)
        pax = (np.cos(deccen_rad) * np.tan(dec_rad) -
               np.sin(deccen_rad) * np.cos(ra_rad - racen_rad))
        pa_rad = np.arctan2(pay, pax)  # I think E of N?

        pa_rad = pa_rad - self.pa * np.pi / 180.

        x = d_rad * 180. / np.pi * scale * np.sin(pa_rad)
        y = d_rad * 180. / np.pi * scale * np.cos(pa_rad)

        return(x, y)

    def _min_xy_diff(self, radec, xt, yt):
        x, y = self.radec2xy(ra=radec[0], dec=radec[1])
        resid2 = (x - xt)**2 + (y - yt)**2
        return(resid2)

    def xy2radec(self, x=None, y=None):
        # This doesn't handle poles well
        # Yikes!
        if(self.observatory == 'apo'):
            scale = 218.
        if(self.observatory == 'lco'):
            scale = 329.
        xa = self._arrayify(x, dtype=np.float64)
        ya = self._arrayify(y, dtype=np.float64)
        rast = self.racen - xa / scale / np.cos(self.deccen * np.pi / 180.)
        decst = self.deccen + ya / scale
        ra = np.zeros(len(xa), dtype=np.float64)
        dec = np.zeros(len(xa), dtype=np.float64)
        for i in np.arange(len(xa)):
            res = optimize.minimize(self._min_xy_diff, [rast[i], decst[i]],
                                    (xa[i], ya[i]))
            ra[i] = res.x[0]
            dec[i] = res.x[1]
        return(ra, dec)

    def _targets_fromarray_strings(self):
        #try:
        #    self.target_cadence = np.array(
        #        [c.decode().strip() for c in self.target_array['cadence']])
        #    self.target_nexposures = np.array(
        #        [1 if x == 'none' else
        #         self.cadencelist.cadences[x].nexposures
        #         for x in self.target_cadence])
        #except AttributeError:
        #    self.target_cadence = np.array(
        #        [c.strip() for c in self.target_array['cadence']])

        try:
            self.target_category = np.array(
                [c.decode().strip() for c in self.target_array['category']])
        except AttributeError:
            self.target_category = np.array(
                [c.strip() for c in self.target_array['category']])
        except ValueError:
            self.target_category = np.array(['SCIENCE'] * self.ntarget)

        try:
            self.target_program = np.array(
                [c.decode().strip() for c in self.target_array['program']])
        except AttributeError:
            self.target_program = np.array(
                [c.strip() for c in self.target_array['program']])
        except ValueError:
            self.target_program = np.array(['PROGRAM'] * self.ntarget)

        self.targetID2indx = dict()
        for itarget in np.arange(self.ntarget, dtype=np.int32):
            self.targetID2indx[self.target_id[itarget]] = itarget
        return

    def _targets_fromarray_mastergrid(self):
        # Add all targets to master grid.
        for itarget in np.arange(self.ntarget, dtype=np.int32):
            if(self.target_requires_apogee[itarget]):
                fiberType = kaiju.ApogeeFiber
            else:
                fiberType = kaiju.BossFiber
            self.mastergrid.addTarget(targetID=self.target_id[itarget],
                                      x=self.target_x[itarget],
                                      y=self.target_y[itarget],
                                      priority=self.target_priority[itarget],
                                      fiberType=fiberType)
        return

    def _targets_fromarray_within(self):
        self.target_within = np.zeros(self.ntarget, dtype=np.bool)
        for tid, t in self.mastergrid.targetDict.items():
            itarget = self.targetID2indx[tid]
            self.target_within[itarget] = len(t.validRobotIDs) > 0
        return

    def _targets_fromarray_valid(self):
        self.robot_validitargets = dict()
        for rid in self.mastergrid.robotDict:
            robot = self.mastergrid.robotDict[rid]
            self.robot_validitargets[rid] = np.array([self.targetID2indx[x]
                                                      for x in robot.validTargetIDs])
        return

    def targets_fromarray(self, target_array=None, add_to_mastergrid=True):
        """Read targets from an ndarray

        Parameters:
        ----------

        target_array : ndarray
            ndarray with columns below

        Notes:
        ------

        Required columns of array:
         'ra', 'dec' should be np.float64
         'pk' should be np.int64
         'cadence', 'type' should be str or bytes

        Optional columns of array:
         'priority'
         'category'
         'program'
"""
        self.target_array = target_array
        self.ntarget = len(self.target_array)
        self.target_ra = self.target_array['ra']
        self.target_dec = self.target_array['dec']
        try:
            self.target_id = self.target_array['targetid']
        except:
            print("FAKE TARGET ID")
            self.target_id = np.arange(0, self.ntarget * 10, 10,
                                       dtype=np.int32)
            np.random.shuffle(self.target_id)
        self.target_pk = self.target_array['pk']
        self.target_x, self.target_y = self.radec2xy(self.target_ra,
                                                     self.target_dec)

        self._targets_fromarray_strings()

        try:
            self.target_priority = self.target_array['priority']
        except ValueError:
            self.target_priority = np.ones(self.ntarget, dtype=np.int32)

        try:
            self.target_value = self.target_array['value']
        except ValueError:
            self.target_value = np.ones(self.ntarget, dtype=np.int32)

        self.target_requires_apogee = np.zeros(self.ntarget, dtype=np.int8)
        iscience = np.where(self.target_category == 'SCIENCE')[0]
        #self.target_requires_apogee[iscience] = [self.cadencelist.cadences[c].requires_apogee
        #                                         for c in self.target_cadence[iscience]]
        self.target_requires_apogee[iscience] = False
        self.target_requires_boss = np.zeros(self.ntarget, dtype=np.int8)
        #self.target_requires_boss[iscience] = [self.cadencelist.cadences[c].requires_boss
        #                                       for c in self.target_cadence[iscience]]
        self.target_requires_boss[iscience] = True
        inotscience = np.where(self.target_category != 'SCIENCE')[0]
        ttype = [t.split('_')[-1] for t in self.target_category[inotscience]]
        self.target_requires_apogee[inotscience] = (ttype == 'APOGEE')
        self.target_requires_boss[inotscience] = (ttype == 'BOSS')

        if(add_to_mastergrid is True):
            self._targets_fromarray_mastergrid()
            self._targets_fromarray_within()
            self._targets_fromarray_valid()

        return

    def targets_fromfits(self, filename=None, add_to_mastergrid=True):
        """Read targets from a FITS file

        Parameters:
        ----------

        filename : str
            FITS file name, for file with columns listed below

        Notes:
        ------

        Required columns:
         'ra', 'dec' should be np.float64
         'targetid' should be np.int64
         'cadence' should be str or bytes

        Optional columns:
         'priority'
         'category'
         'program'
"""
        target_array = fitsio.read(filename)
        self.targets_fromarray(target_array, add_to_mastergrid=add_to_mastergrid)
        return

    def set_field_cadence(self, field_cadence='none'):
        self.field_cadence = field_cadence
        self.nexposures = 1
    #    if(self.field_cadence != 'none'):
    #        self.nexposures = self.cadencelist.cadences[self.field_cadence].nexposures
    #    else:
    #        self.nexposures = 0
    #    return

    def fromfits(self, filename=None, read_assignments=True, make_grids=True):
        """Read field from a FITS file

        Parameters:
        ----------

        filename : str
            FITS file name, where HDU 2 has array of assignments
"""
        hdr = fitsio.read_header(filename, ext=1)
        self.racen = np.float64(hdr['RACEN'])
        self.deccen = np.float64(hdr['DECCEN'])
        #self.set_field_cadence(hdr['FCADENCE'].strip())
        self.set_field_cadence('single')
        #if((self.field_cadence != 'none') & (read_assignments)):
        #    self.assignments = fitsio.read(filename, ext=2)
        self.targets_fromfits(filename, add_to_mastergrid=make_grids)
        if(make_grids):
            self.make_robotgrids()
        #if((self.field_cadence != 'none') & (read_assignments)):
        #    self.set_target_assignments()
        if(read_assignments & make_grids):
            self._assignments_to_grids()
        return

    def targets_toarray(self):
        """Write targets to an ndarray

        Returns:
        -------

        target_array : ndarray
            Array of targets, with columns:
              'ra', 'dec' (np.float64)
              'pk' (np.int64)
              'cadence' ('a30')
              'priority' (np.int32)
              'category' ('a30')
              'program' ('a30')
"""
        target_array_dtype = np.dtype([('ra', np.float64),
                                       ('dec', np.float64),
                                       ('targetid', np.int64),
                                       ('pk', np.int64),
                                       #('cadence', cadence.fits_type),
                                       ('cadence', np.dtype('a10')),
                                       ('category', np.dtype('a30')),
                                       ('program', np.dtype('a30')),
                                       ('value', np.int32),
                                       ('priority', np.int32),
                                       ('within', np.bool),
                                       ('x', np.float64),
                                       ('y', np.float64),
                                       ('assigned', np.bool),
                                       ('robotid', np.int32)])

        target_array = np.zeros(self.ntarget, dtype=target_array_dtype)
        target_array['ra'] = self.target_ra
        target_array['dec'] = self.target_dec
        target_array['pk'] = self.target_pk
        target_array['targetid'] = self.target_id
        #target_array['cadence'] = self.target_cadence
        target_array['cadence'] = 'single'
        target_array['category'] = self.target_category
        target_array['program'] = self.target_program
        target_array['value'] = self.target_value
        target_array['priority'] = self.target_priority
        target_array['within'] = self.target_within
        target_array['x'] = self.target_x
        target_array['y'] = self.target_y
        target_array['assigned'] = self.target_assigned

        self.set_target_assignments()
        target_array['robotid'] = self.target_assignments[:, 0]
        #print(self.target_assignments)


        return(target_array)

    def tofits(self, filename=None, clobber=True):
        """Write targets to a FITS file

        Parameters:
        ----------

        filename : str
            file name to write to

        clobber : boolean
            if True overwrite file, otherwise add an extension

        Notes:
        -----

        Writes header keywords:

            RACEN
            DECCEN
            FCADENCE (if determined)

        Tables has columns:

            'ra', 'dec' (np.float64)
            'pk' (np.int64)
            'cadence', 'type' ('a30')
            'priority' (np.int32)
            'category' ('a30')
            'program' ('a30')
"""
        hdr = dict()
        hdr['RACEN'] = self.racen
        hdr['DECCEN'] = self.deccen
        if(self.field_cadence is not None):
            hdr['FCADENCE'] = self.field_cadence
        tarray = self.targets_toarray()
        fitsio.write(filename, tarray, header=hdr, clobber=clobber)
        if(self.assignments is not None):
            fitsio.write(filename, self.assignments, clobber=False)
        return

    def plot(self, epochs=None):
        """Plot assignments of robots to targets for field

        Parameters:
        ----------

        epochs : list or ndarray, of int or np.int32
            list of epochs to plot (integers)
"""
        if(epochs is None):
            if(self.assignments is not None):
                epochs = np.arange(self.assignments.shape[1])
            else:
                epochs = np.arange(0)
        else:
            epochs = self._arrayify(epochs, dtype=np.int32)

        #target_cadence = np.sort(np.unique(self.target_cadence))
        colors = ['black', 'green', 'blue', 'cyan', 'purple', 'red',
                  'magenta', 'grey']
        #for indx in np.arange(len(target_cadence)):
        #    itarget = np.where(self.target_cadence ==
        #                       target_cadence[indx])[0]
        #    icolor = indx % len(colors)
        #    plt.scatter(self.target_x[itarget],
        #                self.target_y[itarget], s=2, color=colors[icolor])
        icolor = 0
        plt.scatter(self.target_x[:],
                    self.target_y[:], s=2, color=colors[icolor])


        if(self.assignments is not None):
            target_got = np.zeros(self.ntarget, dtype=np.int32)
            iassigned = np.where(self.assignments.flatten() >= 0)[0]
            itarget = np.array([self.targetID2indx[x] for x in
                                self.assignments.flatten()[iassigned]])
            target_got[itarget] = 1
            #for indx in np.arange(len(target_cadence)):
            #    itarget = np.where((target_got > 0) &
            #                       (self.target_cadence ==
            #                        target_cadence[indx]))[0]
            #    icolor = indx % len(colors)
            #    plt.scatter(self.target_x[itarget],
            #                self.target_y[itarget], s=20,
            #                color=colors[icolor],
            #                label=target_cadence[indx])
            icolor = 0
            plt.scatter(self.target_x[:],
                        self.target_y[:], s=20,
                        color=colors[icolor],
                        label='')

        xcen = np.array([r.xPos for r in self.mastergrid.robotDict],
                        dtype=np.float32)
        ycen = np.array([r.yPos for r in self.mastergrid.robotDict],
                        dtype=np.float32)
        plt.scatter(xcen, ycen, s=6, color='grey', label='Used robot')

        if(self.assignments is not None):
            used = (self.assignments >= 0).sum(axis=1) > 0
        else:
            used = np.zeros(self.mastergrid.nRobots, dtype=np.bool)

        inot = np.where(used == False)[0]
        plt.scatter(xcen[inot], ycen[inot], s=20, color='grey',
                    label='Unused robot')

        plt.xlim([-370., 370.])
        plt.ylim([-370., 370.])
        plt.legend()

    def assign_calibration(self, category=None, kaiju=True):
        """Assign calibration targets to robots within the field

        Notes:
        -----

        Assigns calibration targets. All it attempts at the moment
        is that a certain number will be assigned, according to the
        attribute:

           n{category}

        There is no guarantee regarding the spatial distribution.
        In addition, even the number is not guaranteed.

        The current method goes to each exposure, and does the following:

          * For each unassigned robot, tries to match it to
            one of the calibration targets. Assigns up to
            n{category} robots. It prefers robots used
            for calibration in previous exposures, but beyond
            that picks randomly.

          * If there are less than n{category} calibration
            targets assigned, for each robot assigned to a single
            exposure 'SCIENCE' target, tries to match it to one of the
            calibration targets. Assigns more calibration targets up
            to a total of n{category}, randomly selected. If
            there is more than one exposure in the field cadence,
            tries to assign the replaced targets back to their same
            fiber in an earlier (preferentially) or later exposure.

          * If there are still less than n{category}
            calibration targets assigned, for each robot assigned to a
            any other 'SCIENCE' target, tries to match it to one of
            the calibration targets. Assigns more calibration targets
            up to a total of n{category}. It prefers robots
            used for calibration in previous exposures, but beyond
            that picks randomly. The replaced targets are lost.

        This method is a hack. It will usually get the right number of
        calibration targets but isn't optimized.
"""
        iscalib = (self.target_category == category)
        icalib = np.where(iscalib)[0]
        if(len(icalib) == 0):
            return

        ttype = category.split("_")[-1]

        # Match robots to targets (indexed into icalib)
        curr_robot_targets = dict()
        for rindx, robotID in enumerate(self.mastergrid.robotDict):
            robot = self.mastergrid.robotDict[robotID]
            requires_boss = (ttype == 'BOSS')
            requires_apogee = (ttype == 'APOGEE')

            curr_robot_targets[robotID] = np.zeros(0, dtype=np.int32)
            if(len(robot.validTargetIDs) > 0):
                robot_targets = np.array([self.targetID2indx[x]
                                          for x in robot.validTargetIDs])
                curr_icalib = np.where((iscalib[robot_targets] > 0) &
                                       ((requires_boss == 0) |
                                        (robot.hasBoss > 0)) &
                                       ((requires_apogee == 0) |
                                        (robot.hasApogee > 0)))[0]
                if(len(curr_icalib) > 0):
                    curr_robot_targets[robotID] = robot_targets[curr_icalib]

        ncalib = getattr(self, 'n{c}'.format(c=category).lower())

        # Loop over exposures
        robot_used = np.zeros(self.mastergrid.nRobots, dtype=np.int32)
        for iexp, rg in enumerate(self.robotgrids):
            calibration_assignments = (np.zeros(self.mastergrid.nRobots,
                                                dtype=np.int32) - 1)

            # Initial consistency check
            if(kaiju):
                for robotID in rg.robotDict:
                    rindx = self.robotID2indx[robotID]
                    r = rg.robotDict[robotID]
                    if(r.isAssigned() is False):
                        if(self.assignments[rindx, iexp] >= 0):
                            print("PRECHECK {i}: UH OH DID NOT ASSIGN ROBOT".format(i=iexp))
                    else:
                        assignedID = r.assignedTargetID
                        if(assignedID != self.assignments[rindx, iexp]):
                            print("PRECHECK: UH OH ROBOT DOES NOT MATCH ASSIGNMENT")

            # Now make ordered list of robots to use
            exposure_assignments = self.assignments[:, iexp]
            robot_indx = np.where(exposure_assignments >= 0)[0]
            target_indx = np.array([self.targetID2indx[x]
                                    for x in exposure_assignments[robot_indx]], dtype=np.int32)
            assignment_nexp = np.zeros(self.mastergrid.nRobots,
                                       dtype=np.int32)
            iscience = np.where(self.target_category[target_indx] ==
                                'SCIENCE')[0]
            #assignment_nexp[robot_indx[iscience]] = np.array([
            #    self.cadencelist.cadences[x].nexposures
            #    for x in self.target_cadence[target_indx[iscience]]])

            assignment_nexp[robot_indx[iscience]] = 1
            inot = np.where(self.target_category[target_indx] !=
                            'SCIENCE')[0]
            assignment_nexp[robot_indx[inot]] = -1
            chances = np.random.random(size=self.mastergrid.nRobots)
            sortby = (robot_used * (1 + chances) * 1 +
                      np.int32(assignment_nexp == 1) * (1 + chances) * 2 +
                      np.int32(assignment_nexp == 0) *
                      (1 + robot_used) * (1 + chances) * 4)
            indx_order = np.argsort(sortby)[::-1]

            # Set up calibration assignments in that priority
            got_calib = np.zeros(self.ntarget, dtype=np.int32)
            nassigned = 0
            for indx in indx_order:
                robotID = self.indx2RobotID[indx]
                icalib = curr_robot_targets[robotID]
                for itry in icalib:
                    if(got_calib[itry] == 0):
                        got = True
                        if(kaiju):
                            if(rg.robotDict[robotID].isAssigned()):
                                try:
                                    rg.decollideRobot(robotID)
                                except:
                                    print("unassign failure 1")
                            rg.assignRobot2Target(robotID,
                                                  self.target_id[itry])
                            ## TODO: isCollidedWithAssigned
                            if(rg.isCollidedWithAssigned(robotID) == False):
                                got = True
                            else:
                                got = False
                        if(got):
                            calibration_assignments[indx] = self.target_id[itry]
                            got_calib[itry] = 1
                            robot_used[indx] = 1
                            nassigned = nassigned + 1
                            break
                        if(kaiju):
                            try:
                                rg.decollideRobot(robotID)
                            except RuntimeError:
                                print("unassign failure 2")
                            if(self.assignments[indx, iexp] >= 0):
                                tid = self.assignments[indx, iexp]
                                rg.assignRobot2Target(robotID, tid)
                if(nassigned >= ncalib):
                    break

            # If there is a conflict with a single observation
            # swap with another exposure (need to check collision)
            conflicts = ((calibration_assignments >= 0) &
                         (self.assignments[:, iexp] >= 0))
            single = (assignment_nexp == 1)
            isingle = np.where(conflicts & single)[0]
            for irobot in isingle:
                robotID = self.indx2RobotID[irobot]
                ifree = np.sort(np.where(self.assignments[irobot, :] == -1)[0])
                for itry in ifree:
                    targetID = self.assignments[irobot, iexp]
                    if(kaiju):
                        self.robotgrids[itry].assignRobot2Target(robotID,
                                                                 targetID)
                        ica = self.robotgrids[itry].isCollidedWithAssigned(robotID)
                    else:
                        ica = False
                    if(ica == False):
                        self.assignments[irobot, itry] = targetID
                        self.assignments[irobot, iexp] = -1
                        break
                    if(kaiju):
                        try:
                            self.robotgrids[itry].decollideRobot(robotID)
                        except:
                            print("unassign failure 3")

            # If there is a conflict with a multi-exposure observation
            # just completely unassign (need to actually unassign)
            multi = (assignment_nexp > 1)
            imulti = np.where(conflicts & multi)[0]
            for irobot in imulti:
                robotID = self.indx2RobotID[irobot]
                iother = np.where(self.assignments[irobot, :] ==
                                  self.assignments[irobot, iexp])[0]
                for iun in iother:
                    self.assignments[irobot, iun] = -1
                    try:
                        self.robotgrids[iun].decollideRobot(robotID)
                    except:
                        print("unassign failure4")

            iassign = np.where(calibration_assignments >= 0)[0]
            self.assignments[iassign, iexp] = calibration_assignments[iassign]
            if(kaiju):
                for robotID in rg.robotDict:
                    irobot = self.robotID2indx[robotID]
                    if(rg.robotDict[robotID].isAssigned() is False):
                        if(self.assignments[irobot, iexp] >= 0):
                            print("UH OH DID NOT ASSIGN ROBOT")
                            print("indx={indx}".format(indx=indx))
                            print("iexp={iexp}".format(iexp=iexp))
                            print("itarget={itarget}".format(itarget=self.assignments[irobot, iexp]))
                    elif(rg.robotDict[robotID].assignedTargetID !=
                         self.assignments[irobot, iexp]):
                        print("UH OH ROBOT DOES NOT MATCH ASSIGNMENT")

        if(kaiju):
            for iexp, rg in enumerate(self.robotgrids):
                for robotID in rg.robotDict:
                    if(rg.robotDict[robotID].isAssigned() is False):
                        irobot = self.robotID2indx[robotID]
                        if(self.assignments[irobot, iexp] >= 0):
                            print(f"UH OH DID NOT ASSIGN ROBOT(a) robotID={robotID}")
                        if(rg.isCollided(robotID)):
                            try:
                                rg.decollideRobot(robotID)
                            except:
                                print("unassign failure 5")

    def make_robotgrids(self):
        self.robotgrids = []
        for i in np.arange(self.nexposures):
            self.robotgrids.append(self._robotGrid())
            self.robotgrids[i].clearTargetDict()
            for tid, t in self.mastergrid.targetDict.items():
                self.robotgrids[i].addTarget(targetID=t.id, x=t.x,
                                             y=t.y, priority=t.priority,
                                             fiberType=t.fiberType)
        return

    def assign(self, include_calibration=True, kaiju=True):
        """Assign targets to robots within the field

        Parameters:
        ----------

        include_calibration : boolean
            Assign calibration targets if True, do not if False

        Notes:
        -----

        Field needs to have targets loaded into it. For each robot
        positioner, it searches for targets that are covered and that
        have not been assigned yet for a previous robot.

        It uses the pack_targets_greedy() method of CadenceList to
        pack the target cadences into the field cadence greedily.

        The results are stored in the attribute assignments, which is
        an (nposition, nexposures) array with the target index to
        observe for each positioner in each exposure of the field
        cadence.

        This method is optimal (in the usual case) for individual
        positioners, but not necessarily globally; i.e. trades of
        targets between positioners might be possible that would allow
        a better use of time.

        It first assigns targets in category 'SCIENCE', respecting
        cadence categories.

        Then for each exposure it assigns 'STANDARD' and then 'SKY'
        targets for 'APOGEE' and 'BOSS' fibers. It uses the
        assign_calibration() method in each case.

        It does not use the target priorities yet.

        assign() also sets the target_incadence attribute, which
        tells you wich targets fit somehow into the field cadence.
"""

        logs = []
        logs.append("KAIJU_ASSIGN BEGIN")

        # Initialize
        np.random.seed(int(self.racen))
        nexposures = self.nexposures
        self.assignments = (np.zeros((self.mastergrid.nRobots, nexposures),
                                     dtype=np.int32) - 1)
        got_target = np.zeros(self.ntarget, dtype=np.int32)

        iscience = np.where(self.target_category == 'SCIENCE')[0]

        # Find which targets are viable at all
        #        ok_cadence = dict()
        #        for curr_cadence in np.unique(self.target_cadence[iscience]):
        #            ok, s = self.cadencelist.cadence_consistency(curr_cadence,
        #                                                         self.field_cadence,
        #                                                         return_solutions=True)
        #            ok_cadence[curr_cadence] = (
        #                ok | (self.cadencelist.cadences[curr_cadence].nepochs == 1))
        #        ok = [ok_cadence[tcadence]
        #              for tcadence in self.target_cadence[iscience]]
        #        self.target_incadence = np.zeros(self.ntarget, dtype=np.int32)
        #        iok = np.where(np.array(ok))[0]
        #        self.target_incadence[iscience[iok]] = 1
        #        if(len(iok) == 0):
        #            return
        #####self.target_incadence[:] = 1 # all valid

        # Set up robotgrids
        if(kaiju):
            self.make_robotgrids()
        else:
            self.robotgrids = [None] * self.nexposures

        # Assign the robots
        nexp = self.nexposures
        robotIDs = self.mastergrid.robotDict.keys()
        doneRobots = np.zeros(self.mastergrid.nRobots, dtype=np.bool)

        logs.append(f"SUMMARY_at_start nexp={nexp}")
        logs.append(f"SUMMARY_at_start nRobots={self.mastergrid.nRobots}")

        rng = np.random.default_rng()


        #indx_list = np.arange(self.mastergrid.nRobots, dtype=np.int32)
        ## shuffle the robot assignment order randomly
        #rng = np.random.default_rng()
        #rng.shuffle(indx_list)
        #for indx in indx_list:
        #    irobot = indx

        # rs method: dynamic ordering  """Get next robot in order of highest priority of remaining targets"""
        #for indx in np.arange(self.mastergrid.nRobots, dtype=np.int32):
            #irobot = self._next_robot(robotIDs=robotIDs,
            #                          doneRobots=doneRobots,
            #                          got_target=got_target,
            #                          kaiju=kaiju)

        for indx in np.arange(self.mastergrid.nRobots, dtype=np.int32):
            # rs method: dynamic ordering  """Get next robot in order of highest priority of remaining targets"""
            #irobot = self._next_robot(robotIDs=robotIDs,
            #                          doneRobots=doneRobots,
            #                          got_target=got_target,
            #                          kaiju=kaiju)

            # ALT choose a robot that has the fewest available targets
            irobot = self._next_robot_fewest(robotIDs=robotIDs,
                                             doneRobots=doneRobots,
                                             got_target=got_target,
                                             kaiju=kaiju)

            if irobot is None:  # no robots have any targets left
                break

            doneRobots[irobot] = True
            robotID = self.indx2RobotID[irobot]
            cRobot = self.mastergrid.robotDict[robotID]
            if(len(cRobot.validTargetIDs) == 0) :
                if hasattr(cRobot, 'robot_dict'):
                    logs.append(f"ROBOTSTATS {indx:03d} {irobot:03d} "
                                f"{cRobot.robot_dict['xPos']:8.3f} {cRobot.robot_dict['yPos']:8.3f} "
                                f"NO VALID TARGET IDs!")
                else:
                    logs.append(f"ROBOTSTATS {indx:03d} {irobot:03d} "
                                f"{np.nan:8.3f} {np.nan:8.3f} "
                                f"NO VALID TARGET IDs!")


            if(len(cRobot.validTargetIDs) > 0):
                itargets = np.array([self.targetID2indx[x]
                                     for x in cRobot.validTargetIDs])
                it = np.where(
                    (got_target[itargets] == 0)
                    # (self.target_incadence[itargets] > 0) &
#                    ((self.target_requires_boss[itargets] == 0) | (cRobot.hasBoss > 0)) &
#                    ((self.target_requires_apogee[itargets] == 0) | (cRobot.hasApogee > 0))
                )[0]

                if(len(it) == 0):
                    logs.append(f"ROBOTSTATS {indx:03d} {robotID:03d} "
                          f"{cRobot.xPos:8.3f} {cRobot.yPos:8.3f} "
                          f"nvalidTargetIDs= {len(cRobot.validTargetIDs):3} "
                          f"nGot= {np.count_nonzero(got_target[itargets]):3} "
                          f"nNotGot= {np.count_nonzero(got_target[itargets] == 0):3} "
                          f"len(it)= {len(it):3} " )

                if(len(it) > 0):
                    ifull = itargets[it]
                    # Create mask to pass to pack_targets_greedy() based
                    # on collisions
                    emask = np.zeros((len(ifull), nexp), dtype=np.bool)
                    if(kaiju):
                        for tindx, itarget in enumerate(ifull):
                            for iexp in np.arange(nexp, dtype=np.int32):
                                tid = self.target_id[itarget]
                                try:
                                    self.robotgrids[iexp].assignRobot2Target(robotID, tid)
                                except:
                                    print(iexp)
                                    print(robotID)
                                    print(tid)
                                    print(itarget)
                                    print(self.targetID2indx[tid])
                                    print(self.target_id[itarget])
                                    for v in self.mastergrid.robotDict[robotID].validTargetIDs:
                                        print(v)
                                    for v in self.robotgrids[iexp].robotDict[robotID].validTargetIDs:
                                        print(v)
                                    self.robotgrids[iexp].assignRobot2Target(robotID, tid)
                                if(self.robotgrids[iexp].isCollidedWithAssigned(robotID)):
                                    emask[tindx, iexp] = True
                                # Reset robot -- perhaps there are more elegant ways
                                try:
                                    self.robotgrids[iexp].decollideRobot(robotID)
                                except:
                                    logs.append("unassign failure 6")
                    # p = cadence.Packing(self.field_cadence)
                    #                    p.pack_targets_greedy(
                    #                        target_ids=self.target_id[ifull],
                    #                        target_cadences=self.target_cadence[ifull],
                    #                        value=self.target_value[ifull],
                    #                        exposure_mask=emask)
                    #                    target_ids = p.exposures  # make sure this returns targetid

                    ##junk##print(len(self.target_id))
                    ##junk###exposures = np.zeros(0, dtype=np.int32)
                    ##junk###for et in self.epoch_targets:
                    ##junk###    exposures = np.append(exposures, et)
                    ##junk####target_ids=np.array(self.target_id[ifull], dtype=np.int32)

                    # ignore anything that is masked out (it will collide)

                    # choose the highest priority valid target here
                    eemask = emask[:, 0]
#                    parr = np.where(eemask, 1e30, self.target_priority[ifull])
#                    winners = np.argwhere((parr == np.amin(parr)) & (eemask == False)).flatten().tolist()
#                    rng.shuffle(winners)
#                    if len(winners) > 0:
#                        target_ids=self.target_id[winners]
#                    else:
#                        target_ids = []

                    parr = np.where(eemask, 1e30, self.target_priority[ifull])
                    if np.amin(parr) > 1e29:
                        iparr = []   # nothing to do on this robot
                        for rg in self.robotgrids:
                            try:
                                rg.decollideRobot(robotID)
                            except:
                                logs.append("unassign failure 6")
                    else:
                        iparr = parr.argsort()

                    idarr = self.target_id[ifull]
                    #target_ids=np.array([ idarr[iparr[0]], ], dtype=np.int32)
                    target_ids=idarr[iparr]


                    # this just choses the first in the list:
                    #target_ids=np.array([self.target_id[ifull][0]], dtype=np.int32)
                    #print(target_ids)
                    #print (indx, emask[:, 0],)
                    #print (indx, idarr)
                    #print (indx, parr)
                    #print (indx, target_ids)
                    #print (indx, target_ids)

                    logs.append(f"ROBOTSTATS {indx:03d} {robotID:03d} "
                          f"{cRobot.xPos:8.3f} {cRobot.yPos:8.3f} "
                          f"nvalidTargetIDs= {len(cRobot.validTargetIDs):3} "
                          f"nGot= {np.count_nonzero(got_target[itargets]):3} "
                          f"nNotGot= {np.count_nonzero(got_target[itargets] == 0):3} "
                          f"len(it)= {len(it):3} "
                          f"nPassCond= {len(ifull):3} "
                          f"nCollided= {np.count_nonzero(eemask):3} "
                          f"nRemain= {np.count_nonzero(eemask==False):3} "
                    )

                    if len(iparr) == 0:
                        continue   # nothing to do on this robot


                    # TODO - FIX IT!
                    # the above hack is screwing things up below
                    # need to work out what the expected content of p.exposures is.
                    # Causing lots of 'INCONSISTENCY 1' errors
                    #
                    # need to loop until we find a target that can be assigned without collisions


                    iassigned = np.where(target_ids >= 0)[0]
                    nassigned = len(iassigned)
                    if(nassigned > 0):
                        itarget = np.array([self.targetID2indx[x]
                                            for x in target_ids[iassigned]])
                        got_target[itarget] = 1
                    self.assignments[irobot, :] = target_ids[0]
#                    self.assignments[irobot, :] = target_ids
                    if(kaiju):
                        for iexp, rg in enumerate(self.robotgrids):
                            ctarget = self.assignments[irobot, iexp]
                            if(ctarget >= 0):
                                try:
                                    rg.assignRobot2Target(robotID, ctarget)
                                except RuntimeError:
                                    print(robotID)
                                    print(iexp)
                                    print(ctarget)
                                    print(rg.robotDict[robotID].validTargetIDs)
                                    print("\n".join(logs))
                                    sys.exit(1)
                            else:
                                try:
                                    rg.decollideRobot(robotID)
                                except:
                                    logs.append("unassign failure 7")
                            if(rg.isCollidedWithAssigned(robotID)):
                                #print(robotID)
                                #print(ctarget)
                                #print(rg.robotDict[robotID].assignedTargetID)
                                #print("INCONSISTENCY 1")
                                logs.append(f"TD_INCONSISTENCY1 robotID= {robotID:4d} ctarget= {ctarget:5d} assignedTargetID= {rg.robotDict[robotID].assignedTargetID:5d}")

                                ## added by TD
                                try:
                                    rg.decollideRobot(robotID)
                                except:
                                    logs.append("unassign failure td7")

        # Explicitly unassign all unassigned robots so they
        # are out of the way.
        if(kaiju):
            for iexp, rg in enumerate(self.robotgrids):
                iun = np.where(self.assignments[:, iexp] < 0)[0]
                for irobot in iun:
                    robotID = self.indx2RobotID[irobot]
                    try:
                        rg.decollideRobot(robotID)
                    except:
                        logs.append("unassign failure 8")

        # Make sure all assigned robots are assigned
        if(kaiju):
            for iexp, rg in enumerate(self.robotgrids):
                for indx, robotID in enumerate(rg.robotDict):
                    if(rg.robotDict[robotID].isAssigned() is False):
                        if(self.assignments[indx, iexp] >= 0):
                            logs.append(f"UH OH DID NOT ASSIGN ROBOT(b) robotID={robotID}")

                    elif(rg.robotDict[robotID].assignedTargetID !=
                         self.assignments[indx, iexp]):
                        logs.append("UH OH ROBOT DOES NOT MATCH ASSIGNMENT")

        if(include_calibration):
            self.assign_calibration(category='SKY_APOGEE', kaiju=kaiju)
            self.assign_calibration(category='STANDARD_APOGEE', kaiju=kaiju)
            self.assign_calibration(category='SKY_BOSS', kaiju=kaiju)
            self.assign_calibration(category='STANDARD_BOSS', kaiju=kaiju)

        if(kaiju):
            for iexp, rg in enumerate(self.robotgrids):
                for indx, robotID in enumerate(rg.robotDict):
                    if(rg.robotDict[robotID].isAssigned() is False):
                        if(self.assignments[indx, iexp] >= 0):
                            logs.append(f"UH OH DID NOT ASSIGN ROBOT(c) robotID={robotID}")
                        if(rg.isCollided(robotID)):
                            try:
                                rg.decollideRobot(robotID)
                            except:
                                logs.append("unassign failure 9")

        self.set_target_assignments()

        logs.append("KAIJU_ASSIGN END")
        log = "\n".join(logs)

        return log

    def tojs(self, filename):
        """Write Javascript for each exposure

        Parameters:
        ----------

        filename : str
            name of file to write to

        Comments:
        ---------

        Javascript assigns target information to target_obj dict
        and robot information to robot_obj, an array of dicts
"""
        ii = np.where(self.target_within)[0]
        nwithin = len(ii)
        js_str = "{"
        js_str = js_str + '"racen" : ' + str(self.racen) + ',\n'
        js_str = js_str + '"deccen" : ' + str(self.deccen) + ',\n'
        js_str = js_str + '"field_cadence" : "' + str(self.field_cadence) + '",\n'
        js_str = js_str + '"observatory" : "' + str(self.observatory) + '",\n'
        js_str = js_str + '"ntarget" : ' + str(self.ntarget) + ',\n'
        js_str = js_str + '"nwithin" : ' + str(nwithin) + ',\n'
        js_str = js_str + '"target_obj" : ' + json.dumps(self.mastergrid.target_dict()) + ',\n'
        js_str = js_str + '"robot_obj" : [\n'
        for iexp, rg in enumerate(self.robotgrids):
            js_str = js_str +\
                json.dumps(rg.robot_dict())
            if(iexp < len(self.robotgrids) - 1):
                js_str = js_str + ","
            js_str = js_str + "\n"
        js_str = js_str + ']}'
        fp = open(filename, "w")
        fp.write(js_str)
        fp.close()
        return

    def html(self, filename):
        """Write HTML format file for visualizing assignments

        Parameters:
        ----------

        filename : str
            name of file to write to
"""
        html_str = '<script type="text/javascript">\n'
        html_str = html_str + 'target_obj = ' + json.dumps(self.mastergrid.target_dict()) + ";\n"
        html_str = html_str + 'var robot_obj = new Array();\n'
        for iexp, rg in enumerate(self.robotgrids):
            html_str = html_str +\
                'robot_obj[{iexp}] = '.format(iexp=iexp) +\
                json.dumps(rg.robot_dict()) + ";\n"
        html_str = html_str + '</script>\n'
        fp = open(os.path.join(os.getenv('ROBOSTRATEGY_DIR'), 'data',
                               'field.html'), "r")
        for l in fp.readlines():
            html_str = html_str + l
        fp.close()
        fp = open(filename, "w")
        fp.write(html_str)
        fp.close()
        return

    def _next_robot(self, robotIDs=None, doneRobots=None, got_target=None,
                    kaiju=True):
        """Get next robot in order of highest priority of remaining targets"""
        maxPriority = np.zeros(len(robotIDs), dtype=np.int32) - 9999
        for indx, robotID in enumerate(robotIDs):
            if(doneRobots[indx] == np.bool(False)):
                if(len(self.robot_validitargets[robotID]) > 0):
                    itargets = self.robot_validitargets[robotID]
                    inot = np.where(got_target[itargets] == 0)[0]
                    if(len(inot) > 0):
                        it = itargets[inot]
                        maxPriority[indx] = self.target_priority[it].max()
                else:
                    maxPriority[indx] = - 99999

        winners = np.argwhere(maxPriority == np.amax(maxPriority)).flatten().tolist()
        rng = np.random.default_rng()
        rng.shuffle(winners)
        winner = winners[0]
        return(winner)

#        imax = np.argmax(maxPriority)
#        return(imax)

    def _next_robot_fewest(self, robotIDs=None, doneRobots=None, got_target=None,
                           kaiju=True):
        """Get next robot in order of number of remaining targets"""
        nValid = np.full(len(robotIDs), 999999, dtype=np.int32)
        for indx, robotID in enumerate(robotIDs):
            if(doneRobots[indx] == np.bool(False)):
                if(len(self.robot_validitargets[robotID]) > 0):
                    itargets = self.robot_validitargets[robotID]
                    inot = np.where(got_target[itargets] == 0)[0]
                    if(len(inot) > 0):
                        nValid[indx] = len(inot)

        #imin = np.argmin(nValid)
        #return(imin)
        if np.sum(nValid) == 0:
            print("No remaining robots with available targets")
            return None

        winners = np.argwhere(nValid == np.amin(nValid)).flatten().tolist()
        rng = np.random.default_rng()
        rng.shuffle(winners)
        winner = winners[0]
        #print(f"winner={winner} (winners={winners} n={len(winners)})")
        return(winner)

#    def add_observations(self):
#        """For assigned targets, add observations if possible
#
#        Notes:
#        -----
#
#        The assign() method needs to have been run so that the
#        assignments attribute is set.
#
#        The code then adds as many more observations as it can of
#        the targets already observed by this fiber. Note that it is
#        looking for full new observations.
#
#        It does not use the target priorities yet.
#"""
#
#        # Assign the robots
#        for indx in np.arange(self.mastergrid.nRobots):
#            targetids = np.unique(self.assignments[indx, :])[0]
#            igd = np.where(targetids != -1)[0]
#            igd2 = np.where(self.target_category[igd] != 'CALIBRATION')[0]
#            targetids = targetids[igd[igd2]]
#            tcs = self.target_cadence[targetids]
#            if(len(targetids) > 0):
#                p = cadence.Packing(self.field_cadence)
#                p.import_exposures(self.assignments[indx, :])
#                ok = True
#                while(ok):
#                    ok = False
#                    for tc in tcs:
#                        if(p.add_target(tc)):
#                            ok = True
#
#        return

    def _assignments_to_grids(self):
        """Transfer assignments to RobotGrid objects"""
        nexp = self.nexposures
        for i in np.arange(nexp):
            for rindx, robotID in enumerate(self.robotgrids[i].robotDict):
                targetID = self.assignments[rindx, i]
                if(targetID >= 0):
                    self.robotgrids[i].assignRobot2Target(robotID, targetID)
            for rindx, robotID in enumerate(self.robotgrids[i].robotDict):
                itarget = self.assignments[rindx, i]
                if(itarget < 0):
                    try:
                        self.robotgrids[i].decollideRobot(robotID)
                    except:
                        print("unassign failure 10")
        return