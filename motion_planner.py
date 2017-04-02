#!/usr/bin/env python
# coding: utf-8
import numpy as np
import os,sys
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
from matplotlib import patches
import time
import argparse
import operator
from math import acos
from collections import defaultdict,namedtuple
import pylab
import random
import logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
from pympler.asizeof import asizeof
try:
    import cPickle as pickle
except:
    logging.warn("Unable to load cPickle.  Falling back to python pickle.")
    import pickle

__doc__ = "Simple geometric motion planner for Valkyrie Robotics 2017 FIRST competition."
__author__ = "Mark E. Plutowski"
__credits__ = "Valkyrie Robotics LLC"
__copyright__ = "Copyright 2017 Mark E. Plutowski"
__license__ = "MIT"
__version__ = "1.0.0"


## namedtuple reduces memory 42% vs dict
StateNamedtuple = namedtuple('State', 'goal1 dist type r goal2 dist2 rot samples')
StateNamedtuple.__new__.__defaults__ = (None, None, None, None, 1) # goal1,dist,type required


class StateNT(object):
    """
    Wraps StateNamedtuple with getters to facilitate easily swapping with StateP.
    This is an unpacked version of StateP
    """
    __slots__ = ('state') # prevents adding fields but reduces mem consumption by 36%
    def __init__(self, goal1, dist, type, r=None, goal2=None, dist2=None, rot=None, samples=1):
        self.state = StateNamedtuple(goal1,dist,type,r,goal2,dist2,rot,samples)
    def goal1(self):
        return self.state.goal1
    def dist(self):
        return self.state.dist
    def type(self):
        return self.state.type
    def r(self):
        return self.state.r
    def goal2(self):
        return self.state.goal2
    def dist2(self):
        return self.state.dist2
    def rot(self):
        return self.state.rot
    def samples(self):
        return self.state.samples
    def __repr__(self):
        return 'StateNT(goal1=%s,dist=%s,type=%s,r=%s,goal2=%s,dist2=%s,rot=%s,samples=%d' % \
               (self.goal1, self.dist, self.type, self.r, self.goal2, self.dist2, self.rot, self.samples)


TURN_TYPES = ['lh', 'rh', '|', 'qt']
class StateP(object):
    """
    Bytearray packed configuration state.
    Namedtuple can require up to 1000 bytes per stage2 row,
    """
    # See also unpack_v23
    __slots__ = ('state')
    def __init__(self, goal1, dist, type, r=None, goal2=None, dist2=None, rot=None, samples=1):
        if type not in TURN_TYPES:
            raise ValueError("Bad turntype: %s" % type)
        self.state = pack_v23(StateNamedtuple(goal1,dist,type,r,goal2,dist2,rot,samples))
    def goal1(self):
        return (bytes_to_int(self.state[0:2])-SHIFT, bytes_to_int(self.state[2:4])-SHIFT, bytes_to_int(self.state[4:6])-SHIFT)
    def dist(self):
        return bytes_to_int(self.state[6:8])/100.0
    def rawtype(self):
        return chr(self.state[8])
    def type(self):
        type = self.rawtype()
        return  'qt' if type in ('q','Q') else '|' if type in ('f', 'b') else 'rh' if type == 'r' else 'lh' if type == 'l' else None
    def r(self):
        val = bytes_to_int(self.state[9:11])
        return None if val == NULL_2BYTE else val/10.0
    def goal2(self):
        val = (bytes_to_int(self.state[11:13]), bytes_to_int(self.state[13:15]), bytes_to_int(self.state[15:17]))
        return None if val == NULL_XYT else (val[0]-SHIFT,val[1]-SHIFT,val[2]-SHIFT)
    def dist2(self):
        val = bytes_to_int(self.state[17:19])
        rawtype = self.rawtype()
        return None if val == NULL_2BYTE else -val/100.0 if rawtype=='b' else val/100.0 if rawtype in ['f','l','r','q','Q'] else None
    def rot(self):
        val = bytes_to_int(self.state[19:21])
        rawtype = self.rawtype()
        return None if val == NULL_2BYTE else -val/10.0 if rawtype=='Q' else val/10.0 if rawtype=='q' else None
    def samples(self):
        return bytes_to_int(self.state[21:23])
    def __repr__(self):
        state = unpack_v23(self.state)
        return 'StateP(goal1=%s,dist=%s,type=%s,r=%s,goal2=%s,dist2=%s,rot=%s,samples=%d' % \
               (state.goal1(), state.dist(), state.type(), state.r(), state.goal2(), state.dist2(), state.rot(), state.samples())
    def pp(self):
        """
        Same as __repr__ but using self methods.
        """
        return 'StateP(goal1=%s,dist=%s,type=%s,r=%s,goal2=%s,dist2=%s,rot=%s,samples=%d' % \
               (self.goal1(), self.dist(), self.type(), self.r(), self.goal2(), self.dist2(), self.rot(), self.samples())


#   Gives the option of easily switching between a packed and unpacked representation.
#   Class based on namedtuple is simpler however consumes much more memory.
#   Note that the pickled file must be in the same representation selected here or loading it will fail.
#State = StateNamedtuple
State = StateP



#   Planning goal can be either north peg or northeast peg
NORTH = "north"
NORTHEAST = "northeast"



## Dimensions of robot and field components.
BUMPER = 3.5 # likely 3", but adding half-inch for safety margin
ROBOT_WIDTH = 27.5
ROBOT_LENGTH = 28.5
WW = ROBOT_WIDTH + 2*BUMPER  # robot width used in motion planning
LL = ROBOT_LENGTH + 2*BUMPER  # robot length used in motion planning

# Spec'd :  34.5 x 34.81
# Chassis Measured: 27.5w x 28.375long (sans bumpers)
# Estimated with 3.5" bumpers: 34.5" x 35.375"



##
##   CAD field measurements:
##
##   Distance between airship and Alliance Station (the ”north wall”): 144.142
##   - measured at Bellarmine practice field: 113"
NWALL_TO_AIRSHIP = 144.142

## (b) closest distance between airship and east wall, to left of drivers as they look onto field: 121.715 inches
##     - measured at Bellarmine practice field: 122.5"
##     - add 40" to get x => 162.5" = 13.54'
## (c) closest distance between airship and west wall: 121.965 inches
##     - add 40" to get x => 161.965
##     = 13.5'
CENTER_AIRSHIP_TO_WEST_WALL = 162
## (d) dimensions of the airship base (side lengths, angles, divider length): 38.375 inches, 120 degrees, 22 inches
## (e) are the corners of the field square or angled?  angled
## (f) distance from alliance wall to midfield line that gives the 5 point for autonomous crossing : 93.5 inches



# Goal peg on wall facing alliance station.
N_PEG_XY = (0, 0)
PEG_LENGTH = 10
N_PEG_LINE = ([0, 0], [0, PEG_LENGTH])  # Line representing goal peg, as (X, Y) = (x1,x2),(y1,y2)


# Divider between n airship wall and nw airship wall
DIVIDER2 = ([-19, -33], [0, 21]) # X, Y

# Divider between n airship wall and ne airship wall
DIVIDER3 = ([19, 33], [0, 21]) # X, Y

def xy_dist(p1, p2=(0, 0)):
    return np.linalg.norm(np.array(p1) - np.array(p2))

# Wall facing alliance station.
AIRSHIP_N_WALL = ([-19, 19],[0, 0]) # X,Y line, inches

# Wall facing alliance station.
AIRSHIP_S_WALL = ([-19, 19],[-70.6, -70.6]) # X,Y line, inches

# Northwest airship wall. This is the airship wall clockwise looking down with alliance station to north
AIRSHIP_NE_WALL = ([19, 40.035], [0,-35.295]) # X,Y line, eminating down at an angle from divider2.
AIRSHIP_NE_WALL_ANGLE = 1.047 # downward angle between wall3 and ray directed east from 0,0 (==radians(60))
AIRSHIP_NE_WALL_WIDTH = xy_dist((AIRSHIP_NE_WALL[0][0],AIRSHIP_NE_WALL[1][0],),(AIRSHIP_NE_WALL[0][1],AIRSHIP_NE_WALL[1][1],))

# Northwest airship wall. This is the airship wall counterclockwise looking down
AIRSHIP_NW_WALL = ([-19, -40.035], [0,-35.295]) # X,Y line, eminating down at an angle from divider1.
AIRSHIP_NW_WALL_ANGLE = -1.047 # downward angle between wall2 and ray directed east from 0,0 (==radians(-60))
AIRSHIP_NW_WALL_WIDTH = xy_dist((AIRSHIP_NW_WALL[0][0],AIRSHIP_NW_WALL[1][0],),(AIRSHIP_NW_WALL[0][1],AIRSHIP_NW_WALL[1][1],))

# Peg on northeast wall
NE_PEG_ANGLE = np.pi - AIRSHIP_NE_WALL_ANGLE # upward angle between ne peg and ray directed east from its base.
NE_PEG_TIP_XOFF =   PEG_LENGTH * np.sin(NE_PEG_ANGLE)   # y offset of ne peg tip relative to its own base
NE_PEG_TIP_YOFF = - PEG_LENGTH * np.cos(NE_PEG_ANGLE)   # x offset of ne peg tip relative to its own base
NE_PEG_XY = ((AIRSHIP_NE_WALL[0][0]+AIRSHIP_NE_WALL[0][1])/2.0, (AIRSHIP_NE_WALL[1][0]+AIRSHIP_NE_WALL[1][1])/2.0) # (x,y) of ne peg base
NE_PEG = ([NE_PEG_XY[0],NE_PEG_XY[0]+NE_PEG_TIP_XOFF], [NE_PEG_XY[1],NE_PEG_XY[1]+NE_PEG_TIP_YOFF]) # (X,Y) line for peg3

# Peg on northwest wall
NW_PEG_ANGLE = np.pi - AIRSHIP_NW_WALL_ANGLE # upward angle between peg2 and ray directed east from its base.
NW_PEG_TIP_XOFF =   PEG_LENGTH * np.sin(NW_PEG_ANGLE)   # y offset of peg2 tip relative to its own base
NW_PEG_TIP_YOFF = - PEG_LENGTH * np.cos(NW_PEG_ANGLE)   # x offset of peg2 tip relative to its own base
NW_PEG_XY = ((AIRSHIP_NW_WALL[0][0]+AIRSHIP_NW_WALL[0][1])/2.0, (AIRSHIP_NW_WALL[1][0]+AIRSHIP_NW_WALL[1][1])/2.0) # (x,y) of nw peg base
NW_PEG = ([NW_PEG_XY[0],NW_PEG_XY[0]+NW_PEG_TIP_XOFF], [NW_PEG_XY[1],NW_PEG_XY[1]+NW_PEG_TIP_YOFF]) # (X,Y) line for peg2

# NE Goal
NE_GOAL_XOFF =   (PEG_LENGTH/2 + ROBOT_LENGTH/2) * np.sin(NE_PEG_ANGLE)   # y offset of nw goal relative to its peg's base
NE_GOAL_YOFF = - (PEG_LENGTH/2 + ROBOT_LENGTH/2) * np.cos(NE_PEG_ANGLE)   # x offset of nw goal relative to its peg's base
NE_GOAL_X = NE_PEG_XY[0]+NE_GOAL_XOFF
NE_GOAL_Y = NE_PEG_XY[1]+NE_GOAL_YOFF

# Southwest airship wall.
AIRSHIP_SW_WALL = ([-40.035, -19], [-35.295, -70.6])

# Southwest airship wall.
AIRSHIP_SE_WALL = ([40.035, 19], [-35.295, -70.6])

# Bounding box of configuration space, inches.
LOWER_XLIMIT = -5
UPPER_XLIMIT = int(CENTER_AIRSHIP_TO_WEST_WALL)
N_LOWER_YLIMIT = -36
NE_LOWER_YLIMIT = -72
UPPER_YLIMIT = int(NWALL_TO_AIRSHIP)  # in dev, was: 12 * 6.5
N_BOUNDS  = {"xmin": LOWER_XLIMIT, "xmax": UPPER_XLIMIT,
             "ymin": N_LOWER_YLIMIT, "ymax": UPPER_YLIMIT}
NE_BOUNDS = {"xmin": LOWER_XLIMIT, "xmax": UPPER_XLIMIT,
             "ymin": NE_LOWER_YLIMIT, "ymax": UPPER_YLIMIT}

LOAD_STATION_WIDTH = 75.5
NE_CORNER_SIZE = np.sqrt(LOAD_STATION_WIDTH * LOAD_STATION_WIDTH / 2) # using for both sides tho boiler is smaller
NE_CORNER = ([UPPER_XLIMIT - NE_CORNER_SIZE, UPPER_XLIMIT], [UPPER_YLIMIT, UPPER_YLIMIT - NE_CORNER_SIZE])


DISCRETIZATIONS = 180
DISCRETIZATION_FACTOR = 360 / DISCRETIZATIONS


RADII = [35, 40, 5000, 45, 50, 55, 60, 65, 70]  # used only by plot bot, not for planning.
COLORS = ['y', 'b', 'c', 'm', 'k', 'w', 'g', 'r']
COLORS = COLORS + COLORS


#
#   The planning goal is to place the CENTER of the vehicle at a location and heading where
#   placement of gear onto peg is fairly easy. We want:
#   - the nose of the vehicle a bit less than mid-way along the peg,
#   - with (x,y) location within an inch left or right of center,
#   - with heading within 10 degrees of center.
#


#   Goal states and corresponding filepaths
#   We want the CENTER of the vehicle where it is able to place the gear just past mid peg
N_XY_GOALS = [(0, int(PEG_LENGTH/2 + LL/2 - 2),), (1, int(PEG_LENGTH/2 + LL/2 - 2),), (-1, int(PEG_LENGTH/2 + LL/2 - 2),), ]
NE_XY_GOALS = [(int(NE_GOAL_X), int(NE_GOAL_Y),),
               (int(NE_GOAL_X + 1.5*np.cos(NE_PEG_ANGLE)), int(NE_GOAL_Y + 1.1*np.sin(NE_PEG_ANGLE)),),
               (int(NE_GOAL_X - 1.5*np.cos(NE_PEG_ANGLE)), int(NE_GOAL_Y - 2.*np.sin(NE_PEG_ANGLE)))]

N_STAGE1_FILEPATH = 'n_stage1.pickle'
N_STAGE2_FILEPATH = 'n_stage2.pickle'
N_STAGE3_FILEPATH = 'n_stage3.pickle'
N_STAGE4_FILEPATH = 'n_stage4.pickle'
N_UNREACHABLES_FILEPATH = 'n_unreachables.pickle'

NE_STAGE1_FILEPATH = 'ne_stage1.pickle'
NE_STAGE2_FILEPATH = 'ne_stage2.pickle'
NE_STAGE3_FILEPATH = 'ne_stage3.pickle'
NE_STAGE4_FILEPATH = 'ne_stage4.pickle'
NE_UNREACHABLES_FILEPATH = 'ne_unreachables.pickle'

N_FILEPATHS  = ['Reserved', N_STAGE1_FILEPATH, N_STAGE2_FILEPATH, N_STAGE3_FILEPATH, N_STAGE4_FILEPATH]
NE_FILEPATHS = ['Reserved',NE_STAGE1_FILEPATH,NE_STAGE2_FILEPATH,NE_STAGE3_FILEPATH,NE_STAGE4_FILEPATH]


def make_arc(r, fc='y', a=0.9):
    """
    Create a circle for drawing.
    :param r: radius
    :param fc: face color
    :param a: alpha(translucency, lower is more transparent)
    """
    return pylab.Circle((r, 0), radius=r, alpha=a, fc=fc)


def arctan_trick(t):
    """
    Ensures that radian lies between -pi and pi
    """
    return np.arctan2(np.sin(t), np.cos(t))
assert(arctan_trick(3 * np.pi / 2) <= -np.pi / 2 and arctan_trick(3 * np.pi / 2) > -1.5707963268)


def prec(ff,pp=4):
    """
    Multiplies a number or tuple by a power of ten and truncates.
    :param ff: the number or tuple
    :param pp: precision
    :return: integer
    """
    if type(ff) is tuple:
        ll=[]
        for x in ff:
            ll.append(prec(x))
        return tuple(ll)
    else:
        return int(ff*pow(10,pp))


def radians_to360(angle):
    """
    Converts angle in radians to be non-negative degrees, allowing degrees > 360 if angle > 2pi
    """
    angle = np.degrees(angle)
    return angle + 360 if angle < 0 else angle
assert(radians_to360(2.1 * np.pi) == 378)



def heading_tangent_circle(x, y, x_center, y_center, clockwise=True):
    """
    Gets angle of a ray tangent to circle centered at (x_center, y_center) and pointed clockwise.
    Angle is measured counterclockwise from south.
    :param x: x coordinate of tangent point on the circle
    :param y: y coordinate of tangent point on the circle
    :param x_center: center of the circle
    :param y_center:
    :param clockwise: If False, rotate result by pi radians (180 degrees).
    :return: float radians.
    """
    if clockwise:
        return -arctan_trick(np.arctan2(x_center-x, y_center-y)+np.pi/2)
    else:
        return -arctan_trick(np.arctan2(x_center-x, y_center-y)+(3.0*np.pi/2))

assert(radians_to360(heading_tangent_circle(-1, 0, 0, 0)) == 180)
assert(prec(heading_tangent_circle(-1, 1, 0, 0)) == prec(3.0*np.pi/4.0))
assert(prec(heading_tangent_circle(0, 1, 0, 0)) == prec(np.pi/2))
assert(prec(heading_tangent_circle(1, 1, 0, 0)) == prec(np.pi/4))
assert(prec(heading_tangent_circle(1, 0, 0, 0)) == prec(0))
assert(prec(heading_tangent_circle(1, -1, 0, 0)) == prec(-np.pi/4))
assert(prec(heading_tangent_circle(0, -1, 0, 0)) == prec(-np.pi/2))
assert(prec(heading_tangent_circle(-1, -1, 0, 0)) == prec(-3.0*np.pi/4))

assert(prec(heading_tangent_circle(-1, 0, 0, 0,clockwise=False)) == prec(0))
assert(prec(heading_tangent_circle(-1, 1, 0, 0,clockwise=False)) == prec(-1.0*np.pi/4.0))
assert(prec(heading_tangent_circle(0, 1, 0, 0,clockwise=False)) == prec(-np.pi/2))
assert(prec(heading_tangent_circle(1, 1, 0, 0,clockwise=False)) == prec(-3.0*np.pi/4))
assert(radians_to360(heading_tangent_circle(1, 0, 0, 0,clockwise=False))) == 180
assert(prec(heading_tangent_circle(1, -1, 0, 0,clockwise=False)) == prec(3.*np.pi/4))
assert(prec(heading_tangent_circle(0, -1, 0, 0,clockwise=False)) == prec(np.pi/2))
assert(prec(heading_tangent_circle(-1, -1, 0, 0,clockwise=False)) == prec(1.0*np.pi/4))



def get_arrow(theta, scalar=None):
    """
    Returns (x,y) point.

    Points in direction of theta (south is zero, counterclockwise increasing)
    - theta=0 => points down
    - theta=pi/2 points right, theta=-(pi/2) points left
    - theta=pi points up.
    """
    scalar = 1.0 if not scalar else scalar
    return np.sin(theta) * scalar, -np.cos(theta) * scalar


assert(get_arrow(0) == (0.0, -1.0))  # down
assert(tuple(np.trunc(get_arrow(np.pi))) == (0, 1))  # up
assert(tuple(np.trunc(get_arrow(np.pi / 2))) == (1, 0))  # right
assert(tuple(np.trunc(get_arrow(-np.pi / 2))) == (-1, 0))  # left


def p_for_c_r(c, t, r, turntype="lh"):
    """
    Inverts center_for_tangent.
    (x,y) of a point at angle t (radians) on circle having center c and radius r

    WARNING: UNTESTED.
    If you're going to use it, please add asserts to prove it works correctly.
    Otherwise, use at your own risk.

    :param c:
    :param t:
    :param r:
    :param turntype:
    :return:
    """
    if turntype not in ['rh', 'lh']:
        logging.error("unsupported turn type: %s", turntype)
        return None
    cx,cy = c[0],c[1]
    if turntype == 'lh':
        px = cx - (r * np.cos(t))
        py = cy - (r * np.sin(t))
    elif turntype == 'rh':
        px = cx + (r * np.cos(t))
        py = cy + (r * np.sin(t))
    return (px,py)


def center_for_tangent(p, t_rad, r, turntype="lh"):
    """
    Gives the center of the circle tangent to a point on a ray having heading t (radians).
    turntype: lh=counterclockwise, rh=clockwise
    If turntype=rh, gives center to driver's right.
    If turntype=lh, gives center to driver's left.

    Arguments:
    p : tangent point (x,y) inches
    t : heading in RADIANS counterclockwise from South
    r : radius inches
    turntype : lh = clockwise, rh = counterclockwise
    """
    if turntype not in ['rh', 'lh']:
        logging.error("unsupported turn type: %s", turntype)
        return None
    if turntype == 'lh':
        cx = (r * np.cos(t_rad)) + p[0]
        cy = (r * np.sin(t_rad)) + p[1]
    elif turntype == 'rh':
        cx = -(r * np.cos(t_rad)) + p[0]
        cy = -(r * np.sin(t_rad)) + p[1]
    return (cx, cy)

# Following all use p=(0,0)
assert(center_for_tangent(p=(0, 0), t_rad=0, r=10, turntype="rh") == (-10, 0))  # Heading=South
assert(center_for_tangent(p=(0, 0), t_rad=0, r=10, turntype="lh") == (10, 0))  # Heading=S
assert(prec(center_for_tangent(p=(0, 0), t_rad=np.pi, r=10, turntype="rh")) == prec((10.0, 0.0))) # Heading=N
assert(prec(center_for_tangent(p=(0, 0), t_rad=np.pi, r=10, turntype="lh")) == prec((-10, 0)))  # Heading N

# Ray points west
assert(prec(center_for_tangent(p=(0,0), t_rad=np.pi / 2, r=10, turntype="rh")) == prec((0, -10)))
assert(prec(center_for_tangent(p=(0,0), t_rad=np.pi / 2, r=10, turntype="lh")) == prec((0, 10)))

# Ray points east
assert(prec(center_for_tangent(p=(0,0), t_rad=-np.pi / 2, r=10, turntype="rh")) == prec((0, 10)))
assert(prec(center_for_tangent(p=(0,0), t_rad=-np.pi / 2, r=10, turntype="lh")) == prec((0, -10,)))

## Following all use p=(10,10)
assert(center_for_tangent(p=(10, 10), t_rad=0, r=10, turntype="rh") == (0, 10))  # Heading=S, rh turn, circle is to east
assert(center_for_tangent(p=(10, 10), t_rad=0, r=10, turntype="lh") == (20, 10))  # Heading=S, lh turn, circle is to west

# 45 degrees pointing to bottom left
assert(prec(center_for_tangent(p=(10, 10), t_rad=-np.pi / 4, r=10, turntype="rh")[0], 8) == prec(2.9289321881345245, 8))
assert(prec(center_for_tangent(p=(10, 10), t_rad=-np.pi / 4, r=10, turntype="rh")[1], 8) == prec(17.071067811865476, 8))

# 45 degrees pointing to bottom left
assert(prec(center_for_tangent(p=(10, 10), t_rad=-np.pi / 4, r=10, turntype="rh")[0], 8) == prec(2.9289321881345245, 8))
assert(prec(center_for_tangent(p=(10, 10), t_rad=-np.pi / 4, r=10, turntype="rh")[1], 8) == prec(17.071067811865476, 8))
assert(prec(center_for_tangent(p=(10, 10), t_rad=-np.pi / 4, r=10, turntype="lh")[0], 8) == prec(17.071067811865476, 8))
assert(prec(center_for_tangent(p=(10, 10), t_rad=-np.pi / 4, r=10, turntype="lh")[1], 8) == prec(2.9289321881345245, 8))

# # 45 degrees pointing to top right
assert(prec(center_for_tangent(p=(10, 10), t_rad=3 * np.pi / 4, r=10, turntype="rh")[0], 8) == prec(17.071067811865476, 8))
assert(prec(center_for_tangent(p=(10, 10), t_rad=3 * np.pi / 4, r=10, turntype="rh")[1], 8) == prec(2.9289321881345245, 8))
assert(prec(center_for_tangent(p=(10, 10), t_rad=3 * np.pi / 4, r=10, turntype="lh")[0], 8) == prec(2.9289321881345245, 8))
assert(prec(center_for_tangent(p=(10, 10), t_rad=3 * np.pi / 4, r=10, turntype="lh")[1], 8) == prec(17.071067811865476, 8))



def get_heading(p1, p2=(0, 0)):
    """
    Gets heading of ray originating at p1 and pointed at p2.
    Measured from south increasing counterclockwise.

    - trivial case when p1 equals p2 returns zero.
    - p1 above p2 returns zero
    - p1 below p2 returns pi
    - p1 east of p2 returns pi/2
    - p1 west of p2 returns -pi/2
    """
    if p1==p2:
        return 0
    return arctan_trick(np.arctan2(p1[1] - p2[1],p1[0] - p2[0])-np.pi/2)

assert(get_heading((0, 0,))  == 0)   # trivial case p1 == origin => no angle
assert(get_heading((0, 10,)) == 0)          # p1->p2 points south
assert(get_heading((10, 0,)) == -np.pi/2)   # p1->p2 points west
assert(get_heading((0, -10,)) == -np.pi)    # p1->p2 points north
assert(get_heading((-10, 0,)) == np.pi/2)   # p1->p2 points east

assert(get_heading((0, 1,),(0,0)) == 0)         # p1->p2 points south
assert(get_heading((1, 0,),(0,0)) == -np.pi/2)  # p1->p2 points west
assert(get_heading((0, -1,),(0,0)) == -np.pi)   # p1->p2 points north
assert(get_heading((-1, 0,),(0,0)) == np.pi/2)  # p1->p2 points east

assert(prec(get_heading((1, 1,),(0,0)),8) == prec(-np.pi/4,8)) # p1->p2 points southwest
assert(get_heading((1, -1,),(0,0)) == -3*np.pi/4)   # p1->p2 points northwest
assert(prec(get_heading((-1, -1,),(0,0)),8) == prec(3.0/4.0*np.pi,8))  # p1->p2 points northeast
assert(prec(get_heading((-1, 1,),(0,0)),8) == prec(np.pi/4,8))  # p1->p2 points southeast


MAXD=1000 # maximum arc length on this playing field
def arc_dist(p1, p2, r, h, turntype='lh', tol=0.5, debug=False, maxd=MAXD):
    """
    p1,p2 : (x,y)
    r : radius
    h : heading, radians, counterclockwise from south.
    turntype : lh=counterclockwise, rh=clockwise

     Given p1, p2, radius r, and heading, calculate length of arc circle path from p1 to p2
     Two cases:
     (a) p2 is on the same circle. Distance is the length of the arc in the direction of travel
     (b) p2 is not on the circle: distance is d_a plus distance from point to the circle.

    Some edge cases give a negative distance, such as when the offset is negative and exceeds the arc distance.
    This can also blow up in degenerate cases at long radius and short distance, which are guarded by <maxd>.

    Returns: 2-tuple
    - arc distance
    - radius offset: negative if inside circle, positive if outside

    Returns (None, None) in following cases:
    - unrecognized turn type.
    - xy distance between the two points exceeds 2 * r + tol
    """
    if turntype not in ['lh','rh']:
        logging.error("ERROR: unsupported type: %s", turntype)
        return None, None

    # sanity check.=
    if xy_dist(p1, p2) > 2 * r + tol:
        logging.info("ERROR: d_12 (%.1f) > 2r (%0.1f) ", xy_dist(p1, p2), r)
        return None, None

    h = arctan_trick(h)
    h_deg = np.degrees(h)
    if debug:
        logging.info("p1:     (%.3f,%.3f)", p1[0],p1[1])
        logging.info("p2:     (%.3f,%.3f)", p2[0],p2[1])
        logging.info("turn:   %s", turntype)
        logging.info("h:      %.2f (pi)  %d (degrees)", h / np.pi, h_deg)
        logging.info("r:      %d", r)
        logging.info("circum: %.1f", 2 * np.pi * r)

    # Get center of turning radius r for ray originating from p1 at radians heading h and turning direction
    c = center_for_tangent(p1, h, r, turntype=turntype)

    t_c_p1 = radians_to360(get_heading(c, p1))%360  # heading of ray pointing from c to  p1
    t_c_p2 = radians_to360(get_heading(c, p2))%360  # heading of ray pointing from c to  p2

    if debug:
        logging.info("c:        %s", c)
        logging.info("t(c->p1): %d (degrees)", t_c_p1)
        logging.info("t(c->p2): %d (degrees)", t_c_p2)

    if turntype == 'rh':  # clockwise
        if t_c_p1 > t_c_p2:
            t_12 = t_c_p1 - t_c_p2
        else:
            t_12 = (t_c_p2 - t_c_p1)%360
        if debug:
            logging.info("t(p2->p1): %d (degrees)", t_12)
    elif turntype == 'lh':  # counterclockwise
        if t_c_p1>t_c_p2:
            t_12 = 360-(t_c_p1-t_c_p2)
        else:
            t_12 = abs(t_c_p2+360-t_c_p1)%360
        if debug:
            logging.info("t(p1->p2): %d (degrees)", t_12)

    dist = 2*np.pi*r*t_12/360.0
    r2 = xy_dist(c, p2)
    if dist > maxd:
        return None, None
    return dist, r2 - r


# Heading=300, lh
assert(prec(arc_dist((-1.5,2.65625),(2.64,-1.54),3.03225,np.radians(300),turntype='lh'),8) == prec((11.066241813119897, 0.025590366864600256),8))
# # Heading=120, rh
assert(prec(arc_dist((-1.5,2.65625),(2.64,-1.54),3.03225,np.radians(120),turntype='rh'),8) == prec((7.9859468345753992, 0.0255903668646007),8))
# # # inside circle
assert(prec(arc_dist((-1.5,2.65625),(2.6,-1.54),3.03225,np.radians(120),turntype='rh'),8) == prec((8.0065464136041165, -0.0086631053950783077)))
# # # outside circle"
assert(prec(arc_dist((-1.5,2.65625),(2.7,-1.54),3.03225,np.radians(120),turntype='rh'),8) == prec((7.9559007176690093, 0.077227883701158184)))
# # # sums 2-tuple values
assert(prec(sum(arc_dist((2.64, -1.54),(-1.5, 2.65625), 3.03225,h=np.radians(150),turntype='lh')),8)==prec(7.9558773531677716,8))

# ##
# ## Swaps points used above
# ##
# -30, rh
assert(prec(arc_dist((2.64, -1.54),(-1.5, 2.65625),3.03225,np.radians(-30),turntype='rh'),8) == prec((7.9099380690395202, 0.045939284128249991),8))
# #
# # 150, lh
assert(prec(arc_dist((2.64, -1.54),(-1.5, 2.65625),3.03225,np.radians(150),turntype='lh'),8) == prec((7.9099380690395211, 0.045939284128249991),8))
# #
# # inside circle : radius offset should be negative
assert(prec(arc_dist((2.64, -1.54),(-1.5, 2.6),3.03225,np.radians(150),turntype='lh'),8) == prec((7.9376313590248184, -0.0029102585840607986),8))

# center @ (0,0), r=1
assert(prec(arc_dist(p1=(-1,0,),p2=(1,0), r=1, h=0, turntype='lh', tol=0.1),8) == prec((np.pi,0,),8))
assert(prec(arc_dist(p1=(-1,0,),p2=(1,0), r=1, h=np.pi, turntype='rh', tol=0.1),8) == prec((np.pi,0,),8))
assert(prec(arc_dist(p1=(1,0,),p2=(-1,0), r=1, h=0, turntype='rh', tol=0.1),8) == prec((np.pi,0,),8))
assert(prec(arc_dist(p1=(1,0,),p2=(-1,0), r=1, h=np.pi, turntype='lh', tol=0.1),8) == prec((np.pi,0,),8))
assert(prec(arc_dist(p1=(-1,0,),p2=(0,-1), r=1, h=0, turntype='lh', tol=0.1),8) == prec((np.pi/2,0,),8))
assert(prec(arc_dist(p1=(-1,0,),p2=(0,1), r=1, h=np.pi, turntype='rh', tol=0.1),8) == prec((np.pi/2,0,),8))
assert(prec(arc_dist(p1=(1,0,),p2=(0,-1), r=1, h=0, turntype='rh', tol=0.1),8) == prec((np.pi/2,0,),8))
assert(prec(arc_dist(p1=(1,0,),p2=(0,1), r=1, h=np.pi, turntype='lh', tol=0.1),8) == prec((np.pi/2,0,),8))


CW_ARC_TOL = 2.01 # 1"x1" grid discretization can contribute up to 2" of discrepancy to the arc length calculation.
def cw_arclength(p1, p2, r, tol=CW_ARC_TOL, debug=False):
    """
    Returns the circular arc length in inches from p1 to p2 along arc of curvature r,
    as float inches to one decimal precision.

    Assumes that points are within 180 degrees of each other.

    Takes the shortest path, which is why it doesn't require a heading or turn direction.

    Arguments:
    - p1 : (x,y)
    - p2 : (x2,y2)
    - tol : allows points that are within d=2r+tol to still return

    Returns: None if error or out of tolerance, otherwise a float.
    """
    if p1 == p2:
        return 0
    err = 0.0
    d = xy_dist(p1, p2) # cartesian beeline distance
    if d > 2 * r + tol:
        if debug or d > 2*r + (2 * tol) :
            logging.error("xy dist (p1,p2) = %.2f, which is > 2*r + tol for radius %.2f,  ", d, r)
        return None
        err = 2*r - d  # hack to handle measurement error due to discretization
    d = 2*r if d > 2*r else d
    try:
        xx = (2 * (r * r) - d * d) / (2.0 * r * r)
        xx = 1.0 if xx > 0.999999 else -1 if xx < -0.999999 else xx # avoid math domain error due to rounding
        rads = acos(xx)  # <==TODO dcheck rm when done testing
    except ValueError as e:
        # if debug:
        logging.error("%s in cw_arclength: p1:(%d,%d), p2:(%d,%d), r:%.1f ",
                         e, p1[0], p1[1], p2[0], p2[1], r)
        return None
    arclength = r * rads + err
    assert(arclength <= np.pi * r + tol)  # maximum length is circumference/2
    return arclength
assert(cw_arclength((0, 0), (2, 0), 1) == np.pi)
assert(cw_arclength((0, 2), (0, 0), 1) == np.pi)
assert(cw_arclength((2, 2), (4, 2), 1) == np.pi)
assert(round(100.0 * cw_arclength((0, 0), (2, 0), 1)) / 100.0 == 3.14)
assert(round(100.0 * cw_arclength((0.0001, 0.0001), (1.999, 0.0001), 1)) / 100.0 == 3.08)
assert (prec(cw_arclength((2.64, -1.54), (-1.5, 2.65625), 3.03225, tol=0.4),8)==prec(8.08785331109,8))



def discretize_angle_deg(t_deg, factor=DISCRETIZATION_FACTOR):
    """
    Bounds an angle in degrees to lie in [0,359]
    :param t_deg: angle in degrees
    :return: integer between 0 and 359
    """
    t_deg = int(round(t_deg / factor) * factor)
    return (t_deg + 360 if t_deg < 0 else t_deg) % 360


def discretize_angle(t_rad, bound=True, factor=DISCRETIZATION_FACTOR):
    """
    Converts angle in radians to degree, ensures non-negative and discretizes.
    Arguments:
    - bound : ensures within (0,359)
    - factor : degree stepsize, = 360/discretizations, ideally a factor of 360
    """
    t_deg = int(round(np.degrees(t_rad) / factor) * factor)
    t_deg = t_deg + 360 if t_deg < 0 else t_deg
    return t_deg % 360 if bound else t_deg
assert(discretize_angle(0) == 0)
assert(discretize_angle(np.pi, factor=6) == 180)
assert(discretize_angle(2 * np.pi, False, factor=6) == 360)
assert(discretize_angle(3 * np.pi, False, factor=6) == 540)
assert(discretize_angle(4 * np.pi, False, factor=6) == 720)
assert(discretize_angle(2 * np.pi, factor=6) == 0)
assert(discretize_angle(3 * np.pi, factor=6) == 180)
assert(discretize_angle(4 * np.pi, factor=6) == 0)
assert(discretize_angle(-2 * np.pi, factor=6) == 0)
assert(discretize_angle(-2 * np.pi, factor=6) == 0)
assert(discretize_angle(0.49999 * np.pi, factor=6) == 90)
assert(discretize_angle(0.50001 * np.pi, factor=6) == 90)
assert(discretize_angle(np.pi / 4, factor=6) == 48)
assert(discretize_angle(0.52 * np.pi, factor=6) == 96)
assert(discretize_angle(0.52 * np.pi, factor=3) == 93)


#   Goal headings
DISCRETIZED_GOAL_HEADINGS = sorted(set([discretize_angle(angle) for angle in np.arange(0, 2 * np.pi, 0.01)]))
DGH = DISCRETIZED_GOAL_HEADINGS
N_GOAL_HEADINGS = [0, DGH[-3], DGH[3], DGH[-5], DGH[5], DGH[-7], DGH[7]]
NE_GOAL_HEADINGS = []
for hh in N_GOAL_HEADINGS:
    NE_GOAL_HEADINGS.append(discretize_angle_deg(hh-60))



def get_xy_explored(config2):
    """
    Set of xy locations that have a path for some heading.
    """
    return set([(x, y) for x, y, z in config2.iterkeys()])


def get_xy_unexplored(whichpeg, config2, unreachables):
    """
    Set of xy locations that don't have a path for any heading.
    """
    all_xy = xy_grid_set(whichpeg) - unreachables
    explored = get_xy_explored(config2)
    return set([k for k in all_xy if k not in explored])

def unreachables_path_for_peg(whichpeg):
    if whichpeg == NORTH:
        return N_UNREACHABLES_FILEPATH
    elif whichpeg == NORTHEAST:
        return NE_UNREACHABLES_FILEPATH


def get_xy_unexplored_file(whichpeg, planpath):
    explored = load_config(planpath)
    unreachables = load_unreachables(whichpeg)
    xlim,ylim = lower_xylim_for_peg(whichpeg)
    return get_xy_unexplored(whichpeg, explored, unreachables, xlim, ylim)


def get_xyt_explored(config2):
    """
    Set of states that have a path.
    """
    return set(config2.iterkeys())


def get_xyt_unexplored(whichpeg, states, unreachables):
    """
    Set of reachable xyt states that don't have a path.
    """
    all_xyt = xyt_grid_set(whichpeg)
    explored = get_xyt_explored(states)
    unexplored = set([k for k in all_xyt if k not in explored])
    if unreachables:
        return unexplored - unreachables

    else:
        logging.warn("Unreachables set not provided, result may contain unreachable states.")
        return unexplored


def get_xyt_unexplored_file(whichpeg, stage=None):
    explored = load_config(configpath_for_stage(whichpeg, stage))
    explored = set(explored.iterkeys())
    unreachables = load_unreachables(whichpeg)
    return explored - unreachables


def get_remaining(whichpeg, plan_states, unreachables, max=20000):
    reachable_grid = get_reachable_grid(whichpeg, unreachables)
    num_reachable = len(reachable_grid)
    logging.info("Total number of reachable states:      %8d", num_reachable)

    remaining = reachable_grid - get_xyt_explored(plan_states)
    num_missing = len(remaining)
    logging.info("Number reachable states lacking path:  %8d", num_missing)
    logging.info("Percent reachable states having path:     %6.2f", (100.0 - (100.0 * num_missing / num_reachable)))

    if len(remaining) > max:
        logging.info("Saving only %d/%d", max, len(remaining))
        remaining = set(random.sample(remaining,max))
    return remaining


N_REMAINING = "n_remaining.pickle"
NE_REMAINING = "ne_remaining.pickle"
def save_remaining(whichpeg):
    config = config_for_stage(whichpeg, 4)
    unreachables = load_unreachables(whichpeg)
    remaining = get_remaining(whichpeg, config, unreachables)
    path = N_REMAINING if whichpeg==NORTH else NE_REMAINING if whichpeg==NORTHEAST else None
    save_pickle(remaining,path)


def get_xyt_fillable(whichpeg, explored, unreachables):
    """
    Returns list of keys for reachable xyt states that do not yet have an entry for this heading,
    but do have an entry for some other heading.
    """

    #    Gets the xyt states that do not have a state.
    logging.info("Calculating unexplored")
    xyt_fillable = get_xyt_unexplored(whichpeg, explored, unreachables)
    logging.info("Number of xyt unexplored: %d", len(xyt_fillable))

    #    Gets the xy cells that do have an entry for some configuration
    xy_filled = get_xy_explored(explored)

    #    Get xyt states that have a corresponding xy in the explored states.
    return [k for k in xyt_fillable if (k[0],k[1]) in xy_filled]


def get_xyt_fillable_file(whichpeg, planpath, unreachables_path, logonly=False):
    """
    Returns list of keys for reachable xyt states that do not yet have an entry for this heading,
    but do have an entry for some other heading.  Note: use stage2 config.
    """
    explored = load_config(planpath)
    unreachables = load_unreachables(unreachables_path)

    #    Get xyt states that have a corresponding xy in the explored states.
    xyt_fillable = get_xyt_fillable(whichpeg, explored, unreachables)
    logging.info("Number of xyt fillable:  %d", len(xyt_fillable))

    if not logonly:
        return xyt_fillable



def get_reachable_grid(whichpeg, unreachables):
    return xyt_grid_set(whichpeg) - unreachables


def bin_unexplored_xy(whichpeg, config2, unreachables):
    """
    For each xy location gives count of how many states don't have a path.
    """
    binned = defaultdict(int)
    unexplored = get_xyt_unexplored(whichpeg, config2, unreachables)
    for k in unexplored:
        binned[(k[0], k[1],)] += 1
    return binned


def deltap_onheading(state,d,discretization_factor=DISCRETIZATION_FACTOR):
    """
    Calculates a new state from the current one
    driving either directly forwards, if d>0,
    or in reverse, if d<0.

    New state location and heading is discretized.

    :param state:
    :param d: distance
    :return: state
    """
    x,y,h = state[0],state[1],state[2]
    dx = d * np.sin(np.radians(h))
    dy = -d * np.cos(np.radians(h))
    return (int(x + dx), int(y + dy), discretize_angle_deg(state[2], discretization_factor))

assert(deltap_onheading((0,0,0),10, 1)==(0, -10, 0))
assert(deltap_onheading((0,0,90),10, 1)==(10, 0, 90))
assert(deltap_onheading((0,0,45),10, 1)==(7, -7, 45))
assert(deltap_onheading((0,0,135),10, 1)==(7, 7, 135))
assert(deltap_onheading((0,0,180),10, 1)==(0, 10, 180))
assert(deltap_onheading((0,0,225),10, 1)==(-7, 7, 225))
assert(deltap_onheading((0,0,270),10, 1)==(-10, 0, 270))
assert(deltap_onheading((0,0,315),10, 1)==(-7, -7, 315))
assert(deltap_onheading((0,0,360),10, 1)==(0, -10, 0))
assert(deltap_onheading((10,10,0),10, 1)==(10, 0, 0))
assert(deltap_onheading((20,20,-45),28.284, 1)==(0, 0, 315))

assert(deltap_onheading((0,0,0),-10, 1)==(0, 10, 0))
assert(deltap_onheading((0,0,90),-10, 1)==(-10, 0, 90))
assert(deltap_onheading((0,0,45),-10, 1)==(-7, 7, 45))
assert(deltap_onheading((0,0,135),-10, 1)==(-7, -7, 135))
assert(deltap_onheading((0,0,180),-10, 1)==(0, -10, 180))
assert(deltap_onheading((0,0,225),-10, 1)==(7, -7, 225))
assert(deltap_onheading((0,0,270),-10, 1)==(10, 0, 270))
assert(deltap_onheading((0,0,315),-10, 1)==(7, 7, 315))
assert(deltap_onheading((0,0,360),-10, 1)==(0, 10, 0))
assert(deltap_onheading((10,10,0),-10, 1)==(10, 20, 0))
assert(deltap_onheading((20,20,135),-28.284, 1)==(0, 0, 135))



def line_grid_ahead(whichpeg, state, obstacles=None, verbose=False):
    """
    Returns a grid of (x,y) points directly ahead of robot,
    up until first obstacle or out of bounds.
    :param p: (x,y)
    :param h: heading, degrees
    :param obstacles: use xy_obstacles_set()
    :param verbose:
    :return:
    """
    if whichpeg==NORTH:
        bounds = N_BOUNDS
    elif whichpeg==NORTHEAST:
        bounds = NE_BOUNDS
    else:
        ValueError("Unsupported choice of peg.")
    grid=[]
    max=1000
    x,y,h = state[0],state[1],state[2]
    for d in range(1,max):
        dx =  d * np.sin(np.radians(h))
        dy = -d * np.cos(np.radians(h))
        pp3 = (int(x+dx),int(y+dy), h)
        if is_out_of_bounds((pp3[0],pp3[1],), bounds):
            break
        if obstacles and collides(pp3, obstacles):
            break
        if pp3 not in grid:
            grid.append(pp3)
            if verbose:
                logging.info("%s", pp3)
    if len(grid) == max:
        logging.error("Exhausted grid limit.")
    return grid



def line_grid_behind(whichpeg, state, obstacles=None, verbose=False):
    """
    Returns a grid of (x,y) points directly behind robot,
    up until first obstacle or out of bounds.
    :param p: (x,y)
    :param h: heading, degrees
    :param obstacles: use xy_obstacles_set()
    :param verbose:
    :return:
    """
    if whichpeg==NORTH:
        bounds = N_BOUNDS
    elif whichpeg==NORTHEAST:
        bounds = NE_BOUNDS
    else:
        ValueError("Unsupported choice of peg.")
    grid=[]
    max=1000
    x,y,h = state[0],state[1],state[2]
    for d in range(1,max):
        dx = -d * np.sin(np.radians(h))
        dy = d * np.cos(np.radians(h))
        pp3 = (int(x+dx),int(y+dy),h,)
        if is_out_of_bounds((pp3[0],pp3[1],), bounds):
            break
        if obstacles and collides(pp3, obstacles):
            break
        if pp3 not in grid:
            grid.append(pp3)
            if verbose:
                logging.info("%s", pp3)
    if len(grid) == max:
        logging.error("Exhausted grid limit.")
    return grid

#====================================================================
#   robot and obstacles
#====================================================================

def get_robot_rear(nose, theta):
    return (nose[0] - (LL * np.sin(theta)), nose[1] + (LL * np.cos(theta)))  # rear midpoint

def get_robot_center(nose, theta):
    return (nose[0] - (LL/2 * np.sin(theta)), nose[1] + (LL/2 * np.cos(theta)))

def get_robot_nose_from_center(center, theta):
     return (center[0]+(LL/2 * np.sin(theta)), center[1]-(LL/2 * np.cos(theta)),)

def int_tup(tup):
    return map(int, tup)

def get_robot_corners(nose, theta, rear, grid=False):
    xy_fcdl = [nose[0] + (WW / 2 * np.cos(theta)), nose[1] +
               (WW / 2 * np.sin(theta))]  # front driver left
    xy_fcdr = [nose[0] - (WW / 2 * np.cos(theta)), nose[1] -
               (WW / 2 * np.sin(theta))]  # front driver right
    xy_rcdl = [rear[0] + (WW / 2 * np.cos(theta)), rear[1] +
               (WW / 2 * np.sin(theta))]  # rear driver left
    xy_rcdr = [rear[0] - (WW / 2 * np.cos(theta)), rear[1] -
               (WW / 2 * np.sin(theta))]  # rear driver right

    if grid:
        return int_tup(xy_fcdl), int_tup(xy_fcdr), int_tup(xy_rcdl), int_tup(xy_rcdr)
    else:
        return xy_fcdl, xy_fcdr, xy_rcdl, xy_rcdr


def get_robot_chassis(xy_fcdl, xy_fcdr, xy_rcdl, xy_rcdr):
    """
    :param xy_fcdl: front corner driver left
    :param xy_fcdr: front corner driver right
    :param xy_rcdl: rear corner driver left
    :param xy_rcdr: rear corner driver right
    :return:
    """
    line_front = ([xy_fcdr[0], xy_fcdl[0]], [xy_fcdr[1], xy_fcdl[1]])
    line_rear = ([xy_rcdr[0], xy_rcdl[0]], [xy_rcdr[1], xy_rcdl[1]])
    line_dleft = ([xy_fcdl[0], xy_rcdl[0]], [xy_fcdl[1], xy_rcdl[1]])
    line_dright = ([xy_fcdr[0], xy_rcdr[0]], [xy_fcdr[1], xy_rcdr[1]])

    return line_front, line_rear, line_dleft, line_dright


def rtpairs(r, n):
    """
    Generates points evenly distributed on a circle centered at origin.
    r : radius
    n : number of points to generate
    """
    for j in range(n):
        yield r, j * (2 * np.pi / n)


def sample_check(config):
    """
    Returns the number of items in the dictionary having less than two samples.

    - config : a dictionary containing the field 'samples' for each item.
    """
    undersampled = 0
    for key in config.iterkeys():
        if config[key].samples() < 2:
            undersampled += 1
    return undersampled


def is_out_of_bounds(p, bounds):
    """
    Checks whether a (x,y) point is in 2D bounds.
    :param p:
    :param bounds:
    :return:
    """
    xmin = bounds.get("xmin")
    xmax = bounds.get("xmax")
    ymin = bounds.get("ymin")
    ymax = bounds.get("ymax")

    if p[0] > xmax:
        logging.debug("Out of bounds (%.2f, %.2f), x > xmax (%s)", p[0], p[1], xmax)
        return True
    if p[0] < xmin:
        logging.debug("Out of bounds (%.2f, %.2f), x < xmin (%s)", p[0], p[1], xmin)
        return True
    if p[1] < ymin:
        logging.debug("Out of bounds (%.2f, %.2f), y < ymin (%s)", p[0], p[1], ymin)
        return True
    if p[1] > ymax:
        logging.debug("Out of bounds (%.2f, %.2f), y > ymax (%s)", p[0], p[1], ymax)
        return True
    return False


def bound_angle(angle):
    angle = angle + 360 if angle < 0 else angle
    return angle % 360


#====================================================================
#   planning stuff
#====================================================================

def backwards_arc(whichpeg, goal1, r, turntype='lh',
                  gridsize=1000,  # in practice use adaptive
                  verbose=False,
                  debug=False,
                  config=None,
                  goal2=None,
                  dist2=None,
                  stage2=False,
                  tol=CW_ARC_TOL,
                  obstacles=None):
    """
    Performs backwards planning step, searching from goal state defined by goal1 and goal_heading,
    along an arc of radius r.

    The direction of the turn is with respect to the vehicle as when actually following the path forwards.

    Returns a dictionary of (x,y,h,) => {'samples':<int>, 'r' : <float>, dist":<float>},
    where 'h' is the heading in degrees measured counterclockwise from South, and
    'dist' is distance from (x,y) to (goal1[0],goal1[1]) along an arc of radius r from a ray having heading goal[2]

    The dist field of key k contains the total distance from state k to a primary goal state regardless of whether
    it is a stage1 path or stage2 path.

    Each stage2 path contains fields dist1 and goal2
    - dist1 field gives the (x,y) distance dist(goal1, goal2)
    - dist field gives dist(p,goal1) + dist(goal1,goal2).

    If goal2 and dist2 are provided, then the table entry for state goal1 gives a path
    leading to the primary goal goal2 with dist2 = distance(goal1, goal2).

    :param goal1: (x,y,h), the goal for this path, giving the (x,y) location and desired goal heading h
    :param r: turning radius
    :param turntype: 'lh' (counterclockwise) or 'rh' (clockwise),
    :param gridsize: number of points to explore along the arc; use small multiple (6 or 12) times radius.
    :param verbose: logs gridsize and number of states stored
    :param debug: logs when better path found
    :param bounds: dict containing fields xmin,xmax,ymin,ymax for excluding points
    :param config: state configuration dictionary which will be appended with result
    :param goal2: primary goal state (x,y,t). Required if called with stage2==True
    :param dist2: the cartesian distance (goal1, goal2). Required if called with stage2==True
    :param stage2: if True, utilizes tight turns to locate stage 1 paths.
    :param tol: tolerates some error in distance calculation due to discretization
    :param obstacles: set of (x,y,t)
    :return:
    """
    if whichpeg==NORTH:
        bounds = N_BOUNDS
    elif whichpeg==NORTHEAST:
        bounds = NE_BOUNDS
    else:
        ValueError("Unsupported choice of peg.")

    if (stage2 and (not goal2 or not dist2)) or (goal2 and not dist2) or (dist2 and not goal2) :
        logging.error("Stage2 requires arg goal2 and dist2 be specified. Skipping %s", goal1)
        return config

    ghrad = np.radians(goal1[2]) # goal1 heading, radians
    config = config if config else {}
    center = center_for_tangent((goal1[0],goal1[1]), ghrad, r, turntype)

    if stage2:
        amin, amax = fdl_rotation(goal1[2], turntype) # Explores semicircle
        # Tightens up exploration arc so that initial point is closer to goal1
        amin += 0.02 * np.pi
        amax += 0.02 * np.pi
    else:
        # amin = (3.0 / 2.0) * np.pi # explore 75% of the arc, rely on unreachability to eliminate out of bounds etc
        # amax = (5.0 / 2.0) * np.pi
        amin, amax = fdl_rotation(goal1[2], turntype, portion=0.75)

    points = arc_grid(r, gridsize, center, amin, amax, counter=True if turntype == 'rh' else False)

    count = 0
    stored = 0
    x_c, y_c = center[0], center[1]
    max_debug = 100
    debugs = 0
    max_better = 50
    betters = 0
    out_i = 0
    out_lim = 5
    collision_i = 0
    collision_lim = 2
    for point in points:

        if is_out_of_bounds(point, bounds):
            out_i += 1
            if out_i > out_lim :
                #   Prune search
                break
            else:
                continue

        x = int(point[0])
        y = int(point[1])
        if x == goal1[0] and y == goal1[1]:
            continue
        if xy_dist((goal1[0],goal1[1]),(x,y)) < 0.5 :
            continue

        h = discretize_angle(heading_tangent_circle(x, y, x_c, y_c, clockwise=True if turntype=='rh' else False))
        key = (x, y, h,)

        #   Don't continue along this arc once it collides backing from goal to avoid "tunneling".
        #   We don't want to run towards that obstacle on other side of it.
        #   This also prunes unnecessary search.
        #   We tolerate a few touches, e.g. as it backs away from a goal at a new angle
        if obstacles and collides(key, obstacles):
            collision_i += 1
            if collision_i > collision_lim:
                break
            else:
                continue


        count += 1
        dist = cw_arclength((goal1[0],goal1[1]), (x, y,), r, tol=tol)

        #   Handles a discretization error affecting points that are relatively close in Euclidean distance
        #   along an arc of long radius
        if dist == 0 :
            xyd = xy_dist((goal1[0],goal1[1]), (x, y,))
            if xyd < tol:
                dist = xyd


        if not dist:
            if verbose or r < 100 :
                logging.error(
                    "cw_arclength==0 for (x,y)=%s, goal1=%s using radius %.1f for tol %.1f, h=%.1f, turn=%s",
                    (x, y,), (goal1[0],goal1[1],), r, tol, h, turntype)
            distArc = arc_dist((goal1[0],goal1[1]), (x, y,), r, h, turntype, tol=2*tol)
            dista = -1 if distArc == (None,None) else sum(distArc)
            if dista < 0: # usually due to points nearby on the arc but with one inside the circle
                ##  If they're that close they're practically identical in this discretized space
                if verbose or r < 100:
                    logging.error("Skipping. arc_dist: %.1f", dista)
                continue
            else:
                dist = dista

        if debug:
            debugs += 1
            if debugs < max_debug:
                logging.debug(
                    "Goal1: %s  R:%.1f (x,y):(%.1f,%.1f) heading: %.1f  Dist: %.1f", goal1, r, x, y, h, dist)

        if not goal2:  # state for key=goal1 gives direct path to end goal state.
            if key in config and config[key].dist():
                vv = config[key]
                if dist < config[key].dist():
                    if betters < max_better:
                        logging.debug("\tBetter path for %s/%s : WAS r=%.1f,d=%.3f, NOW r=%.1f, dist=%.3f",
                                         goal1, key, config[key].r(), config[key].dist(), r, dist)
                    config[key] = State(vv.goal1(), dist, turntype, r, samples=vv.samples() + 1)
                    betters += 1
                else:
                    config[key] = State(vv.goal1(),vv.dist(),vv.type(),vv.r(),samples=vv.samples()+1)
            else:
                config[key] = State(goal1,dist,turntype,r)
                stored += 1

        else:  # second stage
            if key not in config:
                config[key] = State(goal1,dist+dist2,turntype,r,goal2,dist)
                stored += 1
            elif is_stage1(config[key]) :
                # prefer stage1 path to stage2
                pass
            else:
                if config[key].dist() > dist + dist2:
                    prev_samples = config[key].samples()
                    if betters < max_better:
                        logging.debug("\tBetter stage2 path for %s : WAS r=%.1f, d=%.1f, type=%s NOW r=%.1f, dist=%.1f, type=%s",
                                 goal1, config[key].r(), config[key].dist(), config[key].type(), r,  dist+dist2, turntype)
                    config[key] = State(goal1, dist+dist2, turntype, r, goal2, dist, samples=prev_samples+1)
                    betters += 1
                    stored += 1
    if betters:
        logging.debug("States that were improved: %d/%d", betters, count)
    logging.debug("Gridsize: %d   Stored: %d", gridsize, stored)
    return config


def sim_new_xy(kk, vv, discretize=False):
    pnew = kk
    rot = vv.rot()
    r = vv.r()
    type = vv.type()
    if rot: # this is a quick turn
        hnew = discretize_angle_deg(pnew[2] + rot)
        pnew = (pnew[0], pnew[1], hnew)
    elif type == '|':  # forwards or backwards straight line
        pnew = deltap_onheading(kk, vv.dist2())
    elif r: # this is a circular turn
        dist = vv.dist2() if vv.dist2() is not None else vv.dist() # dist2 iff stage2
        c = center_for_tangent((kk[0],kk[1]), np.radians(kk[2]), r, type)
        arc_deg = 360.0*dist/(2*np.pi*r)

        # New heading
        hnew = (kk[2] + arc_deg if type=='lh' else kk[2] - arc_deg if type=='rh' else kk[2]) % 360

        # Angle of initial point relative to circle center c.
        t_c = ((kk[2]-90) if type=='lh' else (kk[2]+90) if type=='rh' else None)

        # Angle of destination point relative to c, based on r and distance traveled
        t_c_n = (t_c + arc_deg) if type=='lh' else (t_c - arc_deg) if type=='rh' else None

        if t_c_n is None:
            logging.warn("Saw state with bad heading or turn type: %s, %s", kk, vv)
            return pnew
        xnew = c[0] + r * np.sin(np.radians(t_c_n))
        ynew = c[1] - r * np.cos(np.radians(t_c_n))
        pnew = (xnew, ynew, hnew)

    if discretize: # idempotent
        return (int(pnew[0]), int(pnew[1]), discretize_angle_deg(pnew[2]))
    else:
        return pnew


def backward_arcs(whichpeg, radii, goal=(0, 0, 0), turntype='lh', config={},
                  max_gridsize=100000, debug=False, verbose=False, gridsize_mult=12,
                  obstacles=None):
    """
    Performs first stage, backwards planning step,
    Explores multiple circular arc paths in single turn direction.

    Arguments:
    - radii : list of radiuses.
    - origin : (x,y), explores backwards from this goal.
    - max_gridsize : use at this many points to sample circle perimeter.
    - turntype: lh=counterclockwise, rh=clockwise

    Returns a dictionary {(x,y,)}

    """
    logging.info("Exploring %d arcs.  min: %s  max: %s", len(radii), min(radii), max(radii))
    initial_num = 0
    if config:
        initial_num = len(config)
        logging.info("Incoming number of states: %d", initial_num)
    keys = set(config.keys()) if config else set()
    processed = 0
    for rr in radii:
        gridsize = (gridsize_mult * rr) % max_gridsize
        logging.debug("Gridsize for r=%s: %d", rr, gridsize)
        processed += 1
        if processed and (processed % 500 == 0 or processed == len(radii)) :
            logging.info("Completed %d arcs.  On arc R=%d.", processed, rr)
        new_config = backwards_arc(whichpeg, goal, rr, turntype, gridsize, config=config, obstacles=obstacles)
        if new_config:
            keys |= set(new_config.keys())
            under_portion = float(sample_check(new_config)) / len(new_config)
            if under_portion > 0.30:
                if debug:
                    logging.info("Radius %s: Undersampled portion exceeded %.3f ",
                                 rr, under_portion)
                if gridsize < max_gridsize:
                    logging.info("Increasing gridsize from %d", gridsize)
                    gridsize *= 2
                    logging.info("New gridsize: %d", gridsize)
            config.update(new_config)
        else:
            if verbose:
                logging.info(
                    "\tbackwards_arc found nothing to goal %s via radius %s, ", goal, rr)
        if keys and len(keys) > 0 and debug:
            jaccard = len(keys.intersection(set(new_config.keys()))) / float(len(keys))
            if jaccard > 0.25:
                logging.info("R=%.1f  Portion already explored: %.3f", rr, jaccard)
    logging.info("Completed goal %s : %d arcs, %d states, %d new",
                 goal, processed, len(config), len(config) - initial_num)
    return config



def fdl_rotation(heading, turn='lh', portion=0.5):
    """
    Gives the min and max radians angle of an arc covering a portion of circular arc,
    relative to a specified heading.
    :param heading: degrees
    :param turn: turn direction, lh=counterclockwise, rh=clockwise
    :param portion: portion of the arc to cover
    :return: (amin, amax), both in radians.
    """
    amin,amax = None,None
    heading = heading % 360
    if turn == 'lh':
        amin = 1.5 * np.pi
        if 0 <= heading <= 270:
            rotation = np.radians(heading)
            amin -= rotation
        else:
            rotation = np.radians(360 - heading)
            amin += rotation
    elif turn == 'rh':
        amin = 1.5 * np.pi
        if 0 <= heading <= 270:
            rotation = np.radians(heading)
            amin += rotation
        else:
            rotation = np.radians(360 - heading)
            amin -= rotation
    if amin is not None:
        amax = amin + (2.0*np.pi)*portion
    return amin, amax

def lower_xylim_for_peg(whichpeg):
    if whichpeg==NORTH:
        return LOWER_XLIMIT, N_LOWER_YLIMIT
    elif whichpeg == NORTHEAST:
        return LOWER_XLIMIT, NE_LOWER_YLIMIT
    else:
        raise ValueError("Bad peg: %s", whichpeg)

def xy_grid(whichpeg):
    """
    :return: X, Y of playing field subject to planning
    """
    xlim,ylim = lower_xylim_for_peg(whichpeg)
    x = np.arange(xlim, UPPER_XLIMIT + 1)
    y = np.arange(ylim, UPPER_YLIMIT + 1)
    return np.meshgrid(x, y)


def xy_grid_set(whichpeg):
    xlim,ylim = lower_xylim_for_peg(whichpeg)
    rr = set()
    x = np.arange(xlim, UPPER_XLIMIT + 1)
    y = np.arange(ylim, UPPER_YLIMIT + 1)
    for xx in x:
        for yy in y:
            rr.add((xx, yy,))
    return rr


def xyt_grid_set(whichpeg):
    xlim, ylim = lower_xylim_for_peg(whichpeg)
    rr = set()
    x = np.arange(xlim, UPPER_XLIMIT + 1)
    y = np.arange(ylim, UPPER_YLIMIT + 1)
    for xx in x:
        for yy in y:
            for tt in DGH:
                rr.add((xx, yy, tt,))
    return rr


def xyt_reachable_grid_set(whichpeg, unreachables):
    return xyt_grid_set(whichpeg) - unreachables


def xy_grid_set(whichpeg):
    xlim,ylim = lower_xylim_for_peg(whichpeg)
    rr = set()
    x = np.arange(xlim, UPPER_XLIMIT + 1)
    y = np.arange(ylim, UPPER_YLIMIT + 1)
    for xx in x:
        for yy in y:
            rr.add((xx, yy,))
    return rr


def size_xy_grid(whichpeg):
    xlim,ylim = lower_xylim_for_peg(whichpeg)
    X, Y = xy_grid(whichpeg)
    return X.shape[0] * X.shape[1]


def log_and_save(msg, msgs):
    """
    Logging helper in cases where logging doesn't easily display to console.
    Logs the msg at INFO level, then appends to msgs.
    :param msg: string
    :param msgs: string
    :return: msgs+msg
    """
    msgs += msg
    logging.info(msg)
    return msgs

def config_progress(whichpeg, config, unreachables=None):
    """
    Calculates and displays the number of states that have been explored.
    Also displays the portion if <unreachables> provided.
    """
    if unreachables:
        x = np.arange(0, UPPER_XLIMIT + 1)
        y = np.arange(0, UPPER_YLIMIT + 1)
        X, Y = np.meshgrid(x, y)
        config_space_size = X.shape[0] * X.shape[1] * DISCRETIZATIONS
        logging.info("Total number of states : {:,}".format(config_space_size))
        num_reachable = len(xyt_reachable_grid_set(whichpeg, unreachables))
        logging.info("Total reachable states : {:,}".format(num_reachable))
    logging.info("Number explored :        {:,}".format(len(config)))
    if unreachables:
        logging.info("Percent reachable explored:        %.1f " % (100.0 * len(config) / num_reachable))


def summarize_config(config, histo=False, verbose=False):
    if verbose:
        logging.info("%d/%d of headings have entries.", len(set([a for x, y, a in config])), len(DISCRETIZED_GOAL_HEADINGS))

    # Prints distribution of filled cells over headings
    if histo:
        for angle in DISCRETIZED_GOAL_HEADINGS:
            count = len([1 for x, y, a in config if a == angle])
            if count > 0:
                logging.info("%d : %d", angle, count)

    logging.info("Number unique path lengths in plan: %d", len(set([int(x.dist()) for x in config.itervalues()])))
    logging.info("Number unique path headings:        %d", len(set([int(k[2]) for k in config.iterkeys()])))


#   Helper for logging state data
#
def float_str(ff):
    return " None" if ff is None else "%5.1f" % ff
def print_state(kk,vv):
    # Example:
    # (0, 18, 304) {'dist': 20.007475969378717, 'goal1': (0, 19), 'r': 20, 'goal2': (0, 19, 354), 'samples': 1, 'dist2': 1.0001041959744348, 'type': 'rh'}
    print "%15s : g1:%15s  r:%6s  type: %2s  d:%4s   g2: %14s  d2: %4s  samples: %d" % \
        (kk, vv.goal1(), float_str(vv.r()), vv.type(), float_str(vv.dist()),
         vv.goal2(), float_str(vv.dist2()), vv.samples())

def str_state(kk,vv):
    # Example k,v:
    # (0, 18, 304) {'dist': 20.007475969378717, 'goal1': (0, 19), 'r': 20, 'goal2': (0, 19, 354), 'samples': 1, 'dist2': 1.0001041959744348, 'type': 'rh'}
    return "%15s : g1:%15s  r:%6s  type: %2s  d:%6s   g2: %14s  d2: %6s  samples: %d" % \
        (kk, vv.goal1(), float_str(vv.r()), vv.type(), float_str(vv.dist()),
         vv.goal2(), float_str(vv.dist2()), vv.samples())


def tuplize_state(config, key):
    """
    Converts the item in config matching key into tuple (k,r,dist,goal2,dist2)
    of form ((int,int,int),float,int,(int,int,int),float2)
    """
    return [(k, v.r(), v.dist(), v.goal2(), v.dist2()) for k, v in config.iteritems() if k == key][0]

WALL_LOWER_YLIMIT=int(-1.5*UPPER_YLIMIT) # Extends east wall down 1.5x as far as it goes up. Yes, intended, reasonable.
def xy_obstacles_set(divider=True): # doesn't depend on which peg
    """
    Generates a set of (x,y) 1x1 cells containing obstacles
    for right half of field for peg nearest Alliance Station,
    where 'right' is facing Alliance Station.
    Wall across from peg, wall to right of peg, and divider to right of peg.
    """
    rr = set()

    #   Adds alliance station wall and the wall adjacent to it
    xlim = int(round(UPPER_XLIMIT))
    ylim = int(round(UPPER_YLIMIT))
    x = np.arange(LOWER_XLIMIT-10, xlim + 1) # extended a bit west for safety margin
    y = np.arange(WALL_LOWER_YLIMIT, ylim + 1)
    for xx in x:
        rr.add((xx, ylim,))
    for yy in y:
        rr.add((xlim, yy))

    #   Adds airship walls
    rr |= set(bresenham(AIRSHIP_N_WALL[0][0], AIRSHIP_N_WALL[1][0], AIRSHIP_N_WALL[0][1], int(float(AIRSHIP_N_WALL[1][1]))))
    rr |= set(bresenham(AIRSHIP_NE_WALL[0][0], AIRSHIP_NE_WALL[1][0], AIRSHIP_NE_WALL[0][1], int(float(AIRSHIP_NE_WALL[1][1]))))
    rr |= set(bresenham(AIRSHIP_NW_WALL[0][0], AIRSHIP_NW_WALL[1][0], AIRSHIP_NW_WALL[0][1], int(float(AIRSHIP_NW_WALL[1][1]))))
    rr |= set(bresenham(AIRSHIP_SE_WALL[0][0], int(AIRSHIP_SE_WALL[1][0]), int(AIRSHIP_SE_WALL[0][1]), int(float(AIRSHIP_SE_WALL[1][1]))))
    rr |= set(bresenham(AIRSHIP_SW_WALL[0][0], int(AIRSHIP_SW_WALL[1][0]), int(AIRSHIP_SW_WALL[0][1]), int(float(AIRSHIP_SW_WALL[1][1]))))
    rr |= set(bresenham(AIRSHIP_S_WALL[0][0], AIRSHIP_S_WALL[1][0], AIRSHIP_S_WALL[0][1], int(float(AIRSHIP_S_WALL[1][1]))))
    rr |= set(bresenham(int(NE_CORNER[0][0]), int(NE_CORNER[1][0]), int(NE_CORNER[0][1]), int(float(NE_CORNER[1][1]))))

    #   Diagonals inside airship
    rr |= set(bresenham(AIRSHIP_N_WALL[0][0], AIRSHIP_N_WALL[1][0], AIRSHIP_S_WALL[0][1], int(float(AIRSHIP_S_WALL[1][1]))))
    rr |= set(bresenham(AIRSHIP_S_WALL[0][0], int(AIRSHIP_S_WALL[1][0]), int(AIRSHIP_N_WALL[0][1]), int(float(AIRSHIP_N_WALL[1][1]))))
    rr |= set(bresenham(int(AIRSHIP_NW_WALL[0][1]), int(AIRSHIP_NW_WALL[1][1]), int(AIRSHIP_NE_WALL[0][1]), int(float(AIRSHIP_NE_WALL[1][1]))))



    #   Adds dividers
    if divider:
        rr |= set(bresenham(DIVIDER3[0][0],DIVIDER3[1][0],DIVIDER3[0][1],DIVIDER3[1][1]))
        rr |= set(bresenham(DIVIDER2[0][0], DIVIDER2[1][0], DIVIDER2[0][1], DIVIDER2[1][1]))
    return rr


def bresenham(x0, y0, x1, y1):
    """Yield integer coordinates on the line from (x0, y0) to (x1, y1).
    Input coordinates should be integers.
    The result will contain both the start and the end point.
    https://github.com/encukou/bresenham
    """
    dx = x1 - x0
    dy = y1 - y0

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
        dx, dy = dy, dx
        xx, xy, yx, yy = 0, ysign, xsign, 0

    D = 2*dy - dx
    y = 0

    for x in range(dx + 1):
        yield int(round(x0 + x*xx + y*yx)), int(round(y0 + x*xy + y*yy))
        if D > 0:
            y += 1
            D -= dx
        D += dy

def get_robot_xy_outline(center, theta, hood=True):
    return get_robot_xy_outline_nose(get_robot_nose_from_center(center, theta), theta, hood)

def get_robot_xy_outline_nose(nose, theta, hood=True):
    """
    :param nose: (x,y) of nose
    :param theta: radians, south is 0, increasing counterclockwise
    :param hood: draw a hood ornament for easier plotting
    :return: set of xy points outlining robot
    """
    rear = get_robot_rear(nose, theta)

    xy_fcdl, xy_fcdr, xy_rcdl, xy_rcdr = get_robot_corners(nose, theta, rear, grid=True)
    xyset = set(bresenham(xy_fcdl[0], xy_fcdl[1], xy_fcdr[0], int(xy_fcdr[1])))
    xyset |= set(bresenham(xy_fcdl[0], xy_fcdl[1], xy_rcdr[0], int(xy_rcdr[1])))
    xyset |= set(bresenham(xy_fcdr[0], xy_fcdr[1], xy_rcdr[0], int(xy_rcdr[1])))
    xyset |= set(bresenham(xy_fcdr[0], xy_fcdr[1], xy_rcdl[0], int(xy_rcdl[1])))
    #   two diagonals prevent collisions that go undetected due to 2D discretized lines intersecting without colliding
    xyset |= set(bresenham(xy_rcdr[0], xy_rcdr[1], xy_rcdl[0], int(xy_rcdl[1])))
    xyset |= set(bresenham(xy_rcdl[0], xy_rcdl[1], xy_fcdl[0], int(xy_fcdl[1])))

    if hood:
        n2r = list(bresenham(int(nose[0]), int(nose[1]), int(rear[0]), int(rear[1])))
        xyset |= set(n2r[:len(n2r) / 6])

    return xyset

#====================================================================
#   Tools for drawing circles and points on circles
#====================================================================


def state_scatterplot(config, minx=-1, maxx=UPPER_XLIMIT, miny=-1, maxy=UPPER_YLIMIT, figsize=15, max=500, verbose=True,
    plot_obstacles=True):
    "Draws scatterplot of the discretized (x,y) points in the state configuration object."
    plt.figure(figsize=(figsize, figsize))
    ax = pylab.axes(aspect=1)
    ax.set_xlim(minx, maxx)
    ax.set_ylim(miny, maxy)
    plt.grid()
    done = 0
    for key in config.iterkeys():
        plt.plot(key[0], key[1], 'b.')
        if verbose:
            print key[0], key[1]
        done += 1
        if done > max:
            break
    if plot_obstacles:
        obstacles=xy_obstacles_set()
        for oo in obstacles:
            plt.plot(oo[0], oo[1], 'k.')
    plt.show()
    logging.info("Plotted %d/%d states", done, len(config))


def circle_grid(r, n, c, xmin, xmax, ymin, ymax, plot=False, figsize=5):
    """
    Returns a list of (x,y) points evenly spaced around circle of radius r centered at c=(h,k).

    The number of points required to give a sufficiently packed fill grows quickly
    with the radius for this approach, but arguments are provided to limit the search space.

    - r : radius
    - n : number of points to generate
    - c : (x,y) coordinate of circle center
    - xmin : points having x less than this value will be discarded
    - xmax : maximum value of x value
    - ymin : min value of y
    - ymax : max value of y
    - plot : allows generating a plot, useful for sanity checks but can take long to fill for large r
    - figsize : size of plot

    DEPRECATED.  Although useful for plotting, arc_grid is more efficient for sampling an arc.
    """
    points = []
    if plot:
        plt.figure(figsize=(figsize, figsize))
        ax = pylab.axes(aspect=1)
        plt.grid()
    for r, t in rtpairs(r, n):
        x = (r * np.cos(t)) + c[0]
        y = (r * np.sin(t)) + c[1]
        if x < xmin or x > xmax:
            logging.debug("(%s, %s) out of bounds. (xmin,xmax): (%s,%s)", x, y, xmin, xmax)
            continue
        if y < ymin or y > ymax:
            logging.debug("(%s, %s) out of bounds. (ymin,ymax): (%s,%s)", x, y, ymin, ymax)
            continue
        points.append((x, y,))
        if plot:
            plt.plot(x, y, 'bo')
    if plot:
        plt.show()
    return points

def arc_grid(r, n, c, amin=0, amax=2 * np.pi, plot=False, figsize=5, debug=False, counter=False, pp=None, ph=None, plotmax=200):
    """
    Returns a list of (x,y) points evenly spaced between angles amin and amax
    on a circle of radius r centered at c.

    Zero angle points north.
    Points are generated clockwise, with angles measured clockwise from NORTH,
    unless counter=True, where points are generated counterclockwise with angles measuring counterclockwise from NORTH.

    :param r: radius
    :param n: number of points to generate.
    :param c: (x,y) coordinate of circle center
    :param amin: min angle of arc
    :param amax: max angle of arc
    :param plot: generates a plot for debugging
    :param figsize: plot size
    :param debug:
    :param counter:
    :param pp:
    :param ph:
    :return: list of (x,y)
    """
    points = []
    if plot:
        plt.figure(figsize=(figsize, figsize))
        ax = pylab.axes(aspect=1)
        plt.grid()
    agrid = np.linspace(radians_to360(amin), radians_to360(amax), num=n, endpoint=True)
    if debug:
        print agrid
        print
    points = []
    for angle in agrid:
        t = np.radians(angle)
        t = np.arctan2(np.sin(t + np.pi / 2), np.cos(t + np.pi / 2)
                       ) if counter else np.arctan2(np.cos(t), np.sin(t))
        x = (r * np.cos(t)) + c[0]
        y = (r * np.sin(t)) + c[1]
        if debug:
            logging.info("angle: %d  (x,y) : (%.1f, %.1f)", angle, x, y)
        points.append((x, y,))
        if plot and len(points) < plotmax:
            plt.plot(x, y, 'bo')
    if plot and ph and pp:
        arrow = get_arrow(ph, 2)
        ax.arrow(pp[0], pp[1], arrow[0], arrow[1], head_width=2, head_length=2, fc='y', ec='y')
    if plot:
        plt.show()
    return points



#====================================================================
#   Helpers for drawing field elements and state configurations
#====================================================================

def plot_unexplored(whichpeg, config2, max=500, recalc=False):
    unex = get_xy_unexplored(whichpeg, config2, recalc)
    logging.info("Number of unexplored: %d ", len(unex))
    plt.figure(figsize=(15, 15))
    ax = pylab.axes(aspect=1)
    ax.grid(True)
    ax.set_xlim(-1, UPPER_XLIMIT + 5)
    ax.set_ylim(-1, UPPER_YLIMIT + 5)
    ax.set_xticks(np.arange(0, UPPER_XLIMIT + 5, 20))
    ax.set_yticks(np.arange(0, UPPER_YLIMIT + 5, 10))
    plt.plot(DIVIDER3[0], DIVIDER3[1], 'k-', lw=3)  # draws divider
    plt.plot(N_PEG_LINE[0], N_PEG_LINE[1], 'r-', lw=4)  # draws peg
    cc = 0
    for x, y in unex:
        cc += 1
        plt.plot(x, y, 'c.')
        if cc > max:
            logging.info("max reached")
            break
    print "plotted %d" % cc
    plt.show()


def binplot_unexplored(whichpeg, config2, unreachables, max=1000):
    """
    Shows heat map of regions that are relatively unexplored.
    """
    binned = bin_unexplored_xy(whichpeg, config2, unreachables)
    logging.info("Number of bins: %d ", len(binned))
    plt.figure(figsize=(15, 15))
    ax = pylab.axes(aspect=1)
    ax = pylab.axes()
    ax.grid(True)
    ax.set_xlim(-1, UPPER_XLIMIT + 5)
    ax.set_ylim(-1, UPPER_YLIMIT + 5)
    ax.set_xticks(np.arange(0, UPPER_XLIMIT + 5, 20))
    ax.set_yticks(np.arange(0, UPPER_YLIMIT + 5, 10))
    plt.plot(DIVIDER3[0], DIVIDER3[1], 'k-', lw=3)  # draws divider
    plt.plot(N_PEG_LINE[0], N_PEG_LINE[1], 'r-', lw=4)  # draws peg

    x = []
    y = []
    z = []
    cc = 0
    for k, v in binned.iteritems():
        cc += 1
        x.append(k[0])
        y.append(k[1])
        z.append(v)
        if cc > max:
            logging.info("max reached")
            break

    method = 'none'
    # method='nearest'
    heatmap, _, _ = np.histogram2d(x, y, weights=z)
    plt.clf()
    plt.imshow(heatmap, interpolation=method)

    print "plotted %d" % cc
    plt.gray()
    plt.show()


def plot_bot(nose, theta=0, arc_center=None, radii=RADII):
    """
    Plots peg location, divider, and robot outline.
    Robot outline is placed according to nose position and heading.
    Determines heading either explicitly by theta or arc_center of turning radius.

    Arguments:
    - nose: x,y of robot nose.
    - arc_center : x,y center of circle it is tangent to, if any.
    - theta : angle of heading. Zero heading is south, increases counterclockwise, radians
    """
    print ("plot_bot radii     : %s (inch)" % RADII)
    print ("plot_obt curvatures: %s " % ",".join(["%.3f" % (1.0 / x) for x in RADII]))
    plt.figure(figsize=(15, 15))
    ax = pylab.axes(aspect=1)  # Aspect=1 scales so that circles look like circles

    ax.grid(True)

    if theta is None and not arc_center:
        print "ERROR : must provide either arc center of tangent circle or theta."
        return

    if arc_center:
        print "Determining heading from center of tangent arc assuming direction=%s" % 'lh'
        theta = heading_tangent_circle(nose[0], nose[1], arc_center[0], arc_center[1], clockwise=False) # always lh for this drawing
        print "Heading: %.1f degrees (%.1f pi)" % (radians_to360(theta), theta/np.pi)

    print "heading: %d" % discretize_angle(theta)
    rear = get_robot_rear(nose, theta)
    center = get_robot_center(nose, theta)
    print "center: ", center
    print "nose from center: ", get_robot_nose_from_center(center, theta)

    # Robot corners (x,y)
    xy_fcdl, xy_fcdr, xy_rcdl, xy_rcdr = get_robot_corners(nose, theta, rear)

    # Robot chassis edges
    line_front, line_rear, line_dleft, line_dright = get_robot_chassis(xy_fcdl, xy_fcdr, xy_rcdl, xy_rcdr)


    ax.set_xlim(-1, UPPER_XLIMIT)
    ax.set_ylim(-1, UPPER_YLIMIT)
    plt.plot(DIVIDER3[0], DIVIDER3[1], 'k-', lw=3)  # draws divider
    plt.plot(N_PEG_LINE[0], N_PEG_LINE[1], 'r-', lw=4)  # draws peg

    plt.plot([xy_fcdr[0], xy_rcdr[0]], [xy_fcdr[1], xy_rcdr[1]], 'or')  # plot right corners
    plt.plot([xy_fcdl[0], xy_rcdl[0]], [xy_fcdl[1], xy_rcdl[1]], 'ob')  # plot left corners
    plt.plot([nose[0]], [nose[1]], 'oy')
    plt.plot([rear[0]], [rear[1]], 'xk')
    plt.plot([center[0]], [center[1]], '8b')

    plt.plot(line_front[0], line_front[1], 'b-')
    plt.plot(line_rear[0], line_rear[1], 'b-')
    plt.plot(line_dleft[0], line_dleft[1], 'b-')
    plt.plot(line_dright[0], line_dright[1], 'b-')

    # hood ornament
    arrow = get_arrow(theta, 0.7)
    ax.arrow(nose[0], nose[1], arrow[0], arrow[1], head_width=2, head_length=1, fc='y', ec='y')

    start, end = ax.get_xlim()
    ax.xaxis.set_ticks(np.arange(start, end, 10))

    # Plots reference nested arc paths
    for r in radii:
        ax.add_patch(make_arc(r, fc=COLORS[radii.index(r)], a=1.0 / (radii.index(r) + 1)))

    pylab.show()



def plot_airship(xlim=None,ylim=None):
    """

    Gives overhead view of the three airship walls facing the alliance station
    along with the dividers separating them and their respective pegs.

    """
    plt.figure(figsize=(15, 15))
    ax = pylab.axes(aspect=1)  # Aspect=1 scales so that circles look like circles
    ax.grid(True)

    if xlim:
        ax.set_xlim(xlim)
    if ylim:
        ax.set_ylim(ylim)

    #   draws goal wall and peg
    plt.plot(AIRSHIP_N_WALL[0], AIRSHIP_N_WALL[1], 'k-', lw=2)  # airship wall facing alliance station
    plt.plot(N_PEG_XY[0], N_PEG_XY[1], 'ro')
    plt.plot(N_PEG_LINE[0], N_PEG_LINE[1], 'r-', lw=4)  # goal peg
    plt.plot(AIRSHIP_S_WALL[0], AIRSHIP_S_WALL[1], 'k-', lw=2)  # airship wall facing alliance station

    #   adjacent airshipe wall and peg
    plt.plot(AIRSHIP_NW_WALL[0], AIRSHIP_NW_WALL[1], 'k-', lw=2)
    plt.plot(NW_PEG_XY[0], NW_PEG_XY[1], 'ro')
    plt.plot(NW_PEG[0], NW_PEG[1], 'b-', lw=4)

    plt.plot(AIRSHIP_SW_WALL[0], AIRSHIP_SW_WALL[1], 'k-', lw=2)
    plt.plot(AIRSHIP_SE_WALL[0], AIRSHIP_SE_WALL[1], 'k-', lw=2)


    #   adjacent airship wall (wall3) and its peg (peg3), clockwise looking down
    plt.plot(AIRSHIP_NE_WALL[0], AIRSHIP_NE_WALL[1], 'k-', lw=2)
    plt.plot(NE_PEG_XY[0], NE_PEG_XY[1], 'go')
    plt.plot(NE_PEG[0], NE_PEG[1], 'b-', lw=4)

    #   draws divider between goal wall and wall3
    plt.plot(DIVIDER3[0], DIVIDER3[1], 'k-', lw=3)

    #   draws divider between goal wall and wall2
    plt.plot(DIVIDER2[0], DIVIDER2[1], 'k-', lw=3)

    #   draws goals
    for p in N_XY_GOALS:
        plt.plot(p[0], p[1], 'gx')
    for p in NE_XY_GOALS:
        plt.plot(p[0], p[1], 'gx')

    start, end = ax.get_xlim()
    ax.xaxis.set_ticks(np.arange(start, end, 10))

    pylab.show()



def plot_fleur_de_lis(pp, r, figsize=5, xlim=(-1, 55), ylim=(-1, 55), gridsize=25):
    """

    :param: pp : state 3-tuple (x,y,heading) where x and y are location and heading is in DEGREES
            South is zero, angles increase counterclockwisely.

    """
    heading = pp[2] # degrees
    centerL = center_for_tangent((pp[0], pp[1]), np.radians(pp[2]), r, 'lh')
    centerR = center_for_tangent((pp[0], pp[1]), np.radians(pp[2]), r, 'rh')

    logging.info("lh center: (%.1f, %.1f)", centerL[0],centerL[1])
    logging.info("rh center: (%.1f, %.1f)", centerR[0],centerR[1])

    aminL, amaxL = fdl_rotation(heading, turn='lh')
    aminR, amaxR = fdl_rotation(heading, turn='rh')

    print "aminL: %.1f  amaxL: %.1f" % (aminL, amaxL)
    print "aminR: %.1f  amaxR: %.1f" % (aminR, amaxR)

    lpoints = arc_grid(r, gridsize, centerL, aminL, amaxL, counter=False)
    rpoints = arc_grid(r, gridsize, centerR, aminR, amaxR, counter=True)

    plt.figure(figsize=(figsize, figsize))
    ax = pylab.axes(aspect=1)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    plt.grid()
    plt.plot(pp[0], pp[1], 'ro')
    for p in lpoints:
        plt.plot(p[0], p[1], 'b.')
    for p in rpoints:
        plt.plot(p[0], p[1], 'r.')

    arrow = get_arrow(np.radians(pp[2]), 3)
    plt.plot(pp[0], pp[1], 'bo', ms=5)
    ax.arrow(pp[0], pp[1], arrow[0], arrow[1], head_width=2, head_length=3, fc='y', ec='y')

    plt.show()



def plot_center_for_tangent(p, h_rad, r=10, turntype='lh', num=30, xmin=(LOWER_XLIMIT, 40), ymin=(-5, 40)):
    """
    Visual aid for centering a circle tangent to a line
    - p is (x,y) location of tangent point
    - h_rad : heading in radians
    - r : radius of tangent circle, inches
    - turntype: lh=counterclockwise, rh=clockwise
    """
    h_deg = np.degrees(h_rad)
    c = center_for_tangent(p, h_rad, r, turntype)
    logging.info("p(%s), heading:%d deg (%.1f rad), radius:%d, \ncenter: (%s)",
                 p, h_deg, h_rad, r, c)
    sample_size = num
    amin, amax = fdl_rotation(np.degrees(h_rad), turntype)
    circle_points = arc_grid(r, sample_size, c, amin, amax, counter=(turntype == turntype))

    plt.figure(figsize=(7, 7))
    ax = pylab.axes(aspect=1)
    plt.grid()
    ax.set_xlim(xmin)
    ax.set_ylim(ymin)

    #   Plots circle
    for pp in circle_points:
        plt.plot(pp[0], pp[1], 'g.')
    plt.plot(c[0], c[1], 'ro', ms=12)

    #   Plots arrow
    arrow = get_arrow(h_rad, 3)
    plt.plot(p[0], p[1], 'bo', ms=5)
    ax.arrow(p[0], p[1], arrow[0], arrow[1], head_width=2, head_length=3, fc='y', ec='y')

    plt.show()



def flow_plot_state(state, xlim=None, ylim=None, arrowsize=None, arrowlength=None, figsize=6, num=1000, turntype='lh', stage2=False, max=1000):
    """
    Plots the grid location and heading, and a tangent circle on its current turning radius.
    If stage2=True, tries to explore backwards from the (x,y) of the state.

    Arguments:
    - state : ((x,y,t), r, h)
    - xlim : 2-tuple
    - ylim : 2-tuple
    - turntype: lh=counterclockwise, rh=clockwise
    """
    plt.figure(figsize=(figsize, figsize))
    ax = pylab.axes(aspect=1)  # Aspect=1 scales so that circles look like circles
    plt.grid()
    if xlim:
        ax.set_xlim(xlim)
    if ylim:
        ax.set_ylim(ylim)

    k,v = state[0], state[1]
    p = (k[0], k[1])
    h_deg = k[2]
    h_rad = np.radians(h_deg)
    radius = v.r()
    dist = v.dist()

    # Sample arc clockwise or counter clockwise depending on turntype
    c = center_for_tangent(p, h_rad, radius, turntype)
    print "Location p(%s) with heading %d deg (%.1f rad) on radius %d " % (p, h_deg, h_rad, radius)
    print "radius:%d, heading : %.1f radians, %d degrees,  dist: %.1f" % (radius, h_rad, h_deg, dist)
    print "Center of turning radius = (%.1f, %.1f)" % (c[0], c[1])
    sample_size = num
    counter = True if turntype == 'rh' else False

    if stage2:
        amin, amax = fdl_rotation(k[2], 'lh') # Explores semicircle
        lpoints = arc_grid(radius, radius*np.pi*2, center_for_tangent(p, h_rad, radius, 'lh'), amin, amax, counter=False)
        amin, amax = fdl_rotation(k[2], 'rh') # Explores semicircle
        rpoints = arc_grid(radius, radius*np.pi*2, center_for_tangent(p, h_rad, radius, 'rh'), amin, amax, counter=True)
        circle_points = lpoints+rpoints
    else:
        amin = (3.0 / 2.0) * np.pi # explore 75% of the arc, rely on unreachability to eliminate out of bounds etc
        amax = (5.0 / 2.0) * np.pi
        circle_points = arc_grid(radius, sample_size, c, amin, amax, counter=counter)

    xx = 0
    for point in circle_points:
        xx += 1
        plt.plot(point[0], point[1], 'g.')
        if xx > max:
            print "Exceeded max: %d" % max
    print "Plotted: %d " % xx

    #   Draws arrow
    if not arrowsize and xlim:
        arrowsize = 10 if xlim[1] - xlim[0] > 100 else 5 if xlim[1] - xlim[0] > 50 else 2
    if not arrowlength and xlim:
        arrowlength = 10 if xlim[1] - xlim[0] > 50 else 5 if xlim[1] - xlim[0] > 50 else 2
    arrow = get_arrow(h_rad, arrowlength)
    plt.plot(state[0][0], state[0][1], 'b.')
    ax.arrow(state[0][0], state[0][1], arrow[0], arrow[1],
             head_width=arrowsize, head_length=arrowsize, fc='y', ec='y')
    plt.show()


def flow_plot_config(config, xlim=None, ylim=None, arrowsize=None, arrowlength=None, figsize=6,
                     verbose=False, max=500, verbosemax=20, plot_obstacles=True):
    """
    Plots the grid location and headings of up to <max> states from the configuration state object.

    :param config:
    :param xlim:
    :param ylim:
    :param arrowsize:
    :param arrowlength:
    :param figsize:
    :param verbose:
    :param max:
    :return:
    """

    plt.figure(figsize=(figsize, figsize))
    ax = pylab.axes(aspect=1)  # Aspect=1 scales so that circles look like circles
    plt.grid()
    if xlim:
        ax.set_xlim(xlim)
    if ylim:
        ax.set_ylim(ylim)

    if not arrowsize and xlim:
        arrowsize = 10 if xlim[1] - xlim[0] > 100 else 5 if xlim[1] - xlim[0] > 50 else 2
    if not arrowlength and xlim:
        arrowlength = 10 if xlim[1] - xlim[0] > 50 else 5 if xlim[1] - xlim[0] > 50 else 2

    did = 0
    didv = 0
    for kk,vv in config.iteritems():
        if verbose and didv < verbosemax:
            print "%s:%s" % (kk,vv)
            didv += 1
        heading_deg = kk[2]
        heading_rad = np.radians(heading_deg)

        #   Draws arrow
        arrow = get_arrow(heading_rad, arrowlength)
        plt.plot(kk[0], kk[1], 'b.')
        ax.arrow(kk[0], kk[1], arrow[0], arrow[1],
                 head_width=arrowsize, head_length=arrowsize, fc='y', ec='y')
        did += 1
        if did > max:
            break
    print "Plotted %d/%d " % (did, len(config))

    if plot_obstacles:
        obstacles=xy_obstacles_set()
        for oo in obstacles:
            plt.plot(oo[0], oo[1], 'k.')

    plt.show()


def flow_plot_xyt(xyt, xlim=None, ylim=None, arrowsize=None, arrowlength=None, figsize=6,
                     verbose=False, max=500, plot_obstacles=True):
    """
    Plots the grid location and headings of each state point in list of tuples.

    Arguments:
    - xyt : iterable of 3-tuples of form (x,y,t)
    - xlim : 2-tuple
    - ylim : 2-tuple
    """

    plt.figure(figsize=(figsize, figsize))
    ax = pylab.axes(aspect=1)  # Aspect=1 scales so that circles look like circles
    plt.grid()
    if xlim:
        ax.set_xlim(xlim)
    if ylim:
        ax.set_ylim(ylim)

    if not arrowsize and xlim:
        arrowsize = 10 if xlim[1] - xlim[0] > 100 else 5 if xlim[1] - xlim[0] > 50 else 2
    if not arrowlength and xlim:
        arrowlength = 10 if xlim[1] - xlim[0] > 50 else 5 if xlim[1] - xlim[0] > 50 else 2

    did = 0
    for kk in xyt:
        if verbose:
            print "State : ",kk
        if len(kk) < 3:
            logging.warn("Expected len(state)==3, instead: %d", len(kk))
            continue
        heading_deg = kk[2]
        heading_rad = np.radians(heading_deg)

        #   Draws arrow
        arrow = get_arrow(heading_rad, arrowlength)
        plt.plot(kk[0], kk[1], 'b.')
        ax.arrow(kk[0], kk[1], arrow[0], arrow[1],
                 head_width=arrowsize, head_length=arrowsize, fc='y', ec='y')
        did += 1
        if did > max:
            break
    print "Plotted %d/%d " % (did, len(xyt))

    if plot_obstacles:
        obstacles=xy_obstacles_set()
        for oo in obstacles:
            plt.plot(oo[0], oo[1], 'k.')

    plt.show()



def plot_xy(xy, xlim=None, ylim=None, figsize=8, max=1000, grid=False, plot_obstacles=True):
    """
    Plots (x,y) points

    Arguments:
    - xy : collection of tuples of form (x,y)
    - xlim : 2-tuple
    - ylim : 2-tuple
    """
    plt.figure(figsize=(figsize, figsize))
    ax = pylab.axes(aspect=1)
    if xlim:
        ax.set_xlim(xlim)
    if ylim:
        ax.set_ylim(ylim)
    did = 0
    for p in xy:
        plt.plot(p[0], p[1], 'b', marker='.')
        did += 1
        if max and did > max:
            break
    print "Plotted %d/%d " % (did, len(xy))
    if grid:
        plt.grid()
        loc = plticker.MultipleLocator(base=12)
        ax.xaxis.set_major_locator(loc)
        ax.yaxis.set_major_locator(loc)
    if plot_obstacles:
        obstacles=xy_obstacles_set()
        for oo in obstacles:
            plt.plot(oo[0], oo[1], 'k.')
    plt.show()


def deg_wrt_c(p1,c):
    deg1 = np.degrees(np.arctan2(p1[0] - c[0], -p1[1] + c[1]))
    return (deg1 + 360 if deg1 < 0 else deg1) % 360

def path_plot_xyt(trajectories, xlim=None, ylim=None, arrowsize=None, arrowlength=None, figsize=6,
                     verbose=False, maxpaths=3, plot_obstacles=True):
    """
    Plots planned paths.

    :param trajectories: example pickle.load(open('sample_traj.pickle', 'rb'))
    :param xlim: upper x limit for plot
    :param ylim: upper y limit for plot
    :param arrowsize:
    :param arrowlength:
    :param figsize:
    :param verbose:
    :param maxpaths: maximum number of paths to draw
    """
    plt.figure(figsize=(figsize, figsize))
    ax = pylab.axes(aspect=1)  # Aspect=1 scales so that circles look like circles
    plt.grid()
    if xlim:
        ax.set_xlim(xlim)
    if ylim:
        ax.set_ylim(ylim)
    if not arrowsize and xlim:
        arrowsize = 10 if xlim[1] - xlim[0] > 100 else 5 if xlim[1] - xlim[0] > 50 else 2
    if not arrowlength and xlim:
        arrowlength = 10 if xlim[1] - xlim[0] > 50 else 5 if xlim[1] - xlim[0] > 50 else 2

    points = []
    arcs = []
    paths=0
    for key in trajectories:
        prev = key
        waypoints = [key] + [x[0] for x in trajectories[key]]
        for path in trajectories[key]:
            pp = path[0]
            if path[3] == '|':
                pp = deltap_onheading(prev,path[4])
                points += list(bresenham(prev[0],prev[1], pp[0], pp[1]))
            if path[3] in ['lh','rh']:
                r = path[5]
                d = path[4]
                if path[3] == 'lh':
                    c = center_for_tangent((prev[0],prev[1]),np.radians(prev[2]),r,turntype="lh")
                    deg1 = deg_wrt_c(prev,c)
                    deg2 = deg_wrt_c(pp,c)
                    theta1=0
                    theta2=deg2-deg1
                if path[3] == 'rh':
                    c = center_for_tangent((prev[0],prev[1]),np.radians(prev[2]),r,turntype="rh")
                    deg1 = deg_wrt_c(prev,c)
                    deg2 = deg_wrt_c(pp,c)
                    theta1=deg2-deg1
                    theta2=0
                if verbose:
                    print
                    print "c for",prev,r,'lh'
                    print "c   : ", c
                    print "deg1: ", deg1
                    print "deg2: ", deg2
                    print "prev: ", prev
                    print "p   : ", pp
                if r:
                    arcs.append({"r":r,"c":c,"angle":deg1-90,"theta1":theta1,"theta2":theta2})
            prev = pp

        #   Plots xy path points
        for p in points:
            plt.plot(p[0], p[1], 'c.', marker='.', ms=2)

        #   Plots xyt waypoints
        for kk in waypoints:
            if verbose:
                print "Waypoint: ",kk
            heading_deg = kk[2]
            heading_rad = np.radians(heading_deg)

            #   Draws arrow
            arrow = get_arrow(heading_rad, arrowlength)
            plt.plot(kk[0], kk[1], 'b.')
            ax.arrow(kk[0], kk[1], arrow[0], arrow[1],
                     head_width=arrowsize, head_length=arrowsize, fc='y', ec='y')
        if verbose:
            print "Plotted %d waypoints" % (len(waypoints))
        paths+=1
        if paths>=maxpaths:
            break

    for arc in arcs:
        r = arc['r']
        c = arc['c']
        ax.add_patch(patches.Arc(c,2*r,2*r,angle=arc['angle'],theta1=arc['theta1'], theta2=arc["theta2"]))

    if plot_obstacles:
        obstacles=xy_obstacles_set()
        for oo in obstacles:
            plt.plot(oo[0], oo[1], 'k.')
    plt.show()
    return plt


def collides(k, obstacles):
    """
    :param: k=(x,y,h), where h=degrees.
    :return: True if collides with obstacle
    """
    center = (k[0],k[1])
    heading = k[2]
    bot = get_robot_xy_outline(center, np.radians(heading), hood=False)
    if (bot & obstacles):
        return True
    else:
        return False


def check_collisions(keys):
    """
    :param keys: iterable of (x,y,h), where h=degrees.
    :return: set of keys that collided with obstacle.
    """
    obstacles = xy_obstacles_set()
    collisions = set()
    for k in keys:
        if collides(k,obstacles):
            collisions.add(k)
    return collisions


#====================================================================
#   Plotting configuration states
#====================================================================

def plot_check_collision(center,heading,degrees=True,verbose=True):
    """
    :param: nose: (x,y)
    :heading: degrees
    """
    if verbose:
        print "x,y,t: (%s,%s,%s) " % (center[0],center[1],heading)
    head_rad = np.radians(heading) if degrees else heading
    obstacles = xy_obstacles_set()
    bot = get_robot_xy_outline(center, head_rad)
    plot_xy(obstacles | bot, figsize=12)
    print "\t\t\t===   COLLIDES   ===" if (bot&obstacles) else "Clear"


def plot_unreachables_xy(unreachables, max=None):
    """
    This draws the (x,y) grid locations for which there is
    at least one heading which results in an unreachable state.
    """
    unreachables_xy = set()
    for x,y,h in unreachables:
        unreachables_xy.add((x,y))
    plot_xy(unreachables_xy, max=max)


def est_xy_unexplored(config2,verbose=True):
    """
    Uses aggregate calculation to estimate how many (x,y) positions are as yet unexplored.
    :param config2:
    :param verbose:
    :return:
    """
    size = size_xy_grid()
    explored = len(get_xy_explored(config2))
    if verbose:
        print "xy cells not yet explored: %d/%d (%.1f percent)" % ((size-explored),size,100.0*(size-explored)/float(size))
    return (size-explored)




def print_plan_stats(whichpeg, plan_states, unreachables=None):
    """
    Prints progress and stats about state space coverage.
    :param plan_states:
    :return:
    """
    if unreachables:
        reachable_grid = get_reachable_grid(whichpeg, unreachables)
        num_reachable = len(reachable_grid)
        logging.info("Total number of reachable states:      %8d", num_reachable)

    num_stage1 = len([k for k, v in plan_states.iteritems() if is_stage1(v)])
    num_stage2 = len([k for k, v in plan_states.iteritems() if is_stage2(v)])
    num_stage3 = len([k for k, v in plan_states.iteritems() if is_stage3(v)])
    num_stage4 = len([k for k, v in plan_states.iteritems() if is_stage4(v)])
    logging.info("Num stage1 paths : %8d", num_stage1)
    logging.info("Num stage2 paths : %8d", num_stage2)
    logging.info("Num stage3 paths : %8d", num_stage3)
    logging.info("Num stage4 paths : %8d", num_stage4)

    num_stage1_lh = len([k for k, v in plan_states.iteritems() if is_stage1(v) and v.type() == 'lh'])
    num_stage1_rh = len([k for k, v in plan_states.iteritems() if is_stage1(v) and v.type() == 'rh'])
    logging.info("Num stage1 / lh  : %8d", num_stage1_lh)
    logging.info("Num stage1 / rh  : %8d", num_stage1_rh)

    if num_stage1_lh+num_stage1_rh != num_stage1:
        logging.error("DISCREPANCY: num_stage1_lh + num_stage1_rh - num_stage1 = %d ", (num_stage1_lh + num_stage1_rh - num_stage1))

    num_stage2_lh = len([k for k, v in plan_states.iteritems() if is_stage2(v) and v.type() == 'lh'])
    num_stage2_rh = len([k for k, v in plan_states.iteritems() if is_stage2(v) and v.type() == 'rh'])
    logging.info("Num stage2 / lh  : %8d", num_stage2_lh)
    logging.info("Num stage2 / rh  : %8d", num_stage2_rh)

    if num_stage2_lh+num_stage2_rh != num_stage2:
        logging.error("DISCREPANCY: num_stage2_lh + num_stage2_rh - num_stage2 = %d ", (num_stage2_lh + num_stage2_rh - num_stage2))

    num_stage3_lh = len([k for k, v in plan_states.iteritems() if is_stage3(v) and v.rot() > 0])
    num_stage3_rh = len([k for k, v in plan_states.iteritems() if is_stage3(v) and v.rot() < 0])  # TODO investigate this == 0
    # shouldn't be any quickturns in stage3 having zero rotation.
    logging.info("Num stage3 / lh  : %8d", num_stage3_lh)
    logging.info("Num stage3 / rh  : %8d", num_stage3_rh)

    if num_stage3_lh+num_stage3_rh != num_stage3:
        logging.error("DISCREPANCY: num_stage3_lh + num_stage3_rh - num_stage3 = %d ", (num_stage3_lh + num_stage3_rh - num_stage3))


    num_stage4_fwd = len([k for k, v in plan_states.iteritems() if is_stage4(v) and v.dist2() > 0])
    num_stage4_rev = len([k for k, v in plan_states.iteritems() if is_stage4(v) and v.dist2() < 0])
    # shouldn't be any zero distance moves in stage4 having zero rotation.
    logging.info("Num stage4 / fwd : %8d", num_stage4_fwd)
    logging.info("Num stage4 / rev : %8d", num_stage4_rev)

    if num_stage4_fwd+num_stage4_rev != num_stage4:
        logging.error("DISCREPANCY: num_stage4_fwd + num_stage4_rev - num_stage4 = %d ", (num_stage4_fwd + num_stage4_rev - num_stage4))

    # Overall Summary
    logging.info("Number of states having a path:        %8d", len(get_xyt_explored(plan_states)))

    if unreachables:
        num_paths_unreachable = len(get_xyt_explored(plan_states)&unreachables)  # should be 0
        logging.info("Number of these that are unreachable:  %8d    (Should be 0)", num_paths_unreachable)

        num_missing = len(reachable_grid - get_xyt_explored(plan_states))
        logging.info("Number reachable states lacking path:  %8d", num_missing)
        logging.info("Percent reachable states having path:     %6.2f", (100.0-(100.0 * num_missing / num_reachable)))

        num_missing2 = num_reachable - len(plan_states) # valid iff num plan states that are unreachable is zero
        logging.info("Num missing via subtraction  :         %8d", num_missing2)
        logging.info("Percent having path via subtraction :     %6.2f", (100.0-(100.0*num_missing2/num_reachable)))

        if num_paths_unreachable == 0:
            if num_missing != num_missing2:
                logging.info( "Diff missing-missing2           :      %8d ", (num_missing-num_missing2))



def forwards_search_line(whichpeg, explored, xyt_fillable):
    """
    Performs an optimizing forwards search upon each state in the incoming state configuration space
    seeking an existing solved state directly in front of or behind the robot along the current heading.

    Memo: goal2 is primary goal, goal1 is stage3 goal.

    The following fields are created or modified in the resulting state configuration :
    - type = '|'
    - dist2 : negative for backwards travel or positive for forwards travel
    - dist  : overall path distance to goal
    - goal1 : next goal that the plan visits after the move.
    - goal2 : primary goal at the end of the entire (2 or 3) stage path.

    :param explored: configuration returned by stage3 planner.
    :param xyt_fillable: configuration keys of empty states to be explored.
    :param out_path: filepath to where configuration will be checkpointed every PROGRESS steps.
    :return: configuration.
    """
    PROGRESS = 10000
    if explored is None:
        logging.error("Expected a state config file! Required for stage3")
        return explored
    start = time.time()
    num = len(xyt_fillable)
    logging.info( "Stage 4: todo: %d", num)
    ii=0
    warnings, warnings2, warnings3 = 0,0,0
    max_warnings, max_warnings2, max_warnings3 = 5,5,5
    obstacles = xy_obstacles_set()
    for kk in xyt_fillable:
        if kk in explored:
            logging.debug("State already solved in previous stage.  Skipping %s ", kk)
            continue

        #   Find a point (x1,y1) directly behind and out of bounds.
        #   Let (x0,y0) be robot location, and use bresenham(x0, y0, x1, y1)
        #   to provide the grid points between.
        #   Do similar operation using point on a line ahead and out of bounds.
        #   Searching outwards from current location, identify points having a path to goal,
        #   stopping search in that direction if obstacle encountered.
        #   Mark point having shortest 'dist' as best.

        # Find other states at this xy but different heading previously filled with stage1 or stage2 path.
        # Note that this excludes stage3 quickturns.
        lga = line_grid_ahead(whichpeg, kk, obstacles)
        lgb = line_grid_behind(whichpeg, kk, obstacles)
        if not lga or not lgb:
            continue

        filled = [(k,explored[k].dist()) for k in lga+lgb
                    if k in explored and explored[k].dist() and explored[k].goal2()
                        and explored[k].type() != '|']
        if filled:
            best = min(filled, key=operator.itemgetter(1))[0]
        else:
            if warnings < max_warnings:
                logging.warn("In stage4: no adjacent states.  Skipping %s", kk)
                if warnings == max_warnings:
                    logging.warn("NOTE : Silencing this warning")
            warnings+=1
            continue

        if best:
            if kk in explored:
                logging.error("Unexpected: key %s should not yet appear.  Skipping ", kk)
                continue

            dist2 = xy_dist((kk[0],kk[1]), (best[0],best[1]))
            dist2 = dist2 if best in lga else -dist2 if best in lgb else 0
            if dist2 == 0: # Should not occur
                if warnings2 <= max_warnings2:
                    logging.warn("Encountered dist2 == 0 (SKIPPING). kk: %s, best: %s", kk, best)
                    if warnings2 == max_warnings2:
                        logging.warn("NOTE : Silencing this warning")
                warnings2 += 1
                continue

            # goal2 is the goal2 of the next state if that is a stage2 or stage2 state,
            # otherwise goal2 = goal1, which gives the stage1 state.
            goal2 = explored[best].goal2() if explored[best].goal2() or explored[best].rot() \
                                            else explored[best].goal1()

            explored[kk] = State(best, abs(dist2)+explored[best].dist(), '|', goal2=goal2, dist2=dist2)

        else:
            if warnings3 <= max_warnings3:
                logging.warn("Encountered no best (SKIPPING). kk: %s", kk)
                if warnings3 == max_warnings3:
                    logging.warn("NOTE : Silencing this warning")
            warnings3 += 1

        ii+=1
        if ii%PROGRESS == 0 or ii>=num:
            logging.info("Completed %d/%d (%.1f min)", ii, num, (time.time()-start)/60.0)

    if warnings2:
        logging.info("Finished forwards_search_line with %d zero dist2 warnings.", warnings2)
    if warnings:
        logging.info("Finished forwards_search_line with %d no adjacents warnings.", warnings)

    return explored


def forwards_quickturns(explored, xyt_fillable):
    """
    Performs a optimizing forwards planning exploration for each state in the incoming state configuration space 
    looking for an existing solved state at the same (x,y) location but different heading.

    Memo: goal2 is primary goal, goal1 is stage3 goal.

    The following fields are created or modified in the resulting state configuration :
    - type = 'qt'
    - rot: contains rotation in degrees, positive=counterclockwise turn, negative=clockwise
    - goal1 : stage3 goal that the plan visits after the quickturn.
    - goal2 : primary goal at the end of the entire (2 or 3) stage path.

    :param explored: configuration returned by stage2 planner.
    :param xyt_fillable: configuration keys of empty states to be explored.
    :param out_path: filepath to where configuration will be checkpointed every PROGRESS steps.
    :return: configuration.
    """
    PROGRESS = 50000
    if explored is None:
        logging.error("Expected a state config file! Required for stage3")
        return explored
    start = time.time()
    num = len(xyt_fillable)
    logging.info("Stage 3: todo: %d", num)
    ii=0
    warnings=0
    warnings2=0
    max_warnings, max_warnings2 = 5,5
    for kk in xyt_fillable:
        if kk in explored:
            logging.debug("State already solved in previous stage.  Skipping %s ", kk)
            continue

        best = None
        fw_keys = [(kk[0],kk[1],dgh) for dgh in DGH]

        # Find other states at this xy but different heading previously filled with stage1 or stage2 path,
        # Excludes previous stage3 quickturns.
        filled = [(fwk,explored[fwk].dist()) for fwk in fw_keys
                    if fwk in explored and explored[fwk].dist() and explored[fwk].goal1()
                        and explored[fwk].rot() is None and explored[fwk].type() != '|']
        if filled:
            best = min(filled, key=operator.itemgetter(1))[0]
        else:
            if warnings<5:
                logging.warn("In stage3: expected to find adjacent states.  Skipping %s", kk)
            warnings+=1
            continue

        if best:
            if kk in explored:
                logging.error("Unexpected: key %s should not yet appear.  Skipping ", kk)
                continue
            rotation = angle_diff(best[2],kk[2])  # heading to add to current state to reach best state
            #   Positive rotation is left-hand (counterclockwise), negative is right-hand (clockwise)
            if rotation > 180:
                logging.warn("%s => %s : out of range rotation: %s", kk,best,rotation) # on defensive til burned in
                rotation -= 360
                logging.warn("%s => %s : adjusted : %s", kk,best,rotation)
            elif rotation < -180:
                logging.warn("%s => %s : under range rotation: %s", kk,best,rotation)
                rotation += 360
                logging.warn("%s => %s : adjusted : %s", kk, best, rotation)
            # Following warning should not occur if angles have been discretized properly but can be safely ignored
            if rotation == 0 and (best[2]-kk[2] >= DISCRETIZATION_FACTOR):
                if warnings2 <= max_warnings2:
                    logging.warn("Encountered quickturn rotation of 0 (SKIPPING). kk: %s, best: %s", kk, best)
                    if warnings2 == max_warnings2:
                        logging.warn("NOTE : Silencing this warning")
                warnings2 += 1
                continue

            # ensures this state is slightly more costly than its target even tho it travels zero distance
            _dist2 = 0.1
            # goal2 is the ultimate primary goal that is at the end of the entire trajectory.
            # goal2 is the goal2 of the next state if it is a stage2 state, else goal1 if that is a stage1 state.
            _goal2 = explored[best].goal2() if explored[best].goal2() else explored[best].goal1()

            explored[kk] = State(best, explored[best].dist()+0.1, 'qt', None, goal2=_goal2, dist2=_dist2, rot=rotation)

        ii+=1
        if ii%PROGRESS == 0 or ii>=num:
            logging.info("Completed %d/%d (%.1f min)", ii, num, (time.time()-start)/60.0)

    if warnings2:
        logging.info("Finished forwards_quickturns with %d zero rotation warnings.", warnings2)
    if warnings:
        logging.info("Finished forwards_quickturns with %d no adjacents warnings.", warnings)

    return explored

#====================================================================
#
#           The following read from or write to disk
#
#====================================================================

def check_collisions_config(config):
    logging.info("Checking collisions")
    start_time = time.time()
    collisions = check_collisions(config.iterkeys())
    if collisions:
        logging.warn("Collisions: %d/%d  (%.3f)    elapsed: %.1f min)",
                     len(collisions), len(config), float(len(collisions)) / len(config), (time.time() - start_time) / 60.0)
    else:
        logging.info("No collisions")


def run_stage4(whichpeg, in_path, out_path):
    """
    Performs forwards planner stage that for each state not yet explored by stage 2
    looks for a state directly in front of or behind on the same heading.
    """
    start_time = time.time()

    explored = load_config(in_path)
    unreachables = load_unreachables(whichpeg)
    xyt_fillable = get_xyt_unexplored(whichpeg, explored, unreachables)
    logging.info("Number unexplored reachable states remaining: %d", len(xyt_fillable))
    del unreachables
    gc.collect()
    logging.info("Number of states pre-stage4: %d", len(explored))
    explored = forwards_search_line(whichpeg, explored, xyt_fillable)
    logging.info("Number of states post-stage4: %d", len(explored))
    del xyt_fillable
    gc.collect()
    logging.info("Writing to %s", out_path)
    save_pickle(explored, out_path)
    unreachables = load_unreachables(whichpeg)
    logging.info("=========================================================")
    summarize_config(explored)
    logging.info("=========================================================")
    config_progress(whichpeg, explored, unreachables)
    logging.info("=========================================================")
    print_plan_stats(whichpeg, explored, unreachables)
    logging.info("=========================================================")

    check_collisions_config(explored)

    logging.info("===   STAGE 4 DONE   ===")
    logging.info("%.1f min", (time.time() - start_time)/60.0)

    return explored


def run_stage3(whichpeg, in_path, out_path):
    """
    Performs forwards planner stage that for each state not yet explored by stage 2
    looks for a state having the same (x,y) and if one exists rotates to the one 
    having the shortest distance to goal using a quickturn (zero radius turn).
    """
    start_time = time.time()

    explored = load_config(in_path)
    unreachables = load_unreachables(whichpeg)
    xyt_fillable = get_xyt_fillable(whichpeg, explored, unreachables)
    del unreachables
    gc.collect()
    explored = forwards_quickturns(explored, xyt_fillable)
    del xyt_fillable
    gc.collect()
    logging.info("Writing to %s", out_path)
    save_pickle(explored, out_path)
    unreachables = load_unreachables(whichpeg)
    logging.info("=========================================================")
    summarize_config(explored)
    logging.info("=========================================================")
    config_progress(whichpeg, explored, unreachables)
    logging.info("=========================================================")
    print_plan_stats(whichpeg, explored,unreachables)
    logging.info("=========================================================")

    check_collisions_config(explored)

    logging.info("===   STAGE 3 DONE   ===")
    logging.info("%.1f min", (time.time() - start_time)/60.0)

    return explored


STAGE2_TURN_RADIUS = 15  # ~ half ROBOT_WIDTH
def run_stage2(whichpeg, config2, filepath, d_U=1, d_step=10):
    """
    Runs second stage backwards planner using breadth-first search progressively farther from goal.

    :param config2: configuration returned by first stage planner.
    :param unreachables:
    :param d_U: first distance to explore
    :param filepath: where to store stage2 config
    :param d_step: step size of each search wave in terms of distance from goal.
    :return: configuration

    """
    obstacles = xy_obstacles_set()
    start_time = time.time()
    max_d = max([v.dist() for v in config2.itervalues()])
    stage2_r = STAGE2_TURN_RADIUS  # turn radius of stage2 fleur-de-lis maneuvers
    logging.info("Max d over stage 1: %.1f", max_d)
    logging.info("Stage 1 progress:")
    config_progress(whichpeg, config2)
    processed = 0
    d_L = 0
    while d_L < max_d:

        d_L = d_U
        d_U = d_L + d_step

        logging.info("============================================================")
        logging.info("===   Processing %.3f < d <= %.3f ", d_L, d_U)
        logging.info("============================================================")
        c_todo = dict([(k, v) for k, v in config2.iteritems() if is_stage1(v) and (d_L < v.dist() <= d_U)])
        logging.info("Number of cells having %.1f < dist <= %.1f : %d ", d_L, d_U, len(c_todo))
        if d_U >= max_d and len(c_todo) == 0:
            break
        num_states = len(config2)
        for k, v in c_todo.items():
            if is_stage2(v): # only expand stage 1 states
                continue

            config2 = backwards_arc(whichpeg, k, stage2_r,
                                    turntype='lh',
                                    gridsize=12*stage2_r, # adaptive gridsize, ~2 point per radian-inch
                                    config=config2,
                                    goal2=v.goal1(),
                                    dist2=v.dist(),
                                    stage2=True,
                                    obstacles=obstacles)

            config2 = backwards_arc(whichpeg, k, stage2_r,
                                    turntype='rh',
                                    gridsize=12*stage2_r,
                                    config=config2,
                                    goal2=v.goal1(),
                                    dist2=v.dist(),
                                    stage2=True,
                                    obstacles=obstacles)

        logging.info("=============================")
        logging.info("===   Added %d states ", len(config2) - num_states)
        logging.info("=============================")

        config_progress(whichpeg, config2)
        logging.info("=======================================")
        print_plan_stats(whichpeg, config2)
        logging.info("=======================================")

        logging.info("============================================================")
        logging.info("===   Completed : %.3f < d <= %.3f (%.1f min)",
                     d_L, d_U, (time.time() - start_time)/60.0)
        logging.info("============================================================")
        processed += 1
        logging.info("iterations: %d", processed)
        if processed%4 == 0:
            logging.info("Writing to %s", filepath)
            save_pickle(config2, filepath)
        logging.info("")

    logging.info("=========================================================")
    summarize_config(config2)
    logging.info("=========================================================")
    unreachables = load_unreachables(whichpeg)
    config_progress(whichpeg, config2, unreachables)
    logging.info("=========================================================")
    print_plan_stats(whichpeg, config2, unreachables)
    logging.info("=========================================================")

    logging.info("Writing to %s", filepath)
    save_pickle(config2,filepath)

    check_collisions_config(config2)

    logging.info("===   STAGE 2 DONE   ===")
    logging.info("%.1f min", (time.time() - start_time)/60.0)

    return config2


def run_stage1(whichpeg, min_radius=15, max_radius=2000, stepsize=0.5, save=False,
               config=None):
    """
    Runs first stage planner and saves to file.

    :return: configuration

    """
    if whichpeg==NORTH:
        goals = N_XY_GOALS
        goal_headings = N_GOAL_HEADINGS
        filepath = N_FILEPATHS[1]
    elif whichpeg==NORTHEAST:
        goals = NE_XY_GOALS
        goal_headings = NE_GOAL_HEADINGS
        filepath = NE_FILEPATHS[1]

    start_time = time.time()
    arcs = np.arange(min_radius, max_radius + stepsize, stepsize)
    config = config if config else {}
    obstacles = xy_obstacles_set()
    for turn_dir in ['rh','lh']:
        logging.info("===============================")
        logging.info("===   Turn direction %s   ===", turn_dir)
        logging.info("===============================")
        logging.info("Evaluating following goals: %s", goals)
        for xy in goals:
            logging.info("Evaluating goal %s for headings %s", xy, goal_headings)
            for gh in goal_headings:
                goal = (xy[0], xy[1], gh)
                logging.info("===   Starting  : Goal %s  Dir: %s  ===", goal, turn_dir)
                config = backward_arcs(whichpeg, arcs, (xy[0],xy[1],gh), turn_dir, config, obstacles=obstacles)
                logging.info("===   Completed : Goal %s  Dir: %s  (%.1f min)   ===",
                             goal, turn_dir, (time.time() - start_time)/60.0)
            logging.info("")
        logging.info("===   Completed : Direction %s  (%.1f min)   ===", turn_dir, (time.time() - start_time)/60.0)
        logging.info("")
        if save:
            save_pickle(config, filepath)
        logging.info("")
    logging.info("STAGE 1 DONE (%.1f min)", (time.time() - start_time)/60.0)

    return config


def save_pickle(config, path, protocol=-1):
    logging.info("Writing %d rows to %s", len(config), path)
    with open(path, 'wb') as fp:
        pickle.dump(config, fp, protocol)


def unreachables_filepath(whichpeg):
    if whichpeg==NORTH:
        return N_UNREACHABLES_FILEPATH
    elif whichpeg==NORTHEAST:
        return NE_UNREACHABLES_FILEPATH

def run_unreachables(whichpeg):
    start_time = time.time()
    xyt_set = xyt_grid_set(whichpeg)
    obstacles = xy_obstacles_set()
    unreachables = set()
    dd = 0
    todo = len(xyt_set)
    logging.info("Evaluating %d states", todo)
    for x, y, heading in xyt_set:
        dd += 1
        head_rad = np.radians(heading % 360)
        if get_robot_xy_outline((x, y,), head_rad, hood=False) & obstacles:
            unreachables.add((x, y, heading))
        if dd % 100000 == 0 or dd == todo:
            logging.info("Analyzed: %d   Unreachable: %d  (%.3f)   (elapsed: %.1f min)",
                          dd, len(unreachables), len(unreachables)/float(dd), (time.time() - start_time)/60.0)

    if len(unreachables) < 10000:
        logging.error("Expected more unreachables.  Exiting.")
        raise ValueError("Unexpectedly few unreachables")

    filepath = unreachables_filepath(whichpeg)
    logging.info("Writing to %s", filepath)
    save_pickle(unreachables, filepath)

    return unreachables


def load_unreachables(whichpeg):
    unreachables_path = unreachables_filepath(whichpeg)
    logging.info("Loading unreachables from %s", unreachables_path)
    return pickle.load(open(unreachables_path, 'rb'))


def load_config(path):
    logging.info("Loading pickle file : %s", path)
    return pickle.load(open(path, 'rb'))


def check_dist2_bound(config2):
    """
    Returns number of stage2 states having dist2 greater than
    the arc length of a stage2 semicircle.

    This should be zero.

    :param config2: stage configuration object
    :return: integer
    """
    ms2d = (np.pi * STAGE2_TURN_RADIUS)  # arc length of a stage2 semicircle
    return len([1 for v in config2.itervalues() if v.dist2() and v.goal2() and v.dist2() > ms2d])



def validate_stage_print(config, disc=False, stage=None, show=22, max=100, minerr=11, debug=False):
    """
    Pretty prints selected states.

    :param config: configuration state object
    :param disc: if True, discretize the simulated state
    :param show:  maximum number of states to print to console
    :param max: maximum number of states to return
    :param minerr: only print or return values having errd > minerr
    :param debug: print entire state value with each row
    :return: dict of states
    """
    result=[]
    ii=0
    show = show if show else 0
    max = max if max else 0
    if show:
        print "======================================================================================"
        print "         state    goal1                  simulated           r      dist   dir   err "
        print "======================================================================================"
    for kk, vv in config.iteritems() :
        if stage:
            if stage==1 and not_stage1(vv):
                continue
            elif stage == 2 and not_stage2(vv):
                continue
            elif stage == 3 and not_stage3(vv):
                continue
            elif stage == 4 and not_stage4(vv):
                continue
        pp = sim_new_xy(kk, vv, discretize=disc) # simulated
        g1 = vv.goal1()
        if not g1:
            logging.warn("State is missing goal: %s, %s", kk, vv)
        errd = xy_dist((g1[0], g1[1]), (pp[0], pp[1]))
        dist = xy_dist((g1[0], g1[1]), (kk[0], kk[1]))
        vstr = vv if debug else '' # print entire value for state
        r = vv.r()
        turn = vv.type()
        if (minerr and errd<minerr) :
            continue
        if ii<show:
            print "%15s  %-15s   (%5.1f, %5.1f, %5.1f)   %5.1f   %5.1f   %2s   %4.1f   %s" \
                   % (kk, vv.goal1(),pp[0],pp[1],pp[2],r,dist,turn,errd,vstr)
        result.append((kk,pp,r,dist,dir,errd,))
        ii += 1
        if ii>=max:
            break
    return result




def validate_stage_err(config, disc=False, max=10000, stage=None):
    """
    Calculates average absolute error between goal1 and simulated path contained in state over all states.

    :param config: configuration state object
    :param disc: if True, discretize the simulated state
    :return: (average error, count, num exceeding 2.0)
    """
    start = time.time()
    PROGRESS = 200000
    todo = len(config)
    logging.info("Validating %d states", todo)
    sumerr = 0
    ii=0
    thresh=5.0
    threshcount=0
    cleaned_sumerr = 0
    for kk, vv in config.iteritems() :
        if stage:
            if stage==1 and not_stage1(vv):
                continue
            elif stage == 2 and not_stage2(vv):
                continue
            elif stage == 3 and not_stage3(vv):
                continue
        pp = sim_new_xy(kk, vv, discretize=disc)
        g1 = vv.goal1()
        if not g1:
            logging.warn("State is missing goal: %s, %s", kk, vv)
        errd = xy_dist((g1[0], g1[1]), (pp[0], pp[1]))
        sumerr += abs(errd)
        if abs(errd) < thresh:
            cleaned_sumerr += 1
        if errd > thresh:
            threshcount+=1
        ii += 1
        if ii%PROGRESS == 0 or ii==todo:
            logging.info("Processed %d   (%.1f min)", ii, (time.time()-start)/60.0)
        if max and ii>max:
            break
    print "Avg err: %.1f Count: %d  Thresh: %.1f  Threshcount: %d  Portion above thresh: %.2f  Clean Portion: %.2f " % \
          (float(sumerr) / ii, ii, thresh, threshcount, float(threshcount)/ii, float(cleaned_sumerr)/ii)
    return float(sumerr)/ii, ii, thresh, threshcount, float(threshcount)/ii, float(cleaned_sumerr)/ii



def is_stage1(vv):
    return vv.rot() is None and not vv.goal2() and vv.type() != 'qt' and vv.type() != '|' and vv.goal1()
def is_stage2(vv):
    return vv.goal2() and vv.type() != '|' and vv.goal1() and vv.rot() is None
def is_stage3(vv):
    return vv.rot() and vv.type() == 'qt' and vv.type() != '|' and vv.goal1() and vv.goal2()
def is_stage4(vv):
    return vv.type() == '|' and vv.goal1() and vv.goal2()

def not_stage1(vv):
    return vv.goal2() or vv.rot() or vv.type() == '|' or vv.type() == 'qt'
def not_stage2(vv):
    return not vv.goal2() or not vv.dist2() or vv.rot() or vv.type() == 'qt' or vv.type() == '|'
def not_stage3(vv):
    return not vv.rot() or vv.type() != 'qt'
def not_stage4(vv):
    return vv.type() != '|'

def append_to_trajectory(kk, vv, traj):
    """
    Returns list of 6-tuples,
    - next planned state
    - next simulated state
    - primary goal
    - move type
    - distance traveled
    - turn radius
    :param kk: state key
    :param vv: state value
    :param traj: list of 5-tuples
    :return:
    """
    if not traj:
        traj=[]
    pp0 = sim_new_xy(kk, vv, discretize=True)  # simulated path1
    final_goal = vv.goal1() if not vv.goal2() else vv.goal2()
    dist2 = vv.dist() if not vv.dist2() else vv.dist2()
    dist2 = int(1000*dist2)/1000.0 # reduces precision to 3 decimals
    r = vv.r()
    traj.append((vv.goal1(), pp0, final_goal, vv.type(), dist2, r))
    return traj

#import pdb # pdb.set_trace()
def validate_stage_trajectory(config, max=100, stage=None, maxd=None):
    """
    Calculates simulated and planned trajectory.

    :param config: configuration state object
    :param stage:  includes only trajectories having a stage <stage>
    :return: dict of {key => [(planned, predicted, primary, move, dist, r),...]
    """
    start = time.time()
    PROGRESS = 200000
    todo = len(config)
    logging.info("Calulating trajectories for %d states", todo)
    ii=0
    trajectories = {}
    for kk, vv in config.iteritems() :

        if maxd and vv.dist() > maxd:
            continue

        if stage and stage==1 and not is_stage1(vv):
            continue
        elif stage and stage==2 and not is_stage2(vv):
            continue
        elif stage and stage == 3 and not is_stage3(vv):
            continue
        elif stage and stage==4 and not is_stage4(vv):
            continue

        ii += 1
        if ii%PROGRESS == 0 or ii==todo:
            logging.info("Processed %d   (%.1f min)", ii, (time.time()-start)/60.0)
        if max and ii>max:
            break

        trajectories[kk] = append_to_trajectory(kk,vv,traj=[])

        if is_stage1(vv):
            continue
        gg0 = trajectories[kk][0][0]
        pp0 = trajectories[kk][0][1]
        vv0 = config[gg0]
        trajectories[kk] = append_to_trajectory(pp0, vv0, traj=trajectories[kk])
        if is_stage1(vv0):
            continue
        gg1 = trajectories[kk][1][0]
        pp1 = trajectories[kk][1][1]
        vv1 = config[gg1]
        trajectories[kk] = append_to_trajectory(pp1, vv1, traj=trajectories[kk])
        if is_stage1(vv1):
            continue
        gg2 = trajectories[kk][2][0]
        pp2 = trajectories[kk][2][1]
        vv2 = config[gg2]
        trajectories[kk] = append_to_trajectory(pp2, vv2, traj=trajectories[kk])
        if is_stage1(vv2):
            continue
        else:
            print "ERROR: one stage too far.  config[%s]=%s   :   \n\ttrajectory[%s]: %s" % (kk,vv,kk,trajectories[kk])

    return trajectories


def configpath_for_stage(whichpeg=NORTH, stage=None):
    if whichpeg==NORTH:
        filepaths = N_FILEPATHS
    elif whichpeg==NORTHEAST:
        filepaths = NE_FILEPATHS
    else:
        raise ValueError("Expected a choice of goal.")

    return   filepaths[4] if stage and stage == 4  \
        else filepaths[3] if stage and stage == 3  \
        else filepaths[2] if stage and stage == 2  \
        else filepaths[1] if stage and stage == 1  \
        else filepaths[4] if os.path.isfile(filepaths[4]) \
            else filepaths[3] if os.path.isfile(filepaths[3]) \
            else filepaths[2] if os.path.isfile(filepaths[2]) \
            else filepaths[1]

def config_for_stage(whichpeg, stage=None):
    startt = time.time()
    configpath = configpath_for_stage(whichpeg, stage)
    logging.info("Loading %s", configpath)
    config = load_config(configpath)
    logging.info("Loaded %s : %d rows  (%.1f min)", configpath, len(config), (time.time()-startt)/60.0)
    return config

def sanity_check_stage(whichpeg, stage=None, max=50):
    "Prints out some of the stage3 paths and compares simulated outcome with planned paths."
    startt = time.time()
    config = config_for_stage(whichpeg, stage)
    data = validate_stage_trajectory(config, max, stage)
    print "Calculated %d trajectories. (%.1f min) " % (len(data), ((time.time() - startt) / 60.0))
    print "=================================================================================================="
    print "state : next(planned) next(simulated)  target (goal)            move          dist             r"
    print "=================================================================================================="
    for kk, vv in data.iteritems():
        print kk,' : '
        for item in vv:
            print '\t', ' '.join(['%14s' % str(x) for x in item])

def sample_trajectories(config, file, stage=None, max=500):
    """
    Writes out some of the trajectories to file.
    File format: pickle
    Data format: dict of (xyt) key mapped to list of up to 4 5-tuples,
    {key => [(planned, predicted, primary, move, dist, r),...]

    :param file:
    :param stage: restricts trajectories to only those having this stage
    :param max:
    """
    startt = time.time()
    logging.info("Sampling %d trajectories %s", max, "from stage %s" % stage if stage else "")
    data = validate_stage_trajectory(config, max, stage)
    save_pickle(data, file)
    logging.info("Wrote %d trajectories to %s. (%.1f min) ", len(data), file, ((time.time() - startt) / 60.0))

def sample_unreachables(whichpeg, file, samplesize=20000):
    unreachables = load_unreachables(whichpeg)
    sample = set(random.sample(unreachables,samplesize))
    save_pickle(sample,file)

def sample_config(config, file, stage=None, min=10000, allgoal1=False):
    """
    Writes a sample of configuration to file, ensuring that any stage2, stage3, or stage4 states are not orphaned.
    File format: pickle

    :param file:
    :param stage:
    :param max:
    """
    startt = time.time()
    sample = dict([(k,v) for k,v in config.iteritems()][:min])
    logging.info("Sampling %d states and their ancestors.", min)
    for k,v in sample.items() : # copy contents before modifying
        goal1 = v.goal1()
        goal2 = v.goal2()
        #   Follows each trajectory towards goal
        while goal2 and goal2 not in sample:
            #  goal1 may not have an entry, but it doesn't hurt to include if it does.
            if goal1 in config:
                sample[goal1] = config[goal1]
            #  It is possible for goal2 to be a primary goal, in which case it won't have a goal2
            if goal2 and goal2 in config:
                sample[goal2] = config[goal2]
                goal1 = config[goal2].goal1()
                goal2 = config[goal2].goal2()
            else:
                break
    if allgoal1:
        #   Gets all of the stage2 goals found in sample so far
        goal1s = set([v.goal1() for v in sample.itervalues()])
        #   Gets all stages having the same goal1
        same_goal1 = dict([k,v] for k,v in config.iteritems() if v.goal1() in goal1s)
        #   Merges
        sample.update(same_goal1)

    save_pickle(sample, file)
    logging.info("Wrote %d states to %s. (%.1f min) ", len(sample), file, ((time.time() - startt) / 60.0))

def angle_diff(a1,a2):
    """
    :param a1: angle in degrees
    :param a2: angle in degrees
    :return: angle difference in degrees
    """
    return ((a1-a2) + 180) % 360 - 180


def print_trajectory_err(config,stage,samplesize,maxd):

    startt = time.time()
    if stage:
        print "Estimating trajectory error for stage %d" % stage, (", max dist %.1f" % maxd) if maxd else ''
    else:
        print "Estimating trajectory error over all paths", ("max dist %.1f" % maxd) if maxd else ''

    #    Returns a list of up to 3 3-tuples, one for each stage in the path.
    data = validate_stage_trajectory(config, max=samplesize, stage=stage, maxd=maxd)
    goal_mismatches = 0
    errd = 0 # distance error
    errt = 0 # heading error
    count = 0
    for kk, vv in data.iteritems():
        count += 1
        if vv[-1][0] != vv[-1][2]:
            goal_mismatches += 1
        errd += float(xy_dist((vv[-1][0][0],vv[-1][0][1]), (vv[-1][1][0],vv[-1][1][1])))
        h1 = vv[-1][0][2]
        h2 = vv[-1][1][2]
        errt += abs(angle_diff(h1,h2))
    print "Average distance error   : %.1f" % (float(errd)/count)
    print "Average heading error    : %.1f" % (float(errt)/count)
    if goal_mismatches > 0:
        print "Number of goal mismatches: %d" % goal_mismatches

    print "Evaluated %d trajectories. (%.1f min) " % (len(data), ((time.time() - startt) / 60.0))


def trajectory_err(whichpeg, stage=None, samplesize=100000, maxd=None):
    """
    Estimates the error between the outcome of a simulated path and the intended goal
    and prints the result to console.

    :param stage: if provided only considers paths that start from the specified stage.
    :param samplesize: max number of data to process
    :param maxd: if provided limits data to starting states within this distance from goal.
    """
    config = config_for_stage(whichpeg, stage)
    print_trajectory_err(config,stage,samplesize,maxd)


def trajectory_err_config(config, stage=None, samplesize=50000, maxd=None):
    """
    Estimates the error between the outcome of a simulated path and the intended goal
    and prints the result to console.

    :param config: configuration object
    :param stage: if provided only considers paths that start from the specified stage.
    :param samplesize: max number of data to process
    :param maxd: if provided limits data to starting states within this distance from goal.
    """
    print_trajectory_err(config,stage,samplesize,maxd)

def trajectory_err_file(path, stage=None, samplesize=50000, maxd=None):
    """
    Estimates the error between the outcome of a simulated path and the intended goal
    and prints the result to console.
    """
    configg = load_config(path)
    logging.info("Loaded %d rows", len(configg))
    print_trajectory_err(configg,stage,samplesize,maxd)


N_UNEXPLOREDS_XY_FILEPATH = "n_unexploreds_xy.pickle"
N_UNEXPLOREDS_XYT_FILEPATH = "n_unexploreds_xyt.pickle"
NE_UNEXPLOREDS_XY_FILEPATH = "ne_unexploreds_xy.pickle"
NE_UNEXPLOREDS_XYT_FILEPATH = "ne_unexploreds_xyt.pickle"
def get_and_store_unexploreds(whichpeg=NORTH, stage=4):
    """
    Loads configuration from file,
    loads unreachables from file,
    creates set of reachable state 3-tuples that are still unexplored,
    and writes to file.
    :return:
    """
    config = config_for_stage(whichpeg, stage)
    unreachables = load_unreachables(whichpeg)
    xyt_un = get_xyt_unexplored(whichpeg, config, unreachables)
    filepath=N_UNEXPLOREDS_XYT_FILEPATH if whichpeg==NORTH else NE_UNEXPLOREDS_XYT_FILEPATH if whichpeg==NORTHEAST else None
    save_pickle(xyt_un, filepath)


def get_and_store_xy_unexploreds(whichpeg, stage=4):
    """
    Loads stage4 configuration from file,
    loads unreachables from file,
    creates set of reachable xy locations that are still unexplored,
    and writes to file.
    :return:
    """
    xy_un = get_xy_unexplored_file(whichpeg, configpath_for_stage(whichpeg, stage))
    filepath = N_UNEXPLOREDS_XY_FILEPATH if whichpeg == NORTH else NE_UNEXPLOREDS_XY_FILEPATH if whichpeg == NORTHEAST else None
    logging.info("Writing xy unexploreds to %s", filepath)
    save_pickle(xy_un, filepath)

def load_xyt_unexploreds(whichpeg=NORTH):
    if whichpeg==NORTH:
        filepath=N_UNEXPLOREDS_XYT_FILEPATH
    if whichpeg==NORTHEAST:
        filepath=NE_UNEXPLOREDS_XYT_FILEPATH
    logging.info("Loading from %s", filepath)
    return pickle.load(open(filepath, 'rb'))

def load_xy_unexploreds(whichpeg=NORTH):
    if whichpeg==NORTH:
        filepath=N_UNEXPLOREDS_XY_FILEPATH
    if whichpeg==NORTHEAST:
        filepath=NE_UNEXPLOREDS_XY_FILEPATH
    logging.info("Loading from %s", filepath)
    return pickle.load(open(filepath, 'rb'))

def print_overall_stats(whichpeg, config, unreachables):
    # Prints overall plan stats
    if not config:
        config = load_config(configpath_for_stage(whichpeg))
    if not unreachables:
        unreachables = load_unreachables(whichpeg)
    print_plan_stats(whichpeg, config, unreachables)


def run_sample(whichpeg):
    startt = time.time()
    logging.info("Running sample code block")
    prefix="n_" if whichpeg == NORTH else "ne_" if whichpeg == NORTHEAST else ""
    config = config_for_stage(whichpeg, 4)
    sample_trajectories(config, prefix+"sample_traj.pickle", stage=None, max=20000)
    sample_unreachables(whichpeg, prefix+"sample_unreachables.pickle", samplesize=20000)
    sample_config(config, prefix+"sample_stage2.pickle", 2, 1000, True)
    sample_config(config, prefix+"sample.pickle")
    sample_config(config, prefix+"sample_stage4.pickle", 4, 500, True)
    save_remaining(whichpeg)
    logging.info("elapsed %.1f", (time.time() - startt) / 60.0)
    logging.info("SAMPLE DONE")


def paths_ok(config, byes, plot=False):
    """
    DEPRECATED
    :param config:
    :param byes:
    :param plot:
    :return:
    """
    logging.info("Checking %d paths", len(config))
    obstacles = xy_obstacles_set()
    ii = 0
    for key in config.iterkeys():
        ii+=1
        points = path_points(key,config,plot=False,plot_obstacles=False,verbose=False)
        for pp in points:
            if pp in byes:
                continue
            if collides(pp, obstacles):
                logging.warn("%s :  path state collides : %s", key, pp)
                logging.warn("%s", str_state(key, config[key]))
                # Plot bad
                if plot:
                    path_points(key, config, plot=True, plot_obstacles=True, verbose=False)
                return False
        if ii%1000:
            logging.info("Checked %d so far",ii)
    return True


def path_points(key,config,plot=True,plot_obstacles=True,verbose=False,dfactor=2):
    """
    WORK IN PROGRESS
    The intended result was to recreate path points encountered by a stage1 or stage2 path.
    However, this technique does a poor job of recreating the path and often overshoots,
    generating too many false collisions to be useful for plan validation.
    Probably need to use the Bresenham Circle Arc Algorithm aka midpoint circle algorithm.
    However, that cannot handle arcs smaller than octant so will also either undershoot or overshoot.
    :param key:
    :param config:
    :param plot:
    :param plot_obstacles:
    :param verbose:
    :param dfactor:
    :return:
    """
    state = config[key]
    if is_stage3(state) or is_stage4(state):
        return set()

    r = state.r()
    gridsize = state.dist2() * 2 if is_stage2(state) else state.dist() * 2
    turntype = 'lh' if state.type() == 'rh' else 'rh' if state.type() == 'lh' else None
    h = (key[2] + 180) % 360
    center = center_for_tangent((key[0], key[1]), np.radians(h), r, turntype)
    amin_, amax_ = fdl_rotation(h, turntype, portion=0.75)
    if verbose:
        print "State: ", str_state(key, state)
        print 'turntype: ', turntype
        print "h: ", h
        print "center: ", center
    arc_points = arc_grid(r, gridsize, center, amin_, amax_, counter=True if turntype == 'rh' else False, plot=False)
    path_xyh=set()
    path_xy=set()
    steps = 0
    dprev=999
    for pp in arc_points:
        steps += 1
        if state.dist2() and steps < state.dist2()/5:
            path_xyh.add(pp)
            continue
        d1 = cw_arclength((pp[0],pp[1]),(key[0],key[1]),r)
        d2 = cw_arclength((pp[0],pp[1]),(state.goal1()[0],state.goal1()[1]),r)
        if d2<d1 :
            if d2>=dprev:
                break
            else:
                dprev=d2
        h = discretize_angle(heading_tangent_circle(pp[0], pp[1], center[0], center[1], clockwise=True if turntype == 'rh' else False),
                             factor=dfactor)
        h = (h+180)%360 # reverse the heading back again
        path_xyh.add((int(pp[0]),int(pp[1]),h,))
        path_xy.add((int(pp[0]),int(pp[1]),))
    if plot:
        plot_xy(path_xy,plot_obstacles=plot_obstacles,figsize=15)
        flow_plot_xyt(path_xyh,arrowlength=2,arrowsize=3,plot_obstacles=plot_obstacles,figsize=15,max=50,verbose=False)
    return path_xyh


DEV_STAGE1_CONFIG_PICKLE_FILEPATH='/Users/mark/Valkyrie/dev_stage1.pickle'
DEV_STAGE2_CONFIG_PICKLE_FILEPATH='/Users/mark/Valkyrie/dev_stage2.pickle'
def run_dev(whichpeg=NORTH):
    " Scrathpad method for development and testing. "

    startt = time.time()
    logging.info("Running dev code block")

    logging.info("")
    logging.info("Peg: %s", args.which)
    run_stage3(args.which, 'n_sample_stage2.pickle', None)

    # trajectory_err(whichpeg, stage=3, samplesize=50000, maxd=60)


    # # Stores unexploreds to file for intermediate stage
    # get_and_store_xy_unexploreds(whichpeg, 2)
    # get_and_store_unexploreds(whichpeg, 2)

    # print_overall_stats(whichpeg, config=None,unreachables=None)

    # Prints sample of plan vs simulated paths
    # sanity_check_stage(1)

    ##
    ##  Trajectory accuracy for paths shorter than 5'
    ##

    # Stage 2 simulated error for paths within 5' of goal
    # trajectory_err(whichpeg, stage=2, samplesize=2000000, maxd=60)
    # # avg dist err: 0.7  heading: 2.5

    ##
    ##   Trajectory accuracy for all paths
    ##

    # Overall simulated error
    #trajectory_err(whichpeg, stage=None, samplesize=1000000)
    # # avg dist err: 4.2  heading: 3.7

    logging.info("elapsed %.1f", (time.time()-startt)/60.0)
    logging.info("DEV DONE")


def size_config(whichpeg, stage=None):
    config = load_config(configpath_for_stage(whichpeg, stage))
    count = len(config)
    logging.info("Count of config : %s", count)
    logging.info("Size of config  : %s  %d/item", sys.getsizeof(config), sys.getsizeof(config)/count)
    logging.info("asizeof config  : %s  %d/item", asizeof(config), asizeof(config)/count)
    # pickle_length = len(pickle.dumps(config,-1))
    # logging.info("Size of pickle  : %s  %d/item", pickle_length, pickle_length/count)

##
## Memory savings due to byte packing
##

# BEFORE
# Stage1
# 2017-03-04 03:13:30,692 - INFO - Count of config : 297659
# 2017-03-04 03:13:30,990 - INFO - Size of config  : 12583192  42/item
# 2017-03-04 03:13:46,792 - INFO - asizeof config  : 76086616  255/item
# 2017-03-04 03:14:38,718 - INFO - Size of pickle  : 52104044  175/item

# Stage2:
# 2017-03-04 02:59:41,622 - INFO - Count of config : 4790008
# 2017-03-04 02:59:41,623 - INFO - Size of config  : 201326872  42/item
# 2017-03-04 03:08:41,163 - INFO - asizeof config  : 5672576296  1184/item
# 2017-03-04 03:10:20,929 - INFO - Size of pickle  : 351530084  73/item

# AFTER
# After refactor packing state value as bytearray: from 1184/item => 218/item (81.6% reduction)
# 2017-03-05 14:59:16,775 - INFO - Loading config file from stage4.pickle
# 2017-03-05 15:00:35,592 - INFO - Count of config : 5699677
# 2017-03-05 15:00:35,594 - INFO - Size of config  : 402653464  70/item
# 2017-03-05 15:07:00,482 - INFO - asizeof config  : 1,242,873,864  218/item





#====================================================================
#
#                              Dev
#
#====================================================================
#   Tested


# Additional 15% size reduction possible (from 218 down to 186 for stage4 values)
# by packing keys as byte array wrapped in class.
#
# In [113]: asizeof((0,5,10))
# Out[113]: 152
#
# In [114]: asizeof(ByteKey((0,5,10)))
# Out[114]: 120
class ByteKey(object):
    __slots__ = ('packed')
    def __init__(self,xyt):
        self.packed = pack_int_array((xyt[0]+SHIFT, xyt[1]+SHIFT, xyt[2]+SHIFT),[2,2,2])
    def key(self):
        return (bytes_to_int(self.packed[0:2]) - SHIFT, bytes_to_int(self.packed[2:4]) - SHIFT,
         bytes_to_int(self.packed[4:6]) - SHIFT,)
    def __repr__(self):
        return 'ByteKey(%s,%s,%s)' % (self.key())


class State2(object):
    """
    Reduces memory by only 37% vs dict, but slightly more flexible than naked StateNamedtuple
    """
    __slots__ = ('goal1', 'dist', 'type', 'r', 'goal2', 'dist2', 'rot', 'samples')
    def __init__(self, goal1, dist, type, r=None, goal2=None, dist2=None, rot=None, samples=1):
        self.goal1 = goal1
        self.dist = dist
        self.type = type
        self.r = r
        self.goal2 = goal2
        self.dist2 = dist2
        self.rot = rot
        self.samples = samples
    def __repr__(self):
        return 'State2(goal1=%s,dist=%s,type=%s,r=%s,goal2=%s,dist2=%s,rot=%s,samples=%d' % \
               (self.goal1,self.dist,self.type,self.r,self.goal2,self.dist2,self.rot,self.samples)
    def __getstate__(self):
        return dict([(k, getattr(self,k,None)) for k in self.__slots__])
    def __setstate__(self,data):
        for k,v in data.items():
            setattr(self,k,v)



#====================================================================
#
#                           BYTE PACKING
#
#====================================================================
SHIFT = 1000 # shift all (x,y) coordinates by this to ensure positive
MAX_SIGNED_INT = 32767
MAX_UNSIGNED_INT = 65535
MAX_UNSIGNED_CHAR = 255
MAX_ARC_L = 509
MAX_ROWS = 100000000
NULL_XYT = (65535,65535,65535)
NULL_2BYTE = 65535
NULL_BYTE = 255

def pack_int(num, size=2):
    """
    Packs nonnegative integer into byte array of exactly <size> bytes.
    Contains all 255's if <num> negative or cannot be split.
    Packed with zeros on left as necessary to size.
    :param num: integer
    :param size: minimum number of bytes in result
    :return: byte array
    """
    if num > 256**size - 1:
        raise ValueError("Number exceeds max size for %d byte int: %s" % (size, num))
    elif num < 0:
        return bytearray([255] * size)
    bb = splitNumber(num)
    if len(bb) == 0:
        return bytearray([255] * size)
    while len(bb) < size:
        bb.insert(0,0)
    return bytearray(bb)


def splitNumber (num):
    """
    Splits arbitrary size integer into list of one byte ints
    :param num: integer
    :return: list of one byte ints
    """
    if num == 0:
        return [0]
    lst = []
    while num > 0:
        lst.append(num & 0xFF)
        num >>= 8
    return lst[::-1]

v_intsizes_23 =  [2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2] # int byte size for complete state value
kv_intsizes_29 = [2, 2, 2] + v_intsizes_23 # int byte size for key plus complete state value


def pack_int_array(int_array, intsizes):
    """
    Packs array of integers into bytearray
    :param int_array: array of one or two byte integers
    :param intsizes: bytearray size for each integer.
    :return: bytearray
    """
    packed = []
    assert(len(int_array)==len(intsizes))
    for ii in zip(int_array, intsizes):
        if ii[1] == 1:
            if ii[0] > 255:
                raise ValueError("Value big for int1: %s. \nArray: %s" % (ii[0], int_array))
            packed.extend(bytearray(splitNumber(ii[0])))
        elif ii[1] == 2:
            if ii[0] > 256**2 - 1:
                raise ValueError("Value big for int2: %s. \nArray: %s" % (ii[0], int_array))
            packed.extend(pack_int(ii[0]))
        else:
            raise ValueError("Int bytesize exceeds max: %d")
    return packed


def pack_kv_state_ints(k, v):
    """
    Packs a state (key,value) into bytearray length 29

    - key   : 3x2 byte ints = 6
    - goal1 : 3x2 byte ints = 6
    - dist  : 1x2 byte int  = 2
    - type  : 1x1 byte      = 1
    - r     : 1x2 byte int  = 2
    - goal2 : 3x2 byte ints = 6
    - dist2 : 1x2 byte int  = 2
    - rot   : 1x2 byte int  = 2
    - samples:1x2 byte int  = 2

    :param k:
    :param v:
    :return: array of integers
    """
    return [k[0], k[1], k[2]] +  pack_v_state_ints(v)


def pack_kv_bytearray(k, v):
    """
    Packs key-value state as bytearray using int sizes from kv_bytes_27
    :param k: state key triple (x,y,t)
    :param v: state value
    :return: bytearray
    """
    return pack_int_array(pack_kv_state_ints(k, v), kv_intsizes_29)


def pack_v_bytearray(v):
    """
    Packs state value as bytearray using int sizes from v_bytes_21
    :param v: state value
    :return:  bytearray
    """
    return pack_int_array(pack_v_state_ints(v), v_intsizes_23)


def pack_v_state_ints(v):
    """
    Packs a namedtuple state value into array of ints.

    - goal1 : 3x2 byte ints = 6
    - dist  : 1x2 byte int  = 2
    - type  : 1x1 byte      = 1
    - r     : 1x2 byte int  = 2
    - goal2 : 3x2 byte ints = 6
    - dist2 : 1x2 byte int  = 2
    - rot   : 1x2 byte int  = 2
    - samples:1x2 byte int  = 2
    TOTAL   : 23 bytes per row.

    :param v:
    :return:
    """
    d = v.dist
    d2 = v.dist2
    if int(d*100) >= MAX_UNSIGNED_INT or (d2 and int(d2*100) >= MAX_UNSIGNED_INT) :
        logging.error("Dist exceeds max: d:%.2f,  d2:%.2f", d, d2)
    d = int(100*v.dist)
    d2 = int(100*abs(d2)) if d2 else MAX_UNSIGNED_INT
    tt = v.type
    if v.dist2 and v.dist2 < 0 and tt != '|':
        raise ValueError("Type=%s and dist2<0 : %.2f" % (tt,v.dist2))
    g1 = v.goal1
    g1 = (g1[0]+SHIFT, g1[1]+SHIFT, g1[2]+SHIFT)
    if v.goal2:
        g2 = v.goal2
        g2 = (g2[0] + SHIFT, g2[1] + SHIFT, g2[2] + SHIFT)
    else:
        g2 = NULL_XYT
    r = int(10*v.r) if v.r else MAX_UNSIGNED_INT
    rot = int(10*abs(v.rot)) if v.rot else MAX_UNSIGNED_INT
    samples = v.samples
    # chr <==> ord
    t_ord = MAX_SIGNED_INT
    if tt == 'rh':
        t_ord = ord('r')
    elif tt == 'lh':
        t_ord = ord('l')
    elif tt == '|' and v.dist2 < 0 :
        t_ord = ord('b')
    elif tt == '|' and v.dist2 >= 0 :
        t_ord = ord('f')
    elif tt == 'qt' and v.rot < 0:
        t_ord = ord('Q')
    elif tt == 'qt' and v.rot >= 0:
        t_ord = ord('q')
    else:
        logging.error("Bad type: %s", v.type)
    return [g1[0], g1[1], g1[2], d, t_ord, r, g2[0], g2[1], g2[2], d2, rot, samples]

def bytes_to_int(ba):
    """
    Unpacks bytearray <ba> into a nonnegative integer, for bytearrays of up to length 4.
    If length not in (1,2,3,4), returns -1
    :param ba: bytearray
    :return: integer
    """
    if len(ba) == 4:
        return (ba[0] << 24) + (ba[1] << 16) + (ba[2] << 8) + ba[3]
    elif len(ba) == 3:
        return (ba[0]<<16) + (ba[1]<<8) + ba[2]
    elif len(ba) == 2:
        return (ba[0]<<8) + ba[1]
    elif len(ba) == 1:
        return ba[0]
    else:
        return -1


def pack_v23(v):
    """
    Pack namedtuple state values as 23 bytearray
    :param v:
    :return: bytearray
    """
    # logging.info("Val  : %s", v)
    packed_ints = pack_v_state_ints(v)
    # logging.info("packed ints : %s", packed_ints)
    packed_bytes = pack_int_array(packed_ints, v_intsizes_23)
    # logging.info("byte length : %d", len(packed_bytes))
    return packed_bytes


def unpack_v23(ba, intsizes=v_intsizes_23):
    """
    
    :param ba: bytearray
    :param intsizes: size of each int in byte array
    :return: State
    """
    # See also StateP
    done = 0
    ints = []
    for intsize in intsizes:
        bseg = ba[done:done+intsize]
        ints.append(bytes_to_int(bseg))
        done += intsize
    goal1 = (ints[0]-SHIFT,ints[1]-SHIFT,ints[2]-SHIFT)
    dist = ints[3]/100.0
    rawtype = chr(ints[4])
    type = 'qt' if rawtype in ('q', 'Q') else '|' if rawtype in ('f', 'b') else 'lh' if rawtype=='l' else 'rh' if rawtype=='r' else None
    r = ints[5]
    r = None if r == NULL_2BYTE else r/10.0
    goal2 = None if (ints[6],ints[7],ints[8]) == NULL_XYT else (ints[6]-SHIFT,ints[7]-SHIFT,ints[8]-SHIFT)
    dist2 = ints[9]
    dist2 = None if dist2 == NULL_2BYTE else -dist2/100.0 if rawtype=='b' else dist2/100.0 if rawtype in ['f','l','r','q','Q'] else None
    rot = ints[10]
    rot = None if rot == NULL_2BYTE else -rot/10.0 if rawtype == 'Q' else rot/10.0 if rawtype == 'q' else None
    samples = ints[11]
    return State(goal1,dist,type,r,goal2,dist2,rot,samples)


def full_config_bytes(config):  # WIP, dev
    """
    Saves 4 byte integer corresponding to number of rows then saves earch row.
    Following is saved for each row:
    - key   : 3x2 byte ints = 6
    - goal1 : 3x2 byte ints = 6
    - dist  : 1x2 byte int  = 2
    - type  : 1x1 byte      = 1
    - r     : 1x2 byte int  = 2
    - goal2 : 3x2 byte ints = 6
    - dist2 : 1x2 byte int  = 2
    - rot   : 1x2 byte int  = 2
    TOTAL   : 27 bytes per row.

    :param config:
    :return: byte array
    """
    #   Prepend byte array with 4 bytes corresponding to number of rows.
    rows = len(config)
    rows_bytes = pack_int(rows,4)
    logging.info("Rows:     %d  Bytes: %s", rows, rows_bytes)
    for k,v in config.iteritems():
        logging.info("Key  : %s", k)
        logging.info("Val  : %s", v)
        packed = pack_kv_state_ints(k, v)
        logging.info("ints : %s", packed)
        logging.info("blen : %d", len(pack_int_array(packed, kv_intsizes_29)))
        break


def recover_full_config_bytes(ba):  #  WIP, dev
    """
    Recovers state from byte array.
    - key   : 3x2 byte ints = 6
    - goal1 : 3x2 byte ints = 6
    - dist  : 1x2 byte int  = 2
    - type  : 1x1 byte      = 1
    - r     : 1x2 byte int  = 2
    - goal2 : 3x2 byte ints = 6
    - dist2 : 1x2 byte int  = 2
    - rot   : 1x2 byte int  = 2
    TOTAL   : 27 bytes per row.

    :param config:
    :return: byte array
    """
    #   Prepend byte array with 4 bytes corresponding to number of rows.
    rb = ba[0:5]
    rows = rb[0]*65536*256 + rb[1]*65536 + rb[2]*256 + rb[4]
    logging.info("Rows:     %d  ", rows)
    for row in range(0,rows):
        seg = ba[4+row*27:4+row*27+28]


#====================================================================
#
#                               main
#
#====================================================================
import gc
if __name__ == '__main__':
    logging.info("Running %s", __file__)
    start_t = time.time()
    parser = argparse.ArgumentParser(description='Simple motion planner for FIRST 2017')
    parser.add_argument('--dev','-d', action='store_true', help='Runs only dev block, then quits')
    parser.add_argument('--sample','-S', action='store_true', help='Runs only sample block, then quits')
    parser.add_argument('--stats','-s', action='store_true', help='Runs stats then quits')
    parser.add_argument('--un','-u', action='store_true', help='Calculates unreachable states.')
    parser.add_argument('--one','-o', action='store_true', help='Runs stage1')
    parser.add_argument('--two','-t', action='store_true', help='Runs stage2')
    parser.add_argument('--three','-T', action='store_true', help='Runs stage3')
    parser.add_argument('--four','-F', action='store_true', help='Runs stage4')
    parser.add_argument('--which','-w', default=NORTH, help='Can be north or northeast.')
    parser.add_argument('--discretization','-D', default=DISCRETIZATIONS, type=int, help='Can be 180 or 360.')
    parser.add_argument('--x','-x', action='store_true', help='Runs stageX')

    # parser.add_argument('--file','-f' help='Filepath')
    args = parser.parse_args()

    if args.discretization and args.discretization in [180,360]:
        if args.discretization != DISCRETIZATIONS:
            DISCRETIZATIONS = args.discretization
    else:
        raise ValueError("Unsupported discretization: %d" % args.discretization)
    logging.info("Angle discretization: %d" % DISCRETIZATIONS)

    if args.which == NORTH:
        goals = N_XY_GOALS
        filepaths = N_FILEPATHS
    elif args.which == NORTHEAST:
        goals = NE_XY_GOALS
        filepaths = NE_FILEPATHS

    if args.dev:
        run_dev()
        exit(0)

    if args.stats:
        logging.info("Calculating stats then exit")
        unreachable_states = load_unreachables(args.which)
        states = load_config(filepaths[4])
        print_plan_stats(args.which, states, unreachable_states)
        exit(0)

    unreachable_states = None
    if args.un:
        logging.info("Peg: %s", args.which)
        logging.info("Calculating unreachable states.")
        unreachable_states = run_unreachables(args.which)
        logging.info("Cumulative : %.1f min", (time.time() - start_t) / 60.0)

    if args.one:
        logging.info("")
        logging.info("Peg: %s", args.which)
        logging.info("Running stage 1 : shortest arcs")
        stage1_states = run_stage1(args.which, 15, 100, 0.5, save=True)
        logging.info("Running stage 1 : short arcs")
        stage1_states = run_stage1(args.which, 101, 200, 1, save=True, config=stage1_states)
        logging.info("Running stage 1 : medium arcs")
        stage1_states = run_stage1(args.which, 210, 500, 10, save=True, config=stage1_states)
        logging.info("Running stage 1 : long arcs")
        stage1_states = run_stage1(args.which, 600, 2000, 100, save=True, config=stage1_states)
        logging.info("Running stage 1 : longest arcs ")
        stage1_states = run_stage1(args.which, 2200, 5000, 200, save=True, config=stage1_states)
        check_collisions_config(stage1_states)

        logging.info("")
        logging.info("DONE stage 1")
        logging.info("Cumulative : %.1f min", (time.time() - start_t) / 60.0)

    if args.two:
        logging.info("")
        logging.info("Peg: %s", args.which)
        logging.info("Reading %s", filepaths[1])
        stage1_states = pickle.load(open(filepaths[1], 'rb'))
        logging.info("Running stage 2")
        run_stage2(args.which, stage1_states, filepath=filepaths[2])
        logging.info("")
        logging.info("DONE stage 2 ")
        del stage1_states
        del unreachable_states
        gc.collect()
        logging.info("Cumulative : %.1f min", (time.time() - start_t) / 60.0)

    #   TO RESUME ABORTED STAGE2
    if args.x:
        logging.info("")
        logging.info("Peg: %s", args.which)
        logging.info("RELOADING %s", filepaths[2]) # restart from previous stage2
        stage1_states = pickle.load(open(filepaths[2], 'rb'))
        logging.info("RESUMING stage 2")
        run_stage2(args.which, stage1_states, filepath=filepaths[2], d_U=81)  # <=== first dist after checkpoint
        logging.info("")
        logging.info("COMPLETED stage 2 ")
        del stage1_states
        del unreachable_states
        gc.collect()
        logging.info("Cumulative : %.1f min", (time.time() - start_t) / 60.0)

    if args.three:
        logging.info("")
        logging.info("Peg: %s", args.which)
        logging.info("Running stage 3")
        run_stage3(args.which, filepaths[2], filepaths[3])
        logging.info("")
        logging.info("DONE stage 3 ")
        logging.info("Cumulative : %.1f min", (time.time() - start_t) / 60.0)

    if args.four:
        logging.info("")
        logging.info("Peg: %s", args.which)
        logging.info("Running stage 4")
        run_stage4(args.which, filepaths[3], filepaths[4])
        logging.info("")
        logging.info("DONE stage 4 ")

    if args.sample:
        logging.info("")
        logging.info("Running sample")
        logging.info("Angle discretization: %d" % DISCRETIZATIONS)
        logging.info("Peg: %s", args.which)
        run_sample(args.which)

    logging.info("%.1f min", (time.time() - start_t) / 60.0)

# Usage:
#   Runs everything from scratch:
#   python motion_planner.py --un --one --two --three --four
#
#   Loads unreachables and stage2 from file then runs stages 3 & 4:
#   python motion_planner.py --three --four
#
#   Loads unreachables and stage3 from file then runs stages 4:
#   python motion_planner.py --four

# Stage 1: 420 min
# Stage 2: 45 min
# Stage 3: 10 min
# Stage 4: 12 min

