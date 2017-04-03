# **FIRST 2017 Motion Planning** for go to goal driver assist

## Findings
This research project explored means of pre-calculating the motion planning task related to placing a peg once the goal was in view of the robot vision system. The vision system was able to determine position relative to the goal based on reflector strips situated on either side of the goal. 

The approach explored here involved precalculating the trajectories and storing them in a lookup table to be queried at run time.  Simulations showed that this motion planning task could be achieved by a combination of circular arcs, quickturns, and straightline reverse maneuvers for over 99.6% of the initial positions.  

* _The following figure shows 7 sample trajectories found by the planner. Each trajectory has up to four segments, or "stages". Each arrow gives the robot heading at the beginning or end of a stage of the trajectory. The goal is located at (x,y) = (0,0). Straight line maneuvers are represented by a dashed blue line. Quickturn maneuvers are indicated by an arrow changing direction while remaining at the same location._ 
![Figure 00. Examples of paths found by the planner.](https://github.com/pluteski/first2017mp/blob/master/images/stage3_paths.png)

## Alternative Approaches
These maneuvers could be calculated in real-time, a la [A Path Following a Circular Arc](http://rossum.sourceforge.net/papers/CalculationsForRobotics/CirclePath.htm). With a bit more coding effort and computing power one could implement [A Path Based on Third-Degree Polynomials](http://rossum.sourceforge.net/papers/CalculationsForRobotics/CubicPath.htm). More sophisticated approaches circa 2015 are surveyed nicely in [Real-time motion planning methods for autonomous on-road driving: State-of-the-art and future research directions](http://www.sciencedirect.com/science/article/pii/S0968090X15003447).

## Motivation
This was initially intended as an offline planner to discover optimized paths based on third-degree polynomials. In the course of exploring that avenue we found that quadratic curves were adequate. We ended up utilizing only circular arcs, quickturns, and straightline paths, which sufficed to cover over 99.6% of the state space.

## Benefits
Although real-time calculation confers some advantages over a table lookup, some possible advantages of using the precalculated planning approach explored here are the following:
* it provides the ability to comprehensively validate trajectories and optimize their path lengths in advance. 
* it requires less on-board computing power.
* it can be useful for validating a real-time approach in simulations.  
* it can be used as a fallback system for a real-time approach.

The remainder of this document will address the offline planning approach using a combination of backwards and forwards planning stages in a discretized setting.

# Background and Operation
The go-to-goal objective is to be able to provide driver assist when within striking distance of the peg and either assist the driver or take over completely in guiding the robot to a configuration near the peg such that the robot can easily place the gear on the peg. 

Given a map, initial location and heading, goal location, and cost function, the task is to provide the low cost path to goal for all valid initial locations.  Each trajectory can be converted into motion control signals. 

The table is queried by providing the current location and heading (x, y, h).  The query returns a trajectory comprising a sequence of from one to four maneuvers that lead to a goal state. An example of a four-step trajectory might be: (1) back up 11.3 inches along the current heading, (2) quickturn 54 degrees, (3) move forwards for 24 inches along a right hand (clockwise) circular arc at a turning radius of 15 inches, (4) move forwards for 42 inches along a left hand (counterclockwise) circular arc of turning radius for 55 inches. The table can be re-queried along the way for improved accuracy.

The task of converting said trajectory into drivetrain control signals is not addressed here, would be a separate and subsequent step, and is out of the scope of this document.

# Development
Paths are discovered using a optimizing four-stage planner comprised of two backwards planning stages followed by two forwards planning stages. The geometric representations of the trajectories include gentle arcs (e.g. Sun et al., 2014), tight turns, (cf. McNaughton et al., 2011 and Gu et al., 2013), quick turns, and straight line forward and reverse maneuvers. The plan table uses a 4D-configuration space including 2D position, heading and steering angle (cf.  Rufli and Siegwart (2010).  

This can be viewed as a simplified form of a state lattice based planner using a globally fixed state lattice, static state-space and time, and single resolution lattice (cf. Wang et al., 2011, Hardy and Campbell, 2013 and Kala and Warwick, 2013). 

# Implementation Steps
1. Discretized the configuration space into one inch squares and 180 or 360 degree headings.
1. First stage backwards planner connects goal states with all other states that can be achieved in a single constant curvature path.
1. Second stage backwards planner uses space-filling arcs to create tributaries of shorter paths that feed into the paths discovered in stage 1.
1. Mask obstacles and unreachable states.
   * 45% states were found to be unreachable
   * This reduced the time required to run stage 1 planner by 60% 
   * This also reduced table size by 45%
1. Implemented the second state planner 
   * Second stage is able to find paths for 60% of the reachable states. 
1. Implemented third stage forward planner using quickturn
   * This stage is able to find paths for 80% of the remaining 40% of unsolved states.
   * This brings the total coverage to 94%
1. Run path validity checker over all paths in table
   * Avg error is within a few inches over all paths, and less than an inch for paths less than 5’ from goal.
1. Ensure that every path leads to a goal stage
1. Implemented fourth stage forwards planner to handle remaining tight spots
   * This stage handles cases where the robot is nose against an obstacle.  It backs up or drives forwards in a straight line until it encounters a previously filled state. 
   * This brings total coverage to 99.6%
1. Configure planner to robot and playing field specifications
   * Set collision map and goal state to cad specs
   * Moved robot origin from nose to robot center (TBD)
1. Create iPython notebooks for simulating planner stages and visualizing results.
   * Collision map
   * Field drawings
   * Example trajectories
   * Evaluation and simulated test results
1. Reran planner for the side peg 


## Figure 0. Top view of one half of an airship. 
The red line corresponds to the lift and peg nearest the Alliance Station.  The blue lines correspond to the pegs on the other two lifts. The black diagonal lines between the pegs are the dividers.
![Figure 0. Top view of one half of an airship.](https://github.com/pluteski/first2017mp/blob/master/images/top_view_airship.png)

## Figure 1. Nested circular arcs  
This visualization helped us to explore the idea behind this approach in the early investigative stages. The blue rectangle gives the outline of the robot.  The red line is the goal peg.  The diagonal line is the divider, which is one of the obstacles. Upper border is the Alliance Station, which is where the drivers are situated.
![Figure 1. Circular arc paths discovered by the first stage planner.](https://github.com/pluteski/first2017mp/blob/master/images/nested_arcs.png)

## Figure 2. Sample of paths discovered by stage 1 planner
This illustrates a set of paths discovered by the stage 1 backwards planner, all following a single arc.
![Figure 2. Sample of paths discovered by stage 1 planner, all following a single arc.](https://github.com/pluteski/first2017mp/blob/master/images/example_stage1_arc.png)

## Figure 3: fleur-de-lis pattern used in stage 2 planner. 
A robot near a first stage plan (depicted by the gold arrow) but pointing up, east, or south can smoothly join the direct path to goal found in stage 1 by using a right hand turn (the red path).  Ones lying below the first stage path and pointing down, east, or north can join it using a left turn (the blue path).
![Figure 3: fleur-de-lis pattern used in stage 2 planner.](https://github.com/pluteski/first2017mp/blob/master/images/stage2_fleur_de_lis.png)

## Figure 4a: Unreachable state due to collision with field wall.
![Figure 4a: example of an unreachable state due to collision with field wall.](https://github.com/pluteski/first2017mp/blob/master/images/collision_wall.png)

## Figure 4b.  Unreachable state due to collision with divider.
![Figure 4b.  Example of unreachable state due to collision with divider.](https://github.com/pluteski/first2017mp/blob/master/images/collision_divider.png)

## Figure 4.  Sample of stage1 states taken from a planning run. 
Each of these states gives a path along the shortest distance circular arc leading directly towards the goal.
![Figure 4.  Sample of stage1 states](https://github.com/pluteski/first2017mp/blob/master/images/stage1_state_flow.png)

## Figure 5.  Small sample of stage 2 paths from a training run. 
Stage 2 paths use a tight turn to join a stage 1 path (stage 1 path not show here). The turn radius is selected to be 15”, which is the smallest radius turn where the wheels on one side are in the same location before and after the maneuver. 
[https://github.com/pluteski/first2017mp/blob/master/images/stage2_flow.png](Figure 5.  Small sample of stage 2 paths from a training run.)

## Figure 6. Another sample of stage2 paths from a planning run.
All of these paths use a tight right hand turn (15” radius) taking the robot to a point where there is already a left hand turn available from stage1 that takes robot directly to a goal orientation.
![Figure 6. Another small sample of stage2 paths](https://github.com/pluteski/first2017mp/blob/master/images/stage2_flow2.png)

## Figure 7. Sample of states take from a planning run. 
Two fleur-de-lis tributaries discovered by stage2 planner lead into a stage1 path that connects directly to a primary goal. Each of the tributaries follows a 15” right and left turn respectively, leading into the stage1 path, thereafter following a left hand turn having radius 51.0” the remainder of the way to goal.
![Figure 7. Two fleur-de-lis tributaries discovered by stage2 planner lead into a stage1 path](https://github.com/pluteski/first2017mp/blob/master/images/stage2_paths.png)

## Figure 8. Examples of single stage paths found by the planner. 
![Figure 8. Examples of single stage paths found by the planner. ](https://github.com/pluteski/first2017mp/blob/master/images/stage1_paths.png)

## Figure 9. Examples of 2 stage paths found by the planner. 
Some of the stage2 paths are 15” turns and the others are quick turns. A quick turn can be spotted by looking for an arrow that stays in the same location but swivels to a new heading.
![Figure 9. Examples of paths found by the planner having two stages. ](https://github.com/pluteski/first2017mp/blob/master/images/stage2_paths_2.png)

## Figure 10. Examples of 3 stage paths found by the planner.
![Figure 10. Examples of paths found by the planner having three stages.](https://github.com/pluteski/first2017mp/blob/master/images/stage3_paths.png)

## Figure 11. Examples of 4 stage paths found by the planner. 
Note use of reverse to back away from the wall. Also note the path near goal that backs away from the goal to find a better path to goal.
![Figure 11. Examples of four stage paths found by the planner.](https://github.com/pluteski/first2017mp/blob/master/images/stage4_paths.png)


# Compute time and memory
1. Stage 1 takes about 7 hours.
1. Stage 2 takes about 1 hour.
1. Stage 3 takes about 45 minutes
1. Stage 4 takes about 20 minutes

The application can be comfortably run on a machine having eight gigabytes of memory. 

# Summary of results
The planner is able to cover 99.6% of the state space using combinations of circular arcs, quickturns, and straightline maneuvers, including reverse direction.  

# Lookup Table Size
The size of the configuration space is the number of grids in the discretized x,y plane times the number of angles in the discretized circle. Suppose the virtual playing field is 10’ x 10’, discretized into 1” square cells, and the bearing (the angle measured in degrees relative to the optimal goal configuration) is discretized into 60 allowable values.  

NOTE : following needs to be updated to reflect actual playing field size and actual angle discretization used.  

The number of configurations is 1 (cell/inch^2) x 6.5 (feet) x 12 (inch/foot) x 13.5 (feet) x 12 (inch/foot) * 360 (degrees/circle) = 4,635,720.  

This is the number of rows in the lookup table. 

Each row contains:
* Key: x, y, heading (2 short ints, 1 int) = 4 bytes
* Path: 
  * Turn type : char = 1 byte
      * Four turn types: lh, rh, straightline, quickturn.
  * r : turning radius (2 byte float)
  * rotation : for quickturn (2 bytes)
  * Key : key of next waypoint = 4 bytes
  * Dist : distance to next waypoint or goal ( 2 bytes)

15 bytes * 5.5 million rows =  83 MB x 2 tables = 176 MB total.

This would need to utilize two tables, one for the north peg closest to alliance station, and one for the two side pegs.


# Results
See file [results.txt](https://github.com/pluteski/first2017mp/blob/master/results.txt) in this github repo for most recent full results.

See the IPYNB files (e.g., [sample_trajectories.ipynb] in this github repo for additional visualizations (e.g., [sample_trajectories.ipynb](https://github.com/pluteski/first2017mp/blob/master/sample_trajectories.ipynb)) and simulation results (e.g., [test.ipynb](https://github.com/pluteski/first2017mp/blob/master/test.ipynb)).

# Example summary stats from an actual run
`$ python motion_planner.py -w=north -D=180 --stats`  
`2017-02-20 18:21:32,355 - INFO - Running motion_planner.py`  
`Loading config file from stage4.pickle`  
`Loading unreachables from unreachables.pickle`  
`2017-02-20 18:25:37,333 - INFO - Total number of reachable states:      2498840`  
`2017-02-20 18:25:44,046 - INFO - Num stage1 paths :   130239`  
`2017-02-20 18:25:44,046 - INFO - Num stage2 paths :  1413357`  
`2017-02-20 18:25:44,046 - INFO - Num stage3 paths :   797128`  
`2017-02-20 18:25:44,046 - INFO - Num stage4 paths :   192883`  
`2017-02-20 18:25:46,423 - INFO - Num stage1 / lh  :   127000`  
`2017-02-20 18:25:46,423 - INFO - Num stage1 / rh  :     3239`  
`2017-02-20 18:25:50,197 - INFO - Num stage2 / lh  :   651182`  
`2017-02-20 18:25:50,197 - INFO - Num stage2 / rh  :   762175`  
`2017-02-20 18:25:53,092 - INFO - Num stage3 / lh  :   797128`  
`2017-02-20 18:25:53,092 - INFO - Num stage3 / rh  :        0`  
`2017-02-20 18:25:55,902 - INFO - Num stage4 / fwd :     1832`  
`2017-02-20 18:25:55,902 - INFO - Num stage4 / rev :   191051`  
`2017-02-20 18:25:57,336 - INFO - Number of states having a path:        2533607`  
`2017-02-20 18:25:57,336 - INFO - Number of these that are unreachable:        0 (Should be 0)`  
`2017-02-20 18:25:59,619 - INFO - Number reachable states lacking path:     7977`  
`2017-02-20 18:25:59,619 - INFO - Percent reachable states having path:     99.7`  
`elapsed 4.5`  

