# Multi-stage planner 
## Using backwards planning and forwards planning using only circular arcs, straight line, and quickturn maneuvers.

This gives a path to goal for over 99.6% of the reachable configurations using splines comprised of only circular arcs, straight lines, and rotations.

### Sample trajectories found by the planner:
https://github.com/pluteski/first2017mp/blob/master/sample_trajectories.ipynb

## Motion Planning
Planning is the process of computing several steps of a problem-solving procedure before executing. 

Motion planning in our go-to-goal driver assist problem is a process of breaking down the desired movement task into discrete motions that navigate a route towards a goal state.  In our case the desired movement task is to move the robot from its current location and heading to a location very near a peg, ending up in an orientation that allow the robot to easily deposit the gear onto the peg. Motion planning can use both forward and backward search. Backwards planning is often infeasible in many motion planning problems due to combinatorial explosion of the search space.  However, we were able to reduce the search space to make it feasible to compute in reasonable time offline. The "state" we used is defined by (x,y,h), where (x,y) is the two-dimensional location, and h is the heading. 

## Discretization
The state space is discretized into one inch cells, with the number of possible headings within each cell limited to 180.  (We experimented with 60 and 360 discretizations of the heading and found 180 to be optimal for our needs).

## Collision masking
The first step before beginning the planning process is to load a collision mask. This is a set of states (x,y) for which there is a fixed obstacle such as a wall or divider. We also calculate the set of theoretically reachable states, comprised of states (x,y,h) for which the robot does not intersect with the collision mask.

## Stage 1 backwards planner
The first stage of the planner begins at a goal state given by (x,y,h) corresponding to a (x,y) location near a peg with heading h that allows the robot to easily deposit the gear onto the corresponding peg. Then, the backwards planning algorithm simulates the effect of backing the robot away from the goal on a circular arc of radius r and having either a left-hand (counterclockwise forwards motion) or right-hand (corresponding to or clockwise forwards motion). In so doing the planner is exploring the solution space. While simulating this backwards motion away from the goal, the simulated robot encounters other states (x',y',h') on the playing field. For each (x',y',h') encountered, the planner stores an entry in a table.  This entry is stored with key (x',y',h'), and having the following entries:

* turn type : circular left hand, circular right hand
* radius : turning radius of the turn
* distance : the length of the circular arc that connects (x',y',h') with the goal (x,y,h)

If the planner encounters a collision with a fixed obstacle, it halts its search along the current arc.

This is repeated for many radii r over one turn type (say, right hand), and then is repeated again over the other turn type (left hand). 

This is an optimizing planner. If a better path is found it is used.  If a state (x',y',h') is encountered that has been solved previously, it will be overwritten with the new solution if the overall distance to goal would be improved. 

This is repeated again for several small perturbations of (x,y,h), because the robot can be slightly off to either side of the peg and still be able to easily deposit the gear on the peg.  Likewise, the robot can be oriented at an angle that is slightly off from perpendicular to the peg. 

The radii r that are explored range from the shortest turning radius of the vehicle (in our case half the width of the robot, or about 15 inches) to long turning radiuses that are essentially very slightly curved straight lines (e.g., a turning radius of 5000 inches). 

A turning radius r=15 is the shortest turning radius that the robot can perform aside from a "quickturn" which is our term for a maneuver where the robot spins on its vertical axis, changing its heading without moving forwards or backwards from its (x,y) location.

At the completion of stage 1, backwards planning using this approach finds single state solutions to between 3% and 4% of the states. 

## Stage 2 backwards planner
The second stage performs a similar backwards planning exploration as the first state, except that now instead of using the peg to determine the goal state, stage 2 explores backwards from each of the states that were solved in stage 1.  Also, instead of exploring a wide range of turning radii, stage 2 only uses a single turning radius r=15, which is the shortest turning radius that the robot can perform aside from a quickturn. It explores both left handed and right handed circular turns with this turning radius r=15.

Again, as a backwards planner this is simulating what would happen if the robot backed away from each (x',y',h') solved in stage 1, using either a left-handed turn of radius 15, or a right-handed turn of radius 15, traveling backwards along 50% of the arc length.  

I call these fleur-de-lis turns, because the stage 2 paths explored along a single stage 2 path resemble a fleur-de-lis pattern.

Stage 2 needs to save a couple more fields in its table entry (x'',y'',h'').  One is the (x',y',h') of its goal state.  The other is the total distance to the primary goal (x,y,h).  The primary goal (x,y,h) is the target of the stage 1 path that is stored at (x',y',h'). The total distance is the sum of the distance traveled by the stage 2 path and the stage 1 path.

If the stage 2 planner encounters a state for which there is already a stage 1 path to goal, it is never overwritten.  If it encounters a state for which there is already a stage 2 path to goal, it will overwrite it with the newly discovered path only if the new total distance to goal is less than the previous distance to goal.

Stage 2 is able to solve another 80% of the states in this way, such that when it finishes, we end up with a solution for 84% of the states, leaving 26% of the states as yet unsolved. This could be improved somewhat by increasing the number of radii used in each stage; instead, we opted for a stage 3 using an even simpler maneuver. 

## Stage 3 forwards planner
Stage 3 is actually a forwards planner, because it is starting from an unsolved configuration state, and looking for one nearby that has a solution that it can follow into the goal. It turns out that at this point in the planning process, a large portion of the reachable (x,y) locations have already been visited, but there are many (x,y,h) yet unsolved.  Stage 3 exploits this fact.  

At a given (x'',y'',h''), the stage 3 forwards planner looks for all entries in the current table (x,y,h) having (x,y)=(x'',y'',h'').  Because the table only contains entries for solved states, this means that there is a path to goal at (x,y,h).  Because (x,y)=(x'',y''), this also means that the robot can reach the solved state by performing a quickturn.  There might be many possible solutions (x'',y'',h) for various values of h.  We could optimize this step by choosing the one requiring the smallest amount of quickturn rotation, or by choosing the one having the smallest total distance to the primary goal.  Quickturns are not difficult to perform accurately. We opted to optimize on total distance to primary goal.

Once stage 3 is completed the planner typically has solved over 98% of the states.  This leaves less than 2% of the reachable states yet to be solved. 

## Stage 4 forwards planner
Stage 4 exploits a property of the remaining reachable states. Typically there will be a solved state that can be accessed simply by moving in reverse or driving forwards.  This is especially suitable for handling states where the robot is up against an obstacle.  The heading and location (x,y,h) is such that the state was not reached by the previous three stages, and is typically such that it would be unreachable by the previous stages even if they utilized a more granular search strategy. 

Stage 4 is also a forwards planner. It examines each of the reachable states that remain to be solved. It performs a Bresenham line search in the backwards direction but along the same heading until an obstacle is encountered, tallying all of the solved states it encounters along the way.  It performs the same operation for the forwards direction.  Then, it selects the solved state having the best total distance to primary goal. This straight line maneuver is stored as the stage 4 path.

At completion of stage 4, typically over 99.6% of the reachable states have an entry in the solution table. The remaining states are typically ones where the robot is in a configuration that does not collide with an obstacle but would be impossible to reach using normal kinematics attainable by the expected driving behavior.

## Result
The end result is a lookup table containing an entry for almost all of the states that the robot is likely to encounter within striking distance of a goal, each entry providing a trajectory of up to four maneuvers:

1. circular arc 
   * left hand or right hand turn with radius r ranging from 15 to 5000 inches
2. circular arc
   * left hand or right hand turn with radius 15
3. quickturn
   * rotation R, remaining at the same location (x,y)
4. straight line maneuver
   * from (x,y,h), moving a distance along the same heading h but to a different (x',y')
   * forwards or reverse

The trajectory for an initial state (x',y',h') gives a short route to the primary goal (x,y,h). If the trajectory has four maneuvers, the fourth stage maneuver (the straight line maneuver) would be executed first, then the quickturn, then the fleur-de-lis turn, then the circular arc into goal.

For further detail on the algorithm see the code :
https://github.com/pluteski/first2017mp/blob/master/motion_planner.py

## Simulations

The discretization unavoidably introduces error into the pathfinding, even before the errors introduced by real world considerations such as measurement error, motor control error, and collisions. The following iPython notebook simulates pathfinding accuracy.  

https://github.com/pluteski/first2017mp/blob/master/test.ipynb

It turns out that the closer one is to the goal, the better the accuracy, as expected. In practice the robot would deal with this by continually revising its path as it approaches the goal, re-evaluating its present location and heading when convenient to do so, and querying the lookup table for the revised location.


