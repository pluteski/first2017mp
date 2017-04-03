We studied other approaches before settling on the one we used. Several of these are implemented in the python library Open Motion Planning Library (OMPL).  cf. http://ompl.kavrakilab.org/OMPL_Primer.pdf . 

[Real-time motion planning methods for autonomous on-road driving: State-of-the-art and future research directions, 2015](http://www.sciencedirect.com/science/article/pii/S0968090X15003447) surveys approaches used in autonomous vehicles and nicely illustrates the underlying planning approaches.

Cf. [www.kavrakilab.org/robotics](http://www.kavrakilab.org/robotics) for additional tutorial treatments, particularly [From High-Level Tasks to Motion Planning](http://www.kavrakilab.org/publications/plaku2008from-high-level-tasks.pdf). 

Most of the most advanced approaches we found that are actually used in car-like autonomous vehicles rely on real-time motion-planning, which was out of scope for this first-year robotics team. We decided to delve into a so-called "single-query" approach that precalculates a lookup table in advance instead.  This was more likely to be something that could actually be integrated into the final competition robot, and in the case that that was not possible the simulations would be informative to driving strategy. 

Motion planners generally fall into two types: geometric and control-based. 

A **geometric planner** only accounts for the geometric constraints of the system. It is assumed that any feasible path can be turned into a dynamically feasible trajectory [2]. Some geometric planners will also account for kinematic constraints.

A **control-based planner** is suitable if the system under consideration is subject to differential constraints. To generate motions these planners rely on state propagation rather than simple interpolation [2].

Quoting [1] :

“ The overall objective is to enable robots to automatically plan the low-level motions needed to accomplish assigned high-level tasks.” 

“Solution paths obtained in simulation by motion planners that do not take into account robot dynamics but consider only geometric models cannot be easily followed by the robot in the physical world. It is in general difficult and an open problem to design feedback controllers that can convert a geometric solution path into low-level hardware commands that enable the robot to follow the geometric path and thus accomplish the assigned task in the physical world.”

It makes sense to integrate the two models somehow. Either the high-level and low-level models are integrated offline, so that the desired motion control trajectory (the sequence of motor control instructions) can be queried in real-time. Or, the robot needs to be able to convert a geometric trajectory into the sequence of motor control instructions. 

In the first stage, motion planning takes place in the simplified high-level and discrete geometric model. In the second layer, motion planning is based on the full low-level model of the robot and the physical world. The high-level discrete planning in the first layer guides the low-level motion planning in the second layer during the search for a solution, greatly reducing the search space and making it more feasible to utilize a control-based planning planner.

# Problem statement
To simplify the problem we consider the two-dimensional go-to-goal objective of moving the robot from an arbitrary position on the field to a goal in a configuration that allows it to easily deposit the gear on the peg.  Given an x,y coordinate, with some bearing θ , where the goal is placed at coordinate x=0, y=0, provide a trajectory that moves the bot to a configuration (x,y,θ) that lies within some predefined range of locations and angles. For now let θ=0 be defined as the angle where the bot is facing the peg head on.

Further, the configuration space, including obstacles are fixed.  This means that this assumes that other than the fixed obstacles (walls, goal dividers) there is a clear path towards goal, or, that the path can be recalculated on the fly after colliding with moving obstacles such as stray objects (balls, gears) or other robots.  

# Geometric models
The two geometric models that seem to be most popular in actual use are PRM and RRT.  The RRT seems to be favored where a path needs to be calculated in real-time.  The PRM seems to be favored for storing trajectories in a model that can be queried quickly. 

These are more suitable for single-query application but still instructive because some of the same concepts are involved during planning and optimization stages.

## Probabilistic Roadmap (PRM)
The PRM is suitable for multi-query planning: build a roadmap of the entire environment that can be used for multiple queries.  The roadmap can be precalculated offline and then referenced for real-time motion planning. 

This approach decouples collision checking and planning, by “first sampling the configuration space. Each sample corresponds to a possible placement of the robot in the workspace. If the placement does not result in a collision, then the sample is considered valid and it is added to the roadmap.” [1]  This sample can be based on a simple uniform distribution.

During a second step, neighboring roadmap samples are connected via simple paths that avoid collisions with obstacles. 

Finally, for each initial configuration the motion-planning problem is then solved by using graph search to find a path between the initial and goal configurations. 

## The RRT and EST
These are space-filling trees that rapidly find a path to goal while avoiding obstacles. The rapidly exploring random tree (RRT) “pulls” the tree toward the unexplored parts of the configuration space by extending the tree toward random samples. The expansive space tree (EST) “pushes” the tree to unexplored parts of the configuration space by sampling points away from densely sampled areas. RRT and EST can be combined with PRM and are useful for control-based planning, even if dynamic pathfinding isn’t required, because these make PRM much more efficient.

See for example : http://ompl.kavrakilab.org/planners.html#control_planners

# Control-based Models
Solution paths obtained by motion-planning methods that solve the generalized mover’s problem may not be easily executed by the robot in the physical world [1].

We shall assume that motions of our robot are governed by dynamics that impose constraints on curvature.  This simplifies matters by ignoring velocity and acceleration.  

“The execution of a solution path obtained in simulation requires the design of feedback controllers that can convert the simulated motions into low-level hardware commands. The design of feedback controllers is a laborious and challenging task, since it depends on the robot dynamics and the interaction of the robot with the environment.” [1]

So that the produced motions obey the physical constraints of the robot, we either need to have feedback controllers that can easily follow simple geometric paths, or else we need to incorporate robot dynamics directly into motion planning.  I think there is a happy medium here where we ignore some dynamic (most aspects of acceleration and velocity) while enforcing some kinematics strictly (curvature) and heavily penalizing others that we suspect are wasteful (reversing direction).

The configuration space is augmented with new parameters necessary to express the robot dynamics. Motion planning then takes place in this augmented configuration space. Such trajectories are generally computed by propagating the robot dynamics forward in time. 

“It is typical for sampling-based planners to spend more than 90% of their computation performing forward propagation” [3]. 

KPIECE is a tree-based algorithm specifically designed for use with physics-based simulation. It reduces runtime and memory requirements by making better use of information collected during the planning process to decrease the amount of forward propagation the algorithm needs.

# See also: Q learning
We investigated Q learning. This approach was rejected after a cursory analysis of the state configuration space size indicated that this would computationally intractable for this problem statement given available computing resources.

# See also: Flow path
We also investigated using flow paths.  We rejected this approach because the lookup table size was estimated to be far too big for a car-like robot having up to 360 possible headings. This also requires an almost continuous awareness of the location and heading, which would have been nontrivial for this robot and playing field setup.


# REFERENCES:
1. http://www.kavrakilab.org/publications/plaku2008from-high-level-tasks.pdf

2. http://ompl.kavrakilab.org/planners.html

3. http://www.kavrakilab.org/robotics/kpiece.html : tutorial with some intuitive visuals


