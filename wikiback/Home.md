Welcome to the first2017mp wiki!

![Stage3 paths found for northeast peg.](https://github.com/pluteski/first2017mp/blob/master/images/stage3_paths_ne.png)


# Origin Story
This began as an exploratory motion planning problem posed by [Valkyrie Robotics](http://valkyrierobotics.com/), a [FIRST Robotics](https://www.firstinspires.org/) team, for the 2017 competition.

See for example this [robot cam view](https://www.youtube.com/watch?v=qnRytiFeSoM) of a gear handling robot.

The challenge is to automatically find a path to goal from an arbitrary position and heading within striking distance of the goal. 

This could be used to provide driver assist by moving the robot towards the goal along an efficient trajectory unless overridden by the driver.


# Ending
As many FIRST robotics teams will attest, some designs don't make it into the competition robot even after much investment. Likewise, the code in this repo was not deployed in the competition robot for the 2017 season because of competing priorities and resource constraints.  It turned out that it wasn't possible to implement the required dependencies.  This approach required a state estimation system that could determine the position and heading of the robot on the field based on data obtained from several sensors, including vision, gyro, accelerometer, and wheel encoders. However, in the grand scheme of things the cost of this exploratory exercise was minimal. Whereas other teams invested thousands of dollars into hardware and associated control systems while tying up multiple team members only to scrap the feature before competition, the cost of this investigation was primarily coding time by a single individual and some additional time spent running and evaluating simulations.  It was instructive as to what could be achieved with circular arcs and quickturns*, which was useful knowledge for the driver team to have, especially because the motor control system of the robot was designed around circular arcs and quickturns. 

# About what happened in between
See [first2017mp/wiki/Background](Background) for background, summary results, and pointers to additional visualizations and simulation results.

See [first2017mp/wiki/RelatedApproaches](https://github.com/pluteski/first2017mp/wiki/RelatedApproaches) for approaches that are suitable for real-time motion-planning.

* A "quickturn" maneuver is a rotation about the robot vertical axis, changing the heading without changing the (x,y) location.
