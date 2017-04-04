

## What am I?
This is an exploration of simple motion planning applied
to the FIRST 2017 playing field.
It generates a lookup table that provides a trajectory of up to
four paths using circular arcs, quickturns and straight lines.
It demonstrates that a car-like robot can drive to
a peg using only these moves for up to 99.6% of states.
A 'state' (x,y,h) here positions the location (x,y) and heading h
on a playing field discretized into one inch squares with
headings discretized into 180 or 360 angles.

This is strictly a geometric planner, meaning that it does not provide
motor control signals, only a geometric route from current position
and heading to a desired goal state, which for the 2017 competition
is one of three pegs mounted on lifts on the airship near the
alliance station. A prior step is required to
estimate the current position and heading.
A subsequent step would be required to convert
these paths into motor control signals.

See sample_trajectories.ipynb for visualizations of
typical trajectories and motion_planner.py for the main code.


## Usage

#### North goal, start to finish
$ python motion_planner.py -D=180 -w=north --un --one --two --three --four

This generates the following files:

- n_unreachables.pickle
- n_stage1.pickle
- n_stage2.pickle
- n_stage3.pickle
- n_stage4.pickle

The n_stage4.pickle is a stage configuration table containing
trajectories of up to four moves.  The other stage files are checkpoint
output from previous stages.


#### Northeast goal, start to finish
$ python motion_planner.py -D=180 -w=northeast --un --one --two --three --four

This generates the following files:

- ne_unreachables.pickle
- ne_stage1.pickle
- ne_stage2.pickle
- ne_stage3.pickle
- ne_stage4.pickle

In shorter command option notation, as a backgrounded no hangup run :
$ nohup python motion_planner.py -D=180 -w=northeast -uotTF > ne_mar9.log &


#### Rerunning north goal on stage4 only
$ python motion_planner.py -D=180 -w=north --four

This requires the following files already exist:
- n_unreachables.pickle
- n_stage3.pickle

And creates or overwrites the following file:
- n_stage4.pickle


#### Generating sample files
The state configuration files for stages two and above
(stage2, stage3, stage4) can be hundreds of megabytes large,
so for convenience the following generates smaller samples for
development and testing.

$ python mp.py -D=180 -w=north --sample

This generates the following files:
- n_sample_unreachables.pickle
- n_sample.pickle  
- n_sample_stage2.pickle  
- n_sample_stage4.pickle
- n_remaining.pickle
- n_sample_traj.pickle  

The n_sample_traj.pickle file contains a sample of trajectories,
each trajectory containing from one to four paths.  See the code
for details.

#### Loading output files
The following command demonstrates how to load and manipulate
a state configuration file.

$ python load_mp.py -f ne_sample.pickle

#### Calculating statistics

The --stats argument displays statistics then exits.

$ motion_planner.py --stats -w=northeast

There are additional methods for evaluating trajectories.

# Contact
![Email contact](https://github.com/pluteski/first2017mp/blob/master/images/pluteski_email.png)
