## Assumptions

1. All beam and tools grasps are predetermined and fixed, no need to sample.
2. Each object's potential poses in the world is a fixed set (usually just `rack` and `goal` poses) and predetermined, except when it's attached to the robot and its poses can be determined by applying the `grasp` transformation to the robot's tool pose.

## Useful PDDLStream example

- [discrete tamp](https://github.com/caelan/pddlstream/blob/master/examples/discrete_tamp/domain.pddl) example.
- [semantic attachment](https://github.com/caelan/pddlstream/blob/master/examples/continuous_tamp/idtmp/run.py).
