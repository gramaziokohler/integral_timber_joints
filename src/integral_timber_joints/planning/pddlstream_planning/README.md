## Run

```
p -m integral_timber_joints.planning.pddlstream_planning.run --write --design_dir 220407_CantiBoxLeft --problem CantiBoxLeft_process.json

p -m integral_timber_joints.planning.run --design_dir 220407_CantiBoxLeft --problem CantiBoxLeft_process.json --rrt_iterations 400 --solve_timeout 3600 --write
```

## Assumptions

1. All beam and tools grasps are predetermined and fixed, no need to sample.
2. Each object's potential poses in the world is a fixed set (usually just `rack` and `goal` poses) and predetermined, except when it's attached to the robot and its poses can be determined by applying the `grasp` transformation to the robot's tool pose.

## Caveats

1. We haven't modelled second-level grasp, e.g. screwdrivers attached to the beam, where the beam is attached to the robot. Thus, all the screwdrivers' poses are not modelled yet and their collision geometry is not correctly represented in the planning.

## Useful PDDLStream example

- [discrete tamp](https://github.com/caelan/pddlstream/blob/master/examples/discrete_tamp/domain.pddl) example.
- [semantic attachment](https://github.com/caelan/pddlstream/blob/master/examples/continuous_tamp/idtmp/run.py).

## Logs

When not adding `AtRack` to the goal explicitly, the planner can find a decent plan without returning any clamp to the racks at all.
Thus, we should just manually concatenate `return_clamp_to_rack` actions in the end, according to the final state.

`operator_attach_screwdriver`'s `(Attached ?tool ?tool_grasp)` effect is giving the search a hard time.

### pyplanners

## ff-Eager

    Iterations: 16357 | State Space: 18133 | Expanded: 18132 | Generations: 18132 | Heuristic: 0 | Time: 233.209
    Summary: {complexity: 0, cost: 88.000, evaluations: 161, iterations: 1, length: 88, run_time: 233.362, sample_time: 0.000, search_time: 233.362, solutions: 1, solved: True, timeout: False}

Before adding the poses and grasps in:

	Iterations: 1349 | State Space: 1173 | Expanded: 1172 | Generations: 1172 | Heuristic: 0 | Time: 7.551
	Plan operators # 84
