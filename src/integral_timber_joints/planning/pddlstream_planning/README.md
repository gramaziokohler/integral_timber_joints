## Installations

The only installation we need to do is to compile the C++ codebase of `download`, used in `pddlstream`.

1. Open `Developer PowerShell for VS 2019` (`VS 2015/2016` should work as well) by typing it in the search bar.
2. Activate the intended conda environment
3. `cd` into the source directory of `integral_timber_joints`
4. Issue `python .\external\pddlstream\downward\build.py` and wait for the compilation to finish.
5. Close the terminal and you are good to go!

## Run

```
python -m integral_timber_joints.planning.pddlstream_planning.run --write --design_dir 220407_CantiBoxMid --problem CantiBoxMid_process.json

python -m integral_timber_joints.planning.run --design_dir 220407_CantiBoxMid --problem CantiBoxMid_process.json --rrt_iterations 400 --solve_timeout 3600 --write
```

## Verify Process

Checking collisions among attachments and environment meshes:
```
python -m integral_timber_joints.planning.check_states --design_dir 220407_CantiBoxMid --problem CantiBoxMid_process.json
```

Checking IK feasibility:
```
python -m integral_timber_joints.planning.check_ik --design_dir 220407_CantiBoxMid --problem CantiBoxMid_process.json
```