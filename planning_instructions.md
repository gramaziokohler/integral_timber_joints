# Sequence and Motion Planning

<!-- We use `compas_fab_pychoreo` for motion planning and environment management. -->
(TODO: more descriptions)

## Update submodules

Since our planning packages are always under continuous development, please do the following to update the submodules whenever you pull from `integral_timber_joints` to make sure that the submodules are up-to-date:

```
git pull --recurse-submodules
git submodule update --remote --recursive
```

If you are using a GUI tool for git, there should be already an easy way to do this. 
Search `git submodule update + <your GUI tool>` will usually give you the right answer.

## Examples

Run planning:

```bash
# from any dir, with the proper conda env is activated
python -m integral_timber_joints.planning.run --step_sim --watch --seq_i 2 --diagnosis --write
```

Checking states:

```
python -m integral_timber_joints.planning.check_states  --problem_subdir results -p pavilion_process.json --verify_plan --id_only A181_M0 --viewer
```
