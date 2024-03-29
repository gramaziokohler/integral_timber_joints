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

if you use PowerShell, the variables can be supplied in the beginning before running the following commands:

```powershell
# from any dir, with the proper conda env is activated
$design_dir = "220407_CantiBoxRight"
$problem = "CantiBoxRight_process.json"
```



### Pre Planning and Planning

PDDL Planning (Will archive and overwrite original problem json file in the design folder.)

```
p -m integral_timber_joints.planning.pddlstream_planning.run --write --design_dir $design_dir --problem $problem
```



To plan everything of a process file:

```
python -m integral_timber_joints.planning.run --problem $problem --design_dir $design_dir --reinit_tool --write
```

To plan a single `seq_i` with UI

```bash
python -m integral_timber_joints.planning.run --problem $problem --design_dir $design_dir --reinit_tool --write --solve_timeout 21600 --rrt_iterations 1000  --seq_n 30
```

### Re-Planning

Re-plan a single `seq_id:`

```
python -m integral_timber_joints.planning.run --problem $problem --design_dir $design_dir --reinit_tool --write --solve_timeout 21600 --rrt_iterations 1000  --seq_n 30
```

Re-plan a single `beam_id`:

```
python -m integral_timber_joints.planning.run --problem $problem --design_dir $design_dir --reinit_tool --write --solve_timeout 21600 --rrt_iterations 1000  --beam_id b10
```

Re-plan a single `movement_id`:

```
python -m integral_timber_joints.planning.run --problem $problem --design_dir $design_dir --reinit_tool --write --solve_timeout 21600 --rrt_iterations 1000  --solve_mode movement_id --movement_id A167_M0
```

Intense re-plan for a set of `movement_id`:

```
python -m integral_timber_joints.planning.run_global --design_dir 220407_CantiBoxMid --problem CantiBoxMid_process --write --solve_timeout 21600 --rrt_iterations 1000 --mesh_split_long_edge_max_length 50 --movement_id A69_M0,A123_M0,A149_M0,A205_M1
```

**Draw exploration** in viewer while planning a single Free Movement by `movement_id` in a detailed way (with long edge subdivision)

```
p -m integral_timber_joints.planning.run --problem $problem --design_dir $design_dir --reinit_tool --write --solve_timeout 21600 -v --debug --draw_mp_exploration --step_sim --watch --diagnosis --buffers_for_free_motions --solve_mode movement_id --movement_id A235_M0 --mesh_split_long_edge_max_length 150 
```

### Post Planning 

Check planned trajectory of entire Process File 

```
python -m integral_timber_joints.planning.check_states --problem $problem --design_dir $design_dir --verify_plan --traj_collision --mesh_split_long_edge_max_length 100
```

Generate a **statistic** file of all planned result

```
python -m integral_timber_joints.planning.statistics --problem CantiBoxRight_process.json --design_dir 220407_CantiBoxRight
```



### **High accuracy collision check**

Using **check_states** to check with higher mesh resolution

```
p -m integral_timber_joints.planning.check_states --problem $problem --design_dir $design_dir --verify_plan --traj_collision --movement_id A15_M0 --mesh_split_long_edge_max_length 100
```

Using **check_states  to visualize** a problematic trajectory that was flagged with `traj_polyline_collision`

```
python -m integral_timber_joints.planning.check_states --problem $problem --design_dir $design_dir --reinit_tool --debug --viewer --mesh_split_long_edge_max_length 100 --verify_plan --traj_collision --movement_id A139_M0
```

Re-planning one move with **higher mesh resolution** to avoid collision

```
p -m integral_timber_joints.planning.run --problem $problem --design_dir $design_dir --write --movement_id A15_M0 --mesh_split_long_edge_max_length 100 --solve_mode movement_id
```
