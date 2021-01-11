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

```bash
# from integral_timber_joints root dir
python .\external\compas_fab_pychoreo\examples\itj\run.py -v
```
