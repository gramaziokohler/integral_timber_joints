# integral_timber_joints

<a href="https://github.com/compas-dev/compas" rel="compas">![compas](https://img.shields.io/badge/compas->=1.0,<2.0-blue)</a>
<a href="https://github.com/compas-dev/compas_fab" rel="compas_fab">![compas_fab](https://img.shields.io/badge/compas__fab->=0.16,<0.17-ff69b4)</a>

Python library for designing timber structures with integral timber joints.

This repo is part of the [Robotic Assembled Timber Structures with Integral Timber Joints](https://github.com/gramaziokohler/integral_timber_joints) project. 

## Design Goals

The goal of the python library is to be able to achieve the following high-level functions: 

- Classes and Data Structure for describing:
  - Timber elements
  - Integral joints (Subtractive machining joint)
  - Process tools (Grippers, Clamps, Pickup Station)
  - Process action (Actions and Movements)
- Algorithms for assembly design
  - Creating beams from Rhino geometry
  - Creating joint pairs from beam intersection
- Algorithms for process design
  - Computing assembly process information (assembly sequence, gripper type, clamp type, grasp pose, clamp attachment pose)
    - From user input as guide
    - Automatic deduction (Auto Sequence, Auto Lap Joint Direction)
  - Computing robotic movements
    - Key positions
    - Robotic Path (Path Planning)

## Installation (library)

### Conda environment (optional)

Create a new environment with python 3.7 and install this package in the new environment

```bash
conda create --name itj python=3.7
conda activate itj
```

One of the dependency from `compas` is planarity, which depends of `cython`. You will need to install this manually:

```bash 
pip install cython --install-option="--no-cython-compile"
```

or 

```bash
conda install cython
```

### Clone and update submodules

Install this library from source by cloning this repo to local and install from source. 

```bash
git clone --recursive https://github.com/gramaziokohler/integral_timber_joints.git
cd integral_timber_joints
```

The `--recursive` flag when cloning above is used for initializing all the git submodules. You can learn more about submodules [here](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

Later in the development, whenever you need to update the submodules, issue the following:

```bash
git submodule update --init --recursive
```

### Install libraries

Run this the following in terminal from the root folder of this repo. 

```bash
pip install -e .\external\pybullet_planning
pip install -e .\external\compas_fab_pychoreo
pip install -e .
```

The first two lines will first install the two submodule libraries from source, and the last line will install `integral_timber_joints` from source (all in the [editable mode](https://pip.pypa.io/en/stable/reference/pip_install/#install-editable)).

<!-- Note that there maybe error message from `pip` indicating version incompatibility of `compas` with `compas_fab`, this is fine.  -->

Alternatively if development is intended you can install with developer tools:

```bash
# replace `pip install -e .` above with
pip install -r requirements-dev.txt
invoke add-to-rhino
```

#### Python dependencies, explained

The following python dependency are installed in the above process. See [requirements.txt](https://github.com/gramaziokohler/integral_timber_joints/blob/master/requirements.txt) for more info on the specific versions pinned.

The following libraries are installed "automatically" (via pip or conda, learn about their difference [here](https://www.anaconda.com/blog/understanding-conda-and-pip#:~:text=Pip%20installs%20Python%20packages%20whereas,software%20written%20in%20any%20language.&text=Another%20key%20difference%20between%20the,the%20packages%20installed%20in%20them.))
- `compas` (Library for geometrical modeling and graph relationship)
- `compas_fab` (Library for modelling Tools and Robots, functions to call ROS backend for path planning)
- `jsonpickle` (Serialization library for Tools, Assembly, Process)
- `trimesh` (Call Openscad in the background to perform Mesh Boolean)
- `pycddlib` (Computation of blocking direction analysis)

The following two libraries are installed manually from git submodules, to better keep track of our development cycles:
- `pybullet_planning` (Library for planning utilities in pybullet)
- `compas_fab_pychoreo` (Library for providing `pybullet_planning` functionalities through a `compas_fab`-friendly API)

## Installation (external software dependency)

Install the following software:

- Rhino V6 (Design interface for demo file)
- Grasshopper plugin
  - [Elefront](https://www.food4rhino.com/app/elefront) (Handles clickable mesh interface)

- [Openscad](https://www.openscad.org/downloads.html) (This will be running in the background. Start the software at least once after installing)

## Installation (for use in Rhino)

Run the following code in terminal to add them to Rhino Grasshopper Python library path:

```
python -m compas_rhino.install -p compas compas_fab compas_ghpython roslibpy compas_rhino jsonpickle integral_timber_joints geometric_blocking compas_trimesh
```

## Module Structure

See [this file](src/integral_timber_joints/Module_Structure.md)

## Sequence and Motion Planning

See [this file](./planning_instructions.md) for instructions on running the planning.

Credits
-------------

This repository was created by Pok Yin Victor Leung (<leung@arch.ethz.ch>) [@yck011522](https://github.com/yck011522) at [@gramaziokohler](https://github.com/gramaziokohler), with the help of other [contributors](https://github.com/gramaziokohler/integral_timber_joints/blob/master/AUTHORS.rst).



