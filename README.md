# integral_timber_joints

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

## Installation

Install this library from source by cloning this repo to local and install from source. Run this code in terminal from the root folder of this repo.

```
pip install -e .
```

Alternatively if development is intended you can install with developer tools:

```
pip install -r requirements-dev.txt
invoke add-to-rhino
```

### Python dependency

Install the following python dependency (pip install)

- compas
- compas_fab
- jsonpickle
- trimesh

Run the following code in terminal to add them to Rhino Grasshopper Python library path:

```
python -m compas_rhino.install -p compas compas_fab compas_rhino jsonpickle trimesh integral_timber_joints
```

### Software dependency

Install the following software:

- Rhino V6 (Design interface for demo file)
- Grasshopper plugin
  - [Elefront](https://www.food4rhino.com/app/elefront) (Handles clickable mesh interface)

- [Openscad](https://www.openscad.org/downloads.html) (This will be running in the background. Start the software at least once after installing)



Credits
-------------

This repository was created by Pok Yin Victor Leung <leung@arch.ethz.ch> [@yck011522 ](https://github.com/yck011522) at [@gramaziokohler](https://github.com/gramaziokohler)



