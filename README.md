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

## Installation (library)

[optional] Create a new environment with python 3.7 and install this package in the new environment

```
conda create --name itj python=3.7
conda activate itj
```

Install this library from source by cloning this repo to local and install from source. Run this code in terminal from the root folder of this repo. Note that there maybe error message from `pip` indicating version incompatibility of `compas` with `compas_fab`, this is fine. 

```
cd C:\Users\leungp\Documents\GitHub\integral_timber_joints
pip install -e .
```

Alternatively if development is intended you can install with developer tools:

```
pip install -r requirements-dev.txt
invoke add-to-rhino
```

### Python dependency

The following python dependency are automatically installed in the above process.

- compas (Library for geometrical modeling and graph relationship)
- compas_fab (Library for modelling Tools and Robots, functions to call ROS backend for path planning)
- jsonpickle (Serialization library for Tools, Assembly, Process)
- trimesh (Call Openscad in the background to perform Mesh Boolean)
- pycddlib (Computation of blocking direction analysis)

## Installation - Software dependency

Install the following software:

- Rhino V6 (Design interface for demo file)
- Grasshopper plugin
  - [Elefront](https://www.food4rhino.com/app/elefront) (Handles clickable mesh interface)

- [Openscad](https://www.openscad.org/downloads.html) (This will be running in the background. Start the software at least once after installing)

## Installation (for use in Rhino)

Run the following code in terminal to add them to Rhino Grasshopper Python library path:

```
python -m compas_rhino.install -p compas compas_fab roslibpy compas_rhino jsonpickle integral_timber_joints geometric_blocking
```

## Module Structure

See [this file](src/integral_timber_joints/Module_Structure.md)

Credits
-------------

This repository was created by Pok Yin Victor Leung <leung@arch.ethz.ch> [@yck011522 ](https://github.com/yck011522) at [@gramaziokohler](https://github.com/gramaziokohler)



