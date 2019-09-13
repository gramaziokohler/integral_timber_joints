===============================================================
Robotic Movements
===============================================================

The pose to approach a Rack can be hard programmed to a deterministic pose to ensure repeatability and avoid any collision possibility.

1. Pick up clamp and place it on timber
========================================
*Clamps are stored in a Jaw open state*

Transit
...........

Robot - freespace move - from previous location to clamp rack

On Clamp Rack
..............

Robot - cartesian move - approach clamp target

Robot - lock tool (tool = clamp) - io

Robot - cartesian move - leave clamp target - cartesian

Robot - open clamp gripper - io

======= ================================= ===================================
Robot   cartesian move                    approach clamp target
Robot   lock tool (tool = clamp)          io
Robot   cartesian move                    leave clamp target - cartesian
Robot   open clamp gripper                io
======= ================================= ===================================


Transit
...........

Robot - freespace move - from clamp rack to assembly target

On Assembly Area
....................

Robot - cartesian move - approach assembly target

Robot - io - close clamp gripper

Robot - io - unlock tool (tool = clamp)

Robot - cartesian move - leave assembly target


2. Remove used clamps and place it on rack
========================================
*Clamps are in Jaw Closed state attached on timber*

Transit
...........

Robot - freespace move - from previous location to assembly target

On Assembly Area
.................

Robot - cartesian move - approach assembly target

Robot - io - lock tool (tool = clamp)

Clamp - rapid move - open jaw

Robot - io - open clamp gripper

Robot - cartesian move - leave assembly target

*(Note: If the next assembly uses the same clamp, than obviously do not unload the clamp.)*

Transit
...........

Robot - freespace move - from assembly target to clamp rack

On Clamp Rack
..............

Robot - io - close clamp gripper \\
Robot - io - unlock tool (tool = clamp) \\
Robot - cartesian move - leave clamp rack \\
(Clamps are stored in a Jaw open state)


Load a timber gripper
========================================

(Assuming Robot does not have tool in hand) \\
(Assuming gripper is stored in opened jaw state) \\

Transit
...........
Robot - freespace move - from previous location to gripper rack \\

**On Gripper Rack**
Robot - cartesian move - approach gripper target \\
Robot - io - lock tool (tool = timber gripper)\\
Robot - cartesian move - leave gripper target \\
Robot - io - open timber gripper jaw (This is necessary to assert pneumatic pressure to the system)\\

Unload a timber gripper
========================================

Transit
...........
Robot - freespace move - from previous location to gripper rack

On Gripper Rack
................

Robot - cartesian move - approach gripper target \\
Robot - io - remove pneumatic supply to tool \\
Robot - io - unlock tool (tool = timber gripper)\\
Robot - cartesian move - leave gripper target \\
(Assuming gripper is stored in opened jaw state)


Pick up a timber element and place it into Clamp Jaws
========================================

(Assuming Robot have the correct timber gripper at hand) \\
(Assuming timber gripper jaw is opened)

Transit
...........
Robot - freespace move - from previous location to timber stack

On Timber Stack
................
Robot - cartesian move - approach timber stack \\
Robot - io - close timber gripper jaw \\
Robot - cartesian move - leave timber stack

Transit
...........
Robot - freespace move - from timber stack to assembly area

On Assembly Area
................

Robot - cartesian move - approach assembly target \\
Robot - io - open timber gripper jaw \\
Robot - cartesian move - leave assembly target

Close Clamp Jaws (Robot not moving)
========================================

Clamp - coordinated move - close jaw


Close Clamp Jaws (Robot holding and moves with timber)
=======================================================

Transit
...........

Robot - freespace move - from previous location to timber stack

On Timber Stack
................

Robot - cartesian move - approach timber stack \\
Robot - io - close timber gripper jaw \\
Robot - cartesian move - leave timber stack

Transit
...........
Robot - freespace move - from timber stack to assembly area \\
(Until this point, routine is same as "Pick up a timber element and place it into Clamp Jaws")

On Assembly Area
................

Robot - cartesian move - approach assembly target \\
Robot \& Clamp - cartesian move \& coordinated move - move timber while closing jaw
Robot - io - open timber gripper jaw \\
Robot - cartesian move - leave assembly target
