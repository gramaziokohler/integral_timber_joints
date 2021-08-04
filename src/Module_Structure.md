# Module Structure

The structure of integral_timber_joints library

## src

The src folder contains the main `integral_timber_joints` module and other helper modules. They can installed 'in-one-go' with the default pip install.

Note: This is the base folder path that is added to Python Path (for Rhino too)

```
─── src
    ├── integral_timber_joints
    │   ├── geometry  (Geometry classes)
    |   |   ├── beam.py
    |   |   ├── beamcut.py
    |   |   ├── joint.py
    |   |   ├── griphole.py
    |   |   ├── env_model.py
    |   |   ├── screw.py
    |   |   └── ...
    │   ├── assembly  (Assembly classes - a network of beams and joints)
    |   |   └── assembly.py
    │   └── process  (Process Classes - clamp assembly process related classes)
    |       ├── robot_clamp_assembly_process.py
    |       ├── actions.py
    |       ├── movements.py
    |       ├── pathplanner.py
    |       └── algorithms.py
    ├── compas_trimesh (Boolean Helper - Still a better alternative to compas_cgal)
    |   └── boolean.py
    ├── CV2_Animation (Image to Animation tool)
    └── geometric_blocking (Blocking Direction Calculation Helper)

```

## src\integral_timber_joints\geometry

This folder contains geometrical classes. An un-cut beam can be modelled by a `Beam` object. A beam with joints or other machined features are modeled at the `Assembly` level, which maintains the relationship of each `Beam` object with its list of features acting as subtractive geometry. The following feature objects exist:

- Joint (`Joint_90lap`, `Joint_halflap`, `Joint_non_planar_lap`)
- Screw Hole (Stored as a property of `Joint`)
- `Beamcut` (Planar cut at start or end of beam)

Each feature object belongs to only one beam, if two beams intersect to create a joint, each of the beams will have its own joint feature.

### Intrinsic and Extrinsic Geometric Properties

Both the (1) geometry of the beam and (2) the location of the beam within an assembly are stored within the Beam Class. This is largely inherited from BTL data format.

The geometry and location of a feature (such as `Joint_halflap`) in WCF can be dependent on the `Beam` or it can be independent (for example a beam cut defined from a plane in WCF, note that this is not implemented). This can be implemented in whichever way necessary.

All features object implements `get_feature_meshes(BeamRef)` which returns the negative solid model representing the subtractive machining. The `BeamRef` is referring to the `Beam` of which the feature belongs to. This allows the feature to get properties from the parent Beam to construct geometry.

### Clamps and Screwdriver Attachment

Joints that can be clamped or screwed will return a list of `Tools` that can be used to assemble the joint. It will also return a list of possible attachment location for the tool. For screwed joint, it returns the type of `Screw` used.

### Flip-able Joints

Some joints (e.g. `Joint_halflap`) which implemented `swap_faceid_to_opposite_face()`can be flipped to the other side with its neighbor, `Assembly.flip_lap_joint` performs a coordinated flip for both beams at the same time.



## src\integral_timber_joints\assembly

`Assembly` class is a subclass of `compas.Network` and provides functions to create an assembly of Beams.

- Keep track of `Beam` neighbor relationships and their connecting `Joints`.
- Keep track of assembly sequence.
- Stores attributes for individual `Beams` and `Joints`

### Beam Attributes

- Robotic assembly related key frames (storage, pickup_approach, pickup, in_clamp, final ...)
- Robotic assembly related vectors (such as retract, approach, pickup ...)
- Robotic tools related properties (such as which gripper to use, grasp face ...)
- Beam end cuts (they are a type of features contributing to the Processing)

### Joint Attributes

- Robotic Clamp related properties (such as which tool_type to use, tool_id if assigned)
- Robotic manipulation of clamps (storage, attach_approach, detach_approach ...)

### Assembly Class Functions

- Add or remove `Beam`, `Joint`, `Beamcut`.
- Modify assembly sequence
- Flip pair of joints
- Transform `Beam`, `Joint`, `Beamcut` within a world coordinate frame in coherent manner.
- Detect collision between `Beams` and create `Joints` (full automatic creation or with human in the loop selection)
- Creating a Boolean model of `Beam` with its features (`Joints`, `Beamcuts`, Pin holes from clamps, grippers)
- Compute possible assembly direction(s) from joints with neighbors

## src\integral_timber_joints\process

The `RobotClampAssemblyProcess` class (abbreviated to `Process` class) contains all the information related to the robotic assembly of an `Assembly`. It stores all the `Tool(s)` available such as `Clamp(s)`, `Screwdriver(s)`, `Gripper(s)` and `Toolchanger`. It also contains a `RobotWrist` object for disembodied robot visualization. It can optionally store a `RobotModel` of the assembly robot (typically the RFL model, often referred to as `Robot`) for visualization purpose, although this will not be serialized.

Robotic assembly related objects are also stored, such as `PickupStation`, `BeamStorage` and `EnvironmentModel`. These are defined by user using the Rhino interface and are used to compute the storage location, pickup frame and to avoid collision with assembly environment.

### Action and Movement Planning

The `Process` class contains functions to compute `Actions` and `Movements` for planning the assembly process. Actions are high level abstractions of an action, that may require more than one Movement to complete. This can be roughly divided to `OperatorAction` and `RobotAction`, `RobotAction` is further divided into:

- `AttachToolAction` (include `PickClampFromStorageAction`, `PickGripperFromStorageAction`, `PickClampFromStructureAction`)
- `DetachToolAction` (include `PlaceClampToStorageAction`, `PlaceGripperToStorageAction`, `PlaceClampToStructureAction`)
- `AttachBeamAction` (`BeamPickupAction`)
- `DetachBeamAction` (include `BeamPlacementWithoutClampsAction` and `BeamPlacementWithClampsAction`)

`Movements` are low level (almost 1-to-1 equivalent) representation of a movement that is performed:

- by an operator (such as `OperatorLoadBeamMovement`)
- by the assembly robot (`RoboticMovement`)
- by the robotic tools (such as `ClampsJawMovement`)
- by some IO control (such as `RoboticDigitalOutput`)
- by a sync movement of the assembly robot and the tools (such as `RoboticClampSyncLinearMovement`)

### Planning Scene and Intermediate State

Typically the assembly planning process are linearly occurring in the following order:

1. List of `Actions` are computed from Assembly `Sequence`
2. Optimization can be performed on the `Actions` (such as reordering and to remove redundant actions)
3. `Tools` are assigned to the `Actions`
4. List of `Movements` computed from the list of Actions
5. List of `SceneState` computed after every Movement
6. `State` and `RoboticMovement` pairs are passed to motion planner to create `RobotTrajectory` for `Robot`

A `SceneState` dictionary (in form of `dict[str, ObjectState]`) keep track of the static properties (such as object location `Frame` and `RobotConfigutation` for `Robot` and `Tools`) of all the mutable objects in the planning scene at one single moment. It is a static snap shot of the scene. The only objects not included in the `SceneState` are `EnvironmentModels` because they are fixed and cannot be moved.

The initial `SceneState` is defined as:

- `Robot` with `RobotConfiguration` at initial state.
- `Tool Changer` Attached to the `Robot` flange.
- Robotic `Tools` location at `ToolStorage`,  `RobotConfiguration` at initial state.
- `Beams` location at `BeamStorage`

After every `Movement`, one or more objects (`Beams`, `Tools` and `Robot`) in the scene *maybe* changed, therefore starting from the `InitialState`, an intermediate State is created after every Movement. The intermediate states contain an update location of objects based on the `Movement` . However, one important attribute – `RobotConfiguration` of the assembly `Robot` – cannot be deduced from the Movement alone, the `RobotConfiguration` is the result of running the motion planner for the given `RobotMovement`.

If we plan the `RoboticMovements` in the same sequence in the same order of the list of `Movements`, we could fill in the `RobotConfiguration` one after another, using the ending state of one `Movement` as the beginning state of the next `Movement`.  However, as explained later, this is not realistic. It is necessary to plan in non-sequential order and a different approach of managing the state is needed.





