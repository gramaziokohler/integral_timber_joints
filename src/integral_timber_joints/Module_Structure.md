# Module Structure

The structure of integral_timber_joints library

```
─── src
    ├── integral_timber_joints
    │   ├── geometry  (basic geometry classes)
    |   |   ├── beam.py
    |   |   ├── joint.py
    |   |   ├── joint90lap.py
    |   |   ├── griphole.py
    |   |   └── ...
    │   ├── assembly  (Assembly classes - a network of beams and joints)
    |   |   └── assembly.py
    │   └── process  (Process Classes - clamp assembly process related classes)
    |       ├── robot_clamp_assembly_process.py
    |       ├── actions.py
    |       ├── movements.py
    |       ├── pathplanner.py
    |       └── algorithms.py
    ├── compas_trimesh (Boolean Helper - No Longer Used)
    |   └── boolean.py
    └── geometric_blocking (Blocking Direction Calculation Helper)
        └── utils.py
```

