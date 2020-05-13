from integral_timber_joints.process.action import *
from integral_timber_joints.process.target import *
from integral_timber_joints.assembly import Assembly
from integral_timber_joints.geometry import Beam, Joint

def create_actions_from_sequence(process, verbose = True):
    # type: (RobotClampAssemblyProcess) -> None
    """ Perform path planing for one action """

    assembly = process.assembly # type: Assembly
    process.actions = []
    actions = process.actions

    for beam_id in assembly.sequence:
        beam = assembly.beam(beam_id) # type: Beam
        if verbose: print ("Beam %s" % beam_id)
        actions.append(LoadPickupBeamAction(beam))
        if verbose: print ('|- ' + actions[-1].__str__())

        # move clamps from storage to structure
        clamps = assembly.get_joints_of_beam_connected_to_already_built(beam_id)
        for clamp in clamps:
            actions.append(PickToolAction(StorageTarget(), '90lap'))
            if verbose: print ('|- ' + actions[-1].__str__())
            actions.append(PlaceToolAction(ClampTarget(*clamp)))
            if verbose: print ('|- ' + actions[-1].__str__())

            #if verbose: print ("|- Detatch Clamp at Joint %s-%s" % clamp)

        # attach gripper
        gripper_type = assembly.get_beam_attribute(beam_id, "gripper_type")
        actions.append(PickToolAction(StorageTarget(), gripper_type))
        if verbose: print ('|- ' + actions[-1].__str__())

        # pick place beam
        actions.append(PickWorkpieceAction(StorageTarget(),beam))
        if verbose: print ('|- ' + actions[-1].__str__())

        #Syncronized clamp and move beam action
        if clamps:

            #Move beam to Jaw instead of final target
            #actions.append(MoveBeamAction(beam,"BStorage","JawTarget (?)"))
            #actions.append(MoveBeamAction(beam,"BStorage", UnresolvedBeamInJawLocation(beam)))
            actions.append(FreeMoveWorkpieceAction(BeamAboveJawTarget(beam),beam))
            actions.append(LinearMoveWorkpieceAction(BeamInJawTarget(beam),beam))
            if verbose: print ('|- ' + actions[-1].__str__())

            # Syncronized clamp jaw close
            sync_action = ActionGroup()
            # Clamp close actions
            for clamp in clamps:
                sync_action.append(ClampJawCloseAction(clamp))
            sync_action.append(LinearMoveWorkpieceAction(BeamFinalTarget(beam), beam))
            actions.append(sync_action)

            # Syncronized clamp jaw open
            sync_action = ActionGroup()
            # Clamp close actions
            for clamp in clamps:
                sync_action.append(ClampJawOpenAction(clamp))
            # TO DO: Need to add gripper open and retract action.
            #sync_action.append(LinearMoveWorkpieceAction(BeamFinalTarget(beam), beam))
            actions.append(sync_action)

            if verbose: print ("|- Close Clamp Jaws while moving beam")
            for action in sync_action:
                if verbose: print ("  |- " + action.__str__())

            actions.append(DetachWorkpieceAction(BeamFinalTarget(beam),beam))
            if verbose: print ('|- ' + actions[-1].__str__())

        else:
            #actions.append(MoveBeamAction(beam,"BStorage",  UnresolvedBeamFinalLocation(beam) ))
            actions.append(PlaceWorkpieceAction(BeamFinalTarget(beam),beam))
            if verbose: print ('|- ' + actions[-1].__str__())


        # attach gripper
        actions.append(PlaceToolAction(StorageTarget()))
        if verbose: print ('|- ' + actions[-1].__str__())


        # remove clamps from structure to storage

        for clamp in clamps:
            #actions.append(AttachClampAction("UnassignedClamp", False, location = UnresolvedClampLocation(*clamp)))
            actions.append(PickToolAction(ClampTarget(*clamp), '90lap'))
            if verbose: print ('|- ' + actions[-1].__str__())
            #actions.append(DetachClampAction("UnassignedClamp", True, location = UnresolvedStorageLocation()))
            actions.append(PlaceToolAction(StorageTarget()))

            if verbose: print ('|- ' + actions[-1].__str__())


if __name__ == "__main__":
    # Load process
    import jsonpickle
    json_path = "tests/SequencingScene_01_mm.json"
    f = open(json_path, 'r')
    json_str = f.read()
    process = jsonpickle.decode(json_str, keys=True)
    f.close()

    # From process.sequence, create actions (high level actions)
    create_actions_from_sequence(process)

    # Loop through each action, create movements (low level movements)

    # Loop through each movement create trajectory (ROS backend)

    # Save process

    # Result Goal 1: Visualize action, movement, trajectory by visualizing scene objects and robots.

    # Result Goal 2: Execute movements

    pass
