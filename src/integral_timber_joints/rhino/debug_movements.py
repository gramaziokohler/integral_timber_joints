import Rhino  # type: ignore
import rhinoscriptsyntax as rs
import scriptcontext
import re
from integral_timber_joints.rhino.load import get_process, get_process_artist, process_is_none
from integral_timber_joints.geometry import JointHalfLap
from integral_timber_joints.rhino.assembly_artist import AssemblyNurbsArtist

from compas_rhino.utilities import clear_layer, delete_objects, draw_mesh

if __name__ == '__main__':
    process = get_process()

    for movement in process.movements:
        print(movement.movement_id)
        action = process.get_action_of_movement(movement)
        beam_id = process.get_beam_id_of_movement(movement)
        print (" - %s , %s" % (action.seq_n, beam_id))

    # movement = process.get_movement_by_movement_id("A19_M0")
    # print(movement.movement_id)
    # action = process.get_action_of_movement(movement)
    # beam_id = process.get_beam_id_of_movement(movement)
    # print (action.__str__())
    # print (" - %s , %s" % (action.seq_n, beam_id))