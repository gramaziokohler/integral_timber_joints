from copy import copy
from integral_timber_joints.planning.visualization import color_from_object_id

# assembly_wcf_final, assembly_wcf_pickup
def set_beam_frames(client, process, beam_ids, frame_name='assembly_wcf_pickup', scale=1e-3):
    for beam_id in beam_ids:
        beam_frame = copy(process.assembly.get_beam_attribute(beam_id, frame_name))
        beam_frame.point *= scale
        # * set pose according to state
        client.set_object_frame('^{}$'.format(beam_id), beam_frame, options={'color': color_from_object_id(beam_id)})

def get_place_element_gen_fn(client, process, robot,
    collisions=True, allow_failure=False, verbose=False, precompute_collisions=False, teleops=False):

    def place_element_gen_fn(beam_id, assembled=[], diagnosis=False):
        # * set assembled beams using fluent facts
        set_beam_frames(client, process, assembled)

    return place_element_gen_fn
