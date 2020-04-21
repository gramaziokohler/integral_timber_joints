from integral_timber_joints.geometry.beam import Beam


import jsonpickle # JsonPickle to Serialize
import os   # Save file location
import tempfile # Save file location


def test_beam_save_load_file():
    beam = Beam.debug_get_dummy_beam(include_joint=False)

    # JsonPickle to Serialize
    jsonpickle.set_encoder_options('simplejson', sort_keys=True, indent=4)
    jsonpickle.set_encoder_options('json', sort_keys=True, indent=4)

    # Save to file location
    tempfile_location = os.path.join(tempfile.gettempdir(), "beam.json")

    # Save to file
    w = open(tempfile_location, 'w')
    w.write(jsonpickle.encode(beam))
    w.close()

    # Load Beam back
    r = open(tempfile_location, 'r')
    beam_loaded = jsonpickle.decode(r.read())

    assert jsonpickle.encode(beam) == jsonpickle.encode(beam_loaded)
    assert id(beam) != id (beam_loaded)

