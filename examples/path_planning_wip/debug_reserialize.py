import os
import time

import jsonpickle


def reserialize(file_path_in, file_path_out):

    #########################################################################
    # Load file
    #########################################################################

    ms = time.time()*1000.0  # For timing
    with open(file_path_in, 'r') as f:

        json_str = f.read()
        print("LOAD: json_str len:", len(json_str))
        # type: RobotClampAssemblyProcess
        data = jsonpickle.decode(json_str, keys=True)
    print("    file_path_in = %s" % file_path_in)
    print("    time: %.2fms" % (time.time()*1000.0 - ms))

    #########################################################################
    # Save to file
    #########################################################################

    ms = time.time()*1000.0
    with open(file_path_out, 'w') as f:
        json_str = jsonpickle.encode(data, keys=True)
        print("SAVE: json_str len:", len(json_str))
        f.write(json_str)
    print("    file_path_out = %s" % file_path_out)
    print("    time: %.2fms" % (time.time()*1000.0 - ms))


# Reative file path to this python file
script_dir = os.path.dirname(__file__)

# Reserialize assembly
file_path_in = os.path.join(script_dir, "starting_point_assembly.json")
file_path_out = os.path.join(
    script_dir, "starting_point_assembly.reserial.json.json")
reserialize(file_path_in, file_path_out)

# Reserialize process
file_path_in = os.path.join(script_dir, "starting_point_process.json")
file_path_out = os.path.join(
    script_dir, "starting_point_process.reserial.json.json")
reserialize(file_path_in, file_path_out)
