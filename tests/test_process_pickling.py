import jsonpickle
import pickle

load_success = jsonpickle.load_backend('json')
print (load_success)

jsonpickle.set_preferred_backend('json')

json_path_in = "examples/process_design_example/frame_ortho_lap_joints_no_rfl_process.json"
pickle_path_out = "examples/process_design_example/frame_ortho_lap_joints_no_rfl_process.pickle"

# Read process
f = open(json_path_in, 'r')
json_str = f.read()
print ("json_str len:" , len(json_str))
data = jsonpickle.decode(json_str, keys=True)
f.close()

# Writes process into pickle format
f = open(pickle_path_out, 'wb')
pickle.dump(data, f, protocol=0)
f.close()

# Reloading the pickle to ensure it is readable
f = open(pickle_path_out, 'rb')
data2 = pickle.load(f) # This is ok
f.close()
