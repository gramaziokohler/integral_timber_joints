import jsonpickle
# import simplejson

load_success = jsonpickle.load_backend('json')
print (load_success)

jsonpickle.set_preferred_backend('json')

json_path_in = "examples/process_design_example/frame_ortho_lap_joints_no_rfl_process.json"
json_path_out = "examples/process_design_example/frame_ortho_lap_joints_no_rfl_prepathplan.json"

# Read process
f = open(json_path_in, 'r')
json_str = f.read()
print ("json_str len:" , len(json_str))
data = jsonpickle.decode(json_str, keys=True)
f.close()

data._clamps = None
data._grippers = None
data.pickup_station = None

f = open(json_path_out, 'w')
json_str = jsonpickle.encode(data, keys=True) # Somehow iron python refuse to deserialize if make_refs = True
print ("json_str len:" , len(json_str))
f.write(json_str)
f.close()

data2 = jsonpickle.decode(json_str, keys=True)
print (data2)
