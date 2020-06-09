import jsonpickle

json_path = "examples/process_design_example/frame_ortho_lap_joints_no_rfl_prepathplan.json"

# Read process
f = open(json_path, 'r')
json_str = f.read()
print ("json_str len:" , len(json_str))
data = jsonpickle.decode(json_str, keys=True)
f.close()

json_str2 = jsonpickle.encode(data, keys=True)
print ("json_str2 len:" , len(json_str2))

data2 = jsonpickle.decode(json_str2, keys=True)

json_str3 = jsonpickle.encode(data2, keys=True)
print ("json_str3 len:" , len(json_str3))

data3 = jsonpickle.decode(json_str3, keys=True)

