import time, datetime
import json 
import roslibpy
hostip = '192.168.43.141'
current_milli_time = lambda: int(round(time.time() * 1000))

client = roslibpy.Ros(host=hostip, port=9090)
client.run()

talker = roslibpy.Topic(client, '/clamp_command', 'std_msgs/String')
talker.advertise()

sequence_id = 0
clamp_1_move = True
clamp_2_move = True
position = 95.0
velocity = 2.0

while client.is_connected:
    i = input("Press Enter to Send a test Message topic=/clamp_command , x to quit.")
    if i == 'x': break
    message = {}
    message['sequence_id'] = sequence_id 
    message['timestamp'] = current_milli_time()
    message['instructions'] = []
    if clamp_1_move:
        message['instructions'].append(('1', position, velocity))
    if clamp_2_move:
        message['instructions'].append(('2', position, velocity))
    print(message)

    talker.publish(roslibpy.Message({'data' : json.dumps(message) }))
    #talker.publish(roslibpy.Message(message))
    sequence_id += 1
    time.sleep(1)

talker.unadvertise()

client.terminate()