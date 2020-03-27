import time, datetime
import json 
import roslibpy
hostip = '192.168.43.141'
current_milli_time = lambda: int(round(time.time() * 1000))

client = roslibpy.Ros(host=hostip, port=9090)
client.run()

# Dummy Instruction parameters
sequence_id = 0
clamp_1_move = True
clamp_2_move = True
position = 95.0
velocity = 2.0

sent_messages = {}
trip_times = []
def callback(message_string):
    receive_time = current_milli_time()
    # Retrive the sent message and compare time
    feedback_message = json.loads(message_string['data'])
    msg_id = feedback_message['sequence_id']
    send_time = int(sent_messages[msg_id]['timestamp'])
    bounce_time = int(feedback_message['timestamp'])

    # Print it to UI and keep track of one way latency.
    rtt = receive_time - send_time
    t1t = bounce_time - send_time
    t2t = receive_time - bounce_time
    print ("Received; %s" % feedback_message)
    print ("Received Message %s, RoundTripTime = %s (%s + %s)" % (msg_id, rtt, t1t, t2t))
    trip_times.append(rtt)


# Setup talker to send message
talker = roslibpy.Topic(client, '/clamp_command', 'std_msgs/String')
talker.advertise()

# Setup listener to listen for feedback
listener = roslibpy.Topic(client, '/clamp_response', 'std_msgs/String')
listener.subscribe(callback)

# CLI Loop
while client.is_connected:
    i = input("Press Enter to Send a test Message topic=/clamp_command , x to quit.\n")
    if i == 'x': break

    # Create message to send
    message = {}
    message['sequence_id'] = sequence_id 
    message['timestamp'] = current_milli_time()
    message['instructions'] = []
    if clamp_1_move:
        message['instructions'].append(('1', position, velocity))
    if clamp_2_move:
        message['instructions'].append(('2', position, velocity))

    talker.publish(roslibpy.Message({'data' : json.dumps(message)}))

    # place message in dict to allow later retrivel for time comparision.
    print(message)
    sent_messages[sequence_id] = message
    #talker.publish(roslibpy.Message(message))
    sequence_id += 1
    #time.sleep(1)

# Wrap up
talker.unadvertise()
client.terminate()