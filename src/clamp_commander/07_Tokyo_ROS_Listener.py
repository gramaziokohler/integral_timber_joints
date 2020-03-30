import datetime
import json
import time

import roslibpy

hostip = '192.168.43.141'


def current_milli_time(): return int(round(time.time() * 1000))


client = roslibpy.Ros(host=hostip, port=9090)
client.run()

trip_times = []


def callback(message_string):
    message = json.loads(message_string['data'])
    # Reply to the message.
    reply_message = {}
    reply_message['sequence_id'] = message['sequence_id']
    reply_message['timestamp'] = current_milli_time()
    replier.publish(roslibpy.Message({'data': json.dumps(reply_message)}))
    # Print it to UI and keep track of one way latency.
    print(message)
    trip_time = current_milli_time() - int(message['timestamp'])
    trip_times.append(trip_time)
    print("Trip Time = %s" % trip_time)


# Setup listener and replier topics.
listener = roslibpy.Topic(client, '/clamp_command', 'std_msgs/String')
listener.subscribe(callback)
replier = roslibpy.Topic(client, '/clamp_response', 'std_msgs/String')
replier.advertise()

# Allow user to terminate via CLI
while (client.is_connected):
    i = input("Press x to quit and see average trip time. \n")
    if i == 'x':
        break

print("Total %s messages received, trip time = %s" % (len(trip_times), sum(trip_times) / len(trip_times)))

client.terminate()
