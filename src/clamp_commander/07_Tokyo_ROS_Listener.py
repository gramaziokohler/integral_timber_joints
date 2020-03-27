import time, datetime
import json 
import roslibpy
hostip = '192.168.43.141'
current_milli_time = lambda: int(round(time.time() * 1000))

client = roslibpy.Ros(host=hostip, port=9090)
client.run()

trip_times = []
def callback(message_string):
    message = json.loads(message_string['data'])
    print (message)
    trip_time = current_milli_time() - int(message['timestamp'])
    trip_times.append(trip_time)
    print ("Trip Time = %s" % trip_time)

listener = roslibpy.Topic(client, '/clamp_command', 'std_msgs/String')
listener.subscribe(callback)
    
while (client.is_connected):
    i = input("Press x to quit and see average trip time. \n")
    if i == 'x': break

print ("Total %s messages received, trip time = %s" % (len(trip_times), sum(trip_times) / len(trip_times)))

client.terminate()